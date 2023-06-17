import sys
import os

root_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.append(root_dir)

import threading
import serial
import queue
import time

# chose an implementation depending on the OS
if os.name == 'nt':
    from serial.tools.list_ports_windows import *
elif os.name == 'posix':
    from serial.tools.list_ports_posix import *
else:
    raise ImportError("Sorry: no implementation for your platform ('%s') available" % (os.name,))

import PoMoCoModule


class SerialLink(PoMoCoModule.Node):
    """
    This class deals with serial ports: searching for the available ports, connecting to the selected port,
    sending/receiving information from the controller.
    """
    def __init__(self):
        super().__init__()
        threading.Thread.__init__(self)

        self.module_type = 'comms'
        self.baud_rate = 9600
        self.timeout_period = 0

        self.connected = False
        self.debug = True

        self.ports = None
        self.ser = None
        self.connected_port = None

        self.serial_incoming = []
        self.last_received = ""
        self.buffer = ""
        self.auto_conn_str = ""

        self.port_priority = ["VID:PID=2341:8036", "USB", "BT", ""]
        self.priority_desc = ["Servotor32 (VID/PID matched)",
                              "USB Serial device",
                              "Windows Bluetooth Serial device",
                              "Unknown Type Serial Port"]

        PoMoCoModule.Node.modules[self.module_type] = self.inNoteQueue
        self.start()

    def __del__(self):
        """
         A function for closing the open serial port.
        """
        self.ser.close()

    def run(self):
        """

        """
        start_time = time.process_time()
        while True:
            if time.process_time() - start_time >= 1.0:
                self.port_refresh()
                start_time = time.process_time()
            try:
                note = self.inNoteQueue.get(block=False)
                self.process_note(note)
            except queue.Empty:
                self.read_incoming_serial()
                self.process_serial()
            time.sleep(0)

    def port_refresh(self):
        """
        A function for refreshing the list of the available ports and send them to a controller.
        """
        self.debug = False
        old_ports = []
        if self.ports:
            old_ports = self.ports[:]

        self.scan_for_ports()

        if self.ports != old_ports:
            if self.debug:
                print("[INFO] Previous set of ports does not coincide with new one!")

            port_list = ""
            for port in self.ports:
                port_list += port['name'] + ','

            if len(port_list) > 0:
                port_list = port_list[:-1]  # remove last comm

            self.write_send_note('SelfPortList', port_list, 'controller')

    def process_note(self, note):
        """
        A function for processing the input note object depending on the type of the message to be executed.
        :param note: a Node object with message to be sent to the controller
        """
        if note.type == 'RequestConnectPort':
            connect_port = note.message
            self.connect(connect_port)

        elif note.type == 'RequestAutoConnect':
            self.auto_conn_str = note.message
            self.auto_connect()

        elif note.type == 'SendMessage':
            self.send_serial(note.message)

        elif note.type == 'RequestPortList':
            print('[INFO] Comms received port list request.')
            self.scan_for_ports()
            port_list = ''

            for port in self.ports:
                port_list += port['name'] + ','

            if len(port_list) > 0:
                port_list = port_list[:-1]
            self.write_send_note('SetPortList', port_list, note.sender)

    def process_serial(self):
        """
        A function for keeping track of the processed serial ports
        """
        while len(self.serial_incoming) > 0:
            print("[INFO] From controller: ", self.serial_incoming.pop())

    def read_incoming_serial(self):
        """
        A function for reading in the data from the serial port.
        :return: True if data has been successfully received, otherwise - False
        """
        if self.connected and self.ser:
            try:
                self.buffer = self.buffer + str(self.ser.read(self.ser.inWaiting()))
            except IOError as detail:
                print("[ERROR] Serial connection unexpectedly terminated: ", detail)
                self.ser = None
                self.connected = False
                self.write_send_note('SetFirmwareV', '?', 'controller')
                self.write_send_note('SetConnectionState', 'inactive', 'controller')

            if '\n' in self.buffer:
                lines = self.buffer.split('\n')
                last_received = lines[:-2]
                self.serial_incoming.append(last_received)
                self.buffer = lines[-1]
                return True
        return False

    def send_serial(self, to_send):
        """
        A function for sending the message tp serial port
        :param to_send: a string value to be sent
        :return: True if data has been successfully sent, otherwise - False
        """
        if self.ser:
            if self.ser.writable:
                print("writing to the serial port...")
                self.ser.write(str.encode(to_send))
                return True
        return False

    def scan_for_ports(self):
        """
        A function for scanning the system for the available serial ports.
        """
        unsorted_ports = []
        for port_path, desc, hwid in comports():
            port_path = port_path.replace("/dev/cu.", "/dev/tty.", 1)
            port = {"name": port_path, "desc": desc, "hwid": hwid}
            unsorted_ports.append(port)

        priority_val = 0
        sorted_ports = []
        while (len(unsorted_ports) > 0) and (priority_val < len(self.port_priority)):
            for port in unsorted_ports:
                if self.port_priority[priority_val] in port['hwid']:
                    if port not in sorted_ports:
                        sorted_ports.append(port)
                    else:
                        pass
            priority_val += 1

        if len(sorted_ports) > 0:
            self.ports = sorted_ports
            if self.debug:
                print("Available Serial Ports:")
                print("-"*20)
            for port in self.ports:
                if self.debug:
                    print("Name:",port['name'])
                    print("Description:",port['desc'])
                    print("Hardware ID:",port['hwid'])
                    print("-"*20)
        else:
            if self.debug:
                print ("No serial ports available!")
                print ("Are the drivers installed?")
                print ("Device powered/plugged in?")
                print ("Bluetooth paired?")
            self.ports = None

    def auto_connect(self):
        """
        A function for establishing automatic connection to the found available serial port.
        :return: True if connection has been successfully established, otherwise - False
        """
        if self.connected:
            print("Already connected")
            return True
        self.scan_for_ports()
        if self.ports:
            for port in self.ports:
                print("[INFO] Trying", port['name'])
                self.connect(port['name'])
                if self.connected:
                    print("[INFO] Connected.")
                    return True
        return None

    def connect(self, port_name):
        """
        A function for establishing connection to the specified port
        :param port_name: an integer value defining the port
        :return: True if connection has been successfully established, otherwise - False
        """
        self.connected = False
        self.ser = None

        try:
            self.ser = serial.Serial(port_name, baudrate=self.baud_rate, timeout=self.timeout_period,
                                     writeTimeout=self.timeout_period)

            # if the serial port is in use
            if not self.ser.isOpen():
                print("[INFO] Connection failed: <", port_name, "> has already been opened by another process.")
                self.ser = None
                return False

            # connect the serial port
            self.ser.flush()
            time.sleep(0.1)

            self.send_serial('V\n')
            time.sleep(1)
            result = self.ser.readline()

            if self.auto_conn_str.encode() in result:
                firmware = result.rstrip('\n\r'.encode())
                print("[INFO] Connected to port: ", port_name)
                print("[INFO} Current firmware: ", firmware)

                self.write_send_note('SelfFirmwareV', firmware, 'controller')
                self.ser.flush()
                self.connected = True
                self.write_send_note('SetConnectionState', 'active', 'controller')
            else:
                self.ser = None
                print("[WARNING] Device is not ON, port name: ", port_name)

        except serial.serialutil.SerialException as detail:
            self.ser = None
            raise Exception("[ERROR] Serial exception: ", port_name, "; ", sys.exc_info())
        except:
            self.ser = None
            raise Exception("[ERROR] Connection failed: ", port_name, "; ", sys.exc_info())
