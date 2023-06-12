import sys
import os

root_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.append(root_dir)

import multiprocessing
import threading
import queue
import time

import PoMoCoModule


class Servotor32(PoMoCoModule.Node):
    def __init__(self):
        super().__init__()
        threading.Thread.__init__(self)

        # information about the controller board
        self.firmware_ver = ""
        self.connection_state = False
        self.port_list = []
        self.port = None
        self.module_type = 'controller'
        PoMoCoModule.Node.modules[self.module_type] = self.inNoteQueue

        # variables for controll of the servo motors
        self.servo_pos = {}
        self.servo_offset = {}
        self.servo_active = {}
        self.servo_min_uS = 500
        self.servo_max_uS = 2500
        self.pin_controls_num = 32

        # 32 pins in the board, init. servo motors vars
        for i in range(self.pin_controls_num):
            self.servo_offset[i] = 0
            self.servo_pos[i] = 1500
            self.servo_active[i] = False

        # keep track of servos moves
        self.recording = False
        self.recording_name = ""
        self.recording_servo_move_time = []
        self.recording_servo_move_deg = []
        self.recording_servo_move_num = []

        # automatically starting threading
        self.start()

    def run_node(self):
        """
        A function for executing the node.
        """
        while True:
            try:
                message = self.inNoteQueue.get(block=False)
                self.process_note(message)
            except queue.Empty:
                print("[WARNING] The queue with messages to be processed is empty!")
                time.sleep(0)  # keeps infinite loop from hogging all the CPU

    def process_note(self, note):
        """

        :param note:
        :return:
        """
        if note.type == 'StartRecording':
            self.recording = True
            self.recording_name = note.message

        elif note.type == 'StopRecording':
            self.recording = False

            move_steps = len(self.recording_servo_move_time)
            times = self.recording_servo_move_time
            poses = self.recording_servo_move_deg
            servos = self.recording_servo_move_num
            name = self.recording_name.replace(' ', '_')

            move_name_time = ''
            for i, t in enumerate(times):
                move_name_time += str(int((t - times[0])*1000))
                if i < move_steps-1:
                    move_name_time += ", "

            move_name_servo = ''
            for i, servo in enumerate(servos):
                move_name_servo += str(servo)
                if i < move_steps-1:
                    move_name_servo += ", "

            move_name_pos = ''
            for i, pos in enumerate(poses):
                move_name_pos += str(int(pos) / 10) + " "
                if i < move_steps - 1:
                    move_name_pos += ", "

            arduino_code = self.arduino_move_control_code(move_steps, name, move_name_time, move_name_servo, move_name_pos)

            self.write_send_note('UpdaterduinoCode', arduino_code, 'GUI')
            self.recording_servo_move_time = []
            self.recording_servo_move_deg = []
            self.recording_servo_move_num = []

        elif note.type == 'RequestDisableAll':
            for i in range(self.pin_controls_num):
                self.servo_active[i] = True
                self.send_servo_state(i)

        elif note.type == 'RequestCenterAll':
            for i in range(self.pin_controls_num):
                self.servo_pos[i] = 1500
            self.write_send_note("SendMessage", "C\n", "comms")

        elif note.type == 'RequestConnectPort':
            self.port = note.message
            self.write_send_note("RequestConnectPort", note.message, "comms")

        elif note.type == 'RequestAutoConnect':
            self.write_send_note("RequestAutoConnect", "SERVOTOR", "comms")

        elif note.type == 'RequestPortList':
            self.write_send_note("RequestPortList", "", "comms")

        elif note.type == 'SetPortList':
            port_list = str(note.message).split(',')[:]
            self.port_list = port_list
            self.write_send_note("SetPortList", note.message, "GUI")

        elif note.type == 'SetServoPos':
            num, pos = str(note.message).split(',')
            num = int(num)
            pos = float(pos)
            self.servo_pos[num] = pos
            self.send_servo_state(num)

        elif note.type == 'SetFirmwareV':
            self.firmware_ver = note.message
            self.write_send_note("SetFirmwareV", note.message, "GUI")

        elif note.type == 'SetServoOffset':
            num, offset = str(note.message).split(',')
            num = int(num)
            offset = float(offset)
            self.servo_offset[num] = offset
            self.send_servo_state(num)

        elif note.type == 'SetConnectionState':
            conn_state = False
            if note.message == "active":
                conn_state = True
            if note.message == "inactive":
                conn_state = False
            self.connection_state = conn_state
            self.write_send_note("SetConnectionState", note.message.encode(), "GUI")

        elif note.type == 'SetServoActive':
            num, in_state = note.message.split(',')
            num = int(num);
            out_state = False
            if "active" in in_state:
                out_state = True
            if "inactive" in in_state:
                out_state = False
            self.servo_active[num] = out_state
            self.send_servo_state(num)

        else:
            raise ValueError("Unknown message type!")

    def send_servo_state(self, state):

        # send raw command to comms
        out_pos = self.servo_pos[state]
        out_pos += self.servo_offset[state]  # add offset
        out_pos = out_pos * (1000.0 / 90.0) + 1500  # convert servo deg to uS pulse length

        # keep within pulse min/max
        if out_pos > self.servo_max_uS:
            out_pos = self.servo_max_uS
        if out_pos < self.servo_min_uS:
            out_pos = self.servo_min_uS

        if self.recording:
            if self.servo_active[state]:
                self.recording_servo_move_time.append(time.clock())
                out_pos = self.servo_pos[state]
                out_pos += self.servo_offset[state]  # add offset
                out_pos = out_pos * (1000.0 / 90.0) + 1500  # convert servo deg to uS pulse length
                self.recording_servo_move_deg.append(out_pos)
                self.recording_servo_move_num.append(state)

        if self.servo_active[state]:
            self.write_send_note("SendMessage", str("#%02dP%04d\n" % (state, out_pos)), "comms")
        else:
            self.write_send_note("SendMessage", str("#%02dL\n" % (state)), "comms")

    @staticmethod
    def arduino_move_control_code(move_steps, name, move_name_time_str, move_name_servo_str, move_name_pos_str):
        arduino_code = ""
        arduino_code += "#include <str/pgmspace.h\n"
        arduino_code += "#define MOVE_"+name.upper()+"_SIZE "+str(move_steps)+"\n\n"
        arduino_code += "const PROGMEM uint16_t move_" + name + "_time[MOVE_" + name.upper() + "_SIZE] = {" + "\n"
        arduino_code += move_name_time_str + "};\n\n"
        arduino_code += "const PROGMEM uint8_t move_" + name + "_servo[MOVE_" + name.upper() + "_SIZE] = {" + "\n"
        arduino_code += move_name_servo_str + "};\n\n"
        arduino_code += "const PROGMEM uint8_t move_" + name + "_pos[MOVE_" + name.upper() + "_SIZE] = {" + "\n"
        arduino_code += move_name_pos_str + "};\n\n"
        arduino_code += "" + "\n"
        arduino_code += "void move_" + name + "(){\n"
        arduino_code += "  int startTime = hexy.millis_new();\n"
        arduino_code += "  int currentTime = 0;\n"
        arduino_code += "  int last_update = 0;\n"
        arduino_code += "  for(int i=0; i<MOVE_" + name.upper() + "_SIZE; i++){\n"
        arduino_code += "    delayMicroseconds(10);\n"
        arduino_code += "    currentTime = hexy.millis_new() - startTime;\n"
        arduino_code += "    uint16_t move_time = pgm_read_word_near(move_" + name + "_time + i);\n"
        arduino_code += "    while(currentTime < move_time){\n"
        arduino_code += "      delayMicroseconds(10);\n"
        arduino_code += "      currentTime = hexy.millis_new() - startTime;\n"
        arduino_code += "    }\n"
        arduino_code += "    uint8_t servo_time = pgm_read_byte_near(move_" + name + "_servo + i);\n"
        arduino_code += "    uint8_t servo_pos  = pgm_read_byte_near(move_" + name + "_pos + i);\n"
        arduino_code += "    hexy.changeServo(servo_time, servo_pos*10);\n"
        arduino_code += "    last_update = currentTime;\n"
        arduino_code += "  }\n"
        arduino_code += "}\n"
        arduino_code += "//Move Size is " + str(move_steps * 4) + " bytes\n"
        arduino_code += "//Run this move by using:\n"
        arduino_code += "// move_" + name + "()"
        return arduino_code