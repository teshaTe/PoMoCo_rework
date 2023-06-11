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
        self.module_type = 'controller'
        PoMoCoModule.Node.modules[self.module_type] = self.inNoteQueue

        # variables for controll of the servo motors
        self.servo_pos = {}
        self.servo_offset = {}
        self.servo_active = {}
        self.servo_min_uS = 500
        self.servo_max_uS = 2500

        # 32 pins in the board, init. servo motors vars
        for i in range(32):
            self.servo_offset[i] = 0
            self.servo_pos[i] = 1500
            self.servo_active[i] = False

        # keep track of servos moves
        self.recording = False
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
    def process_note(self, message):
        if message.type == 'StartRecording':
            pass

        if message.type == 'StopRecording':
            pass

        if message.type == 'RequestDisableAll':
            pass

        if message.type == 'RequestCenterAll':
            pass

        if message.type == 'RequestConnectPort':
            pass

        if message.type == 'RequestAutoConnect':
            pass

        if message.type == 'RequestPortList':
            pass

        if message.type == 'SetPortList':
            pass

        if message.type == 'SetServoPos':
            pass

        if message.type == 'SetFirmwareV':
            pass

        if message.type == 'SetServoOffset':
            pass

        if message.type == 'SetConnectionState':
            pass

        if message.type == 'SetServoActive':
            pass

    def send_servo_state(self, state):
        pass

    def arduino_move_control_code(self, move_steps, name, move_name_time_str, move_name_servo_str, move_name_pos_str):
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