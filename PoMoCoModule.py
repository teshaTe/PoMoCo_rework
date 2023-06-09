import time
import threading
import queue


class Note:
    """
    A class for handling the messages being sent to the controller
    """
    def __init__(self):
        self.sender = ""
        self.receiver = ""
        self.type = ""
        self.message = ""


class Node(threading.Thread):
    """
    A class that creates a queue of nodes with messages to be processed by the controller
    """
    # a dictionary with modules that are created using input notes
    modules = {}

    def __init__(self):
        super().__init__()
        self.inNoteQueue = queue.Queue()
        self.moduleType = ""
        self.NoteTypes = []

    def send_note(self, note):
        """
        A function for  pushing the input note message in the modules dictionary.
        :param note: an object that is defines using Note class. Should contain a message to be pushed to the controller
        """
        Node.modules[note.receiver].put(note)

    def write_send_note(self, type, message, receiver):
        """
        A function for preparing the input Note structure & storing it in the modules dictionary until it will be sent
        to the controller.
        :param type: a string that contains the type of the message to be sent;
        :param message: a string that contains the message to be sent to the controller;
        :param receiver: a string that contains the name of the object that will receive the message;
        """
        note_to_send = Note()
        note_to_send.type = type
        note_to_send.message = message
        note_to_send.receiver = receiver
        self.send_note(note_to_send)

    def process_note(self, note):
        pass

    def add_note(self, note):
        """
        A function for putting a note in the queue for further processing
        :param note: an object that is defines using Note class. Should contain a message to be pushed to the controller
        """
        print("[INFO] Current queue of the messages to be processed: \n", self.inNoteQueue)
        print("\n[INFO] Adding new note: ", note)
        self.inNoteQueue.put(note)

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
                pass
            time.sleep(0)  # keeps infinite loop from hogging all the CPU

