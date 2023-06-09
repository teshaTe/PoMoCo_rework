import threading
import queue
import time
import PoMoCoModule


class TestNode(PoMoCoModule.Node):
    def __init__(self):
        super().__init__()
        threading.Thread.__init__(self)
        self._i = 0

    def test_add_note(self, type, message, receiver):
        """

        :param type:
        :param message:
        :param receiver:
        :return:
        """
        self.add_note(type, message, receiver)
        if self.inNoteQueue.empty():
            raise ValueError("The queue is empty after a new node has been added.")

        note = list(self.inNoteQueue.queue)[self._i]
        if note.type != type:
            raise ValueError("Type of the message in the queue does not coincide with the input one.")
        elif note.message != message:
            raise ValueError("The message in the queue does not coincide with the input one.")
        elif note.receiver != receiver:
            raise ValueError("Receiver name in the queue does not coincide with the input one.")

        self._i += 1

    def test_write_send_note(self, type, message, receiver):
        self.write_send_note(type, message, receiver)

        modules = self.modules
        if receiver in modules:
            if modules[receiver].type != type:
                raise ValueError("Type of the message in the queue does not coincide with the input one.")
            elif modules[receiver].message != message:
                raise ValueError("The message in the queue does not coincide with the input one.")
            elif modules[receiver].receiver != receiver:
                raise ValueError("Receiver name in the queue does not coincide with the input one.")
        else:
            raise ValueError("No such entry in the dictionary.")

    def process_note(self, note):
        """

        :param note:
        :return:
        """
        if note.type != "" and note.message != "" and note.receiver!= "":
            print("[INFO] Passed note to process: \n", note.message)
        else:
            print("[WARNING] Empty note has been processed.")

    def test_run(self):
        """

        """
        while True:
            try:
                message = self.inNoteQueue.get(block=False)
                self.process_note(message)
            except queue.Empty:
                print("Queue is empty!")
                break


if __name__ == '__main__':
    test_module = TestNode()

    types = ['type0', 'type1', 'type2', 'type3']
    messages = ['message0', 'message1', 'message2', 'message3']
    receivers = ['receiver0', 'receiver1', 'receiver2', 'receiver3']


    print("Testing add_note() functionality.")
    for t, m, r in zip(types, messages,receivers):
        test_module.test_add_note(t, m, r)
    print("<PASSED>\n")


    print("Testing write_send_note() functionality.")
    for t, m, r in zip(types, messages, receivers):
        test_module.test_write_send_note(t, m, r)
    print("<PASSED>\n")


    print("Testing run() functionality.")
    test_module.test_run()
    print("<PASSED>\n")

