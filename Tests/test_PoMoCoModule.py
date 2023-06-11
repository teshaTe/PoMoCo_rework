import sys
import os

root_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.append(root_dir)

import PoMoCoModule

nodes = PoMoCoModule.Node()
types = ['type0', 'type1', 'type2', 'type3']
messages = ['message0', 'message1', 'message2', 'message3']
receivers = ['receiver0', 'receiver1', 'receiver2', 'receiver3']


def test_add_note():
    for t, m, r, i in zip(types, messages, receivers, range(0, len(messages))):
        nodes.add_note(t, m, r)
        assert not nodes.inNoteQueue.empty()

        note = list(nodes.inNoteQueue.queue)[i]
        assert note.type == t
        assert note.message == m
        assert note.receiver == r


def test_write_send_note():
    for t, m, r in zip(types, messages, receivers):

        nodes.write_send_note(t, m, r)

        modules = nodes.modules
        assert r in modules
        assert modules[r].type == t
        assert modules[r].message == m
        assert modules[r].receiver == r
