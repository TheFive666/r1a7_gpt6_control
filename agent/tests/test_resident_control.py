import tempfile
import unittest
from pathlib import Path
from agent.resident_control import ResidentInbox, atomic_json


class ResidentTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.actions = []
        self.holds = []
        self.inbox = ResidentInbox(self.root, self.execute, lambda: self.holds.append(True))

    def execute(self, request):
        self.actions.append(request['action'])
        if request['action'] == 'fail':
            raise RuntimeError('IK error')
        return {}

    def send(self, number, action):
        atomic_json(self.root / f'{number}.request.json', dict(action=action))
        self.inbox.poll()

    def test_starts_held_and_client_disappearance_does_not_release(self):
        self.send(1, 'move')
        self.assertEqual(self.actions, [])
        self.send(2, 'resume')
        self.send(3, 'move')
        for _ in range(10):
            self.inbox.poll()
        self.assertEqual(self.actions, ['resume', 'move'])
        self.assertFalse(self.inbox.halted)

    def test_error_latches_and_resume_is_explicit(self):
        self.send(1, 'resume')
        self.send(2, 'fail')
        self.send(3, 'move')
        self.assertTrue(self.inbox.halted)
        self.assertEqual(self.actions, ['resume', 'fail'])
        self.send(4, 'resume')
        self.send(5, 'move')
        self.assertEqual(self.actions[-1], 'move')

    def test_completed_and_interrupted_commands_never_replay(self):
        self.send(1, 'resume')
        self.send(2, 'move')
        restarted = ResidentInbox(self.root, self.execute, lambda: None)
        restarted.poll()
        self.assertEqual(self.actions.count('move'), 1)
        atomic_json(self.root / '3.request.json', dict(action='move'))
        (self.root / '3.claimed').touch()
        restarted.poll()
        self.assertEqual(self.actions.count('move'), 1)
        self.assertTrue(restarted.halted)
