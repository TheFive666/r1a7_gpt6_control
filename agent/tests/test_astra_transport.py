import io
import json
import os
import unittest
from unittest.mock import patch

import numpy as np

from agent.astra_policy import AstraClient, Halt


class TransportTests(unittest.TestCase):
    def test_structured_vision_request_and_no_model_substitution(self):
        response = {'status':'completed', 'output':[{'type':'message', 'content':[{'type':'output_text', 'text':'{"ok": true}'}]}]}
        with patch.dict(os.environ, {'TEST_ASTRA_KEY':'test-only'}), patch('urllib.request.urlopen', return_value=io.BytesIO(json.dumps(response).encode())) as send:
            client = AstraClient('gpt-6-astra', 'https://api.openai.com/v1', 'TEST_ASTRA_KEY')
            self.assertEqual(client.decide({'phase':'wrist'}, [('wrist_right', np.zeros((4,4,3),dtype=np.uint8))]), {'ok':True})
            body = json.loads(send.call_args.args[0].data)
            self.assertEqual(body['model'], 'gpt-6-astra')
            self.assertEqual(body['text']['format']['type'], 'json_schema')
            content = body['input'][0]['content']
            self.assertEqual(sum(b['type'] == 'input_image' for b in content), 1)
            self.assertEqual(content[1]['text'], 'Camera: wrist_right')
            self.assertFalse(body['store'])

    def test_failure_has_no_retry(self):
        with patch.dict(os.environ, {'TEST_ASTRA_KEY':'test-only'}), patch('urllib.request.urlopen', side_effect=TimeoutError) as send:
            with self.assertRaises(Halt):
                AstraClient('gpt-6-astra','https://api.openai.com/v1','TEST_ASTRA_KEY').decide({}, [])
            send.assert_called_once()

    def test_company_endpoint_and_explicit_proxy(self):
        from unittest.mock import Mock
        opener = Mock()
        opener.open.return_value = io.BytesIO(json.dumps({'status':'completed','output':[{'type':'message','content':[{'type':'output_text','text':'{}'}]}]}).encode())
        with patch.dict(os.environ, {'TEST_ASTRA_KEY':'test-only'}), patch('urllib.request.build_opener', return_value=opener), patch('urllib.request.urlopen') as direct:
            client=AstraClient('gpt-6-astra','https://ys3.chunxueai.com/v1','TEST_ASTRA_KEY',proxy='http://127.0.0.1:7897')
            client.decide({},[])
            self.assertEqual(opener.open.call_args.args[0].full_url,'https://ys3.chunxueai.com/v1/responses')
            direct.assert_not_called()


class HandStartupTests(unittest.TestCase):
    def test_right_only_startup_seeds_before_writer_and_does_not_calibrate_left(self):
        from pathlib import Path
        import sys
        sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'third_party/unitree_sdk2_python'))
        from robot.eef.dex1 import Dex1
        initial = []
        def backend(left, right, *args, **kwargs):
            initial.append((left.value, right.value, kwargs['publish_left']))
            from unittest.mock import Mock
            return Mock()
        with patch('robot.eef.dex1.Dex1_1_Gripper_Controller', side_effect=backend), patch.object(Dex1,'calibrate_tau_zero') as calibrate:
            hand = Dex1(initial_ratios=(.4, 0), publish_left=False)
            self.assertEqual(initial, [(3.,5.,False)])
            self.assertEqual(hand.left_hand_pos_array.value, 3.)
            self.assertEqual(hand.right_hand_pos_array.value, 5.)
            calibrate.assert_not_called()


if __name__ == '__main__':
    unittest.main()
