import unittest

from services.act.protocol import (
    authorized,
    bearer_token,
    ProtocolError,
    validate_actions,
    validate_request,
)


class ProtocolTest(unittest.TestCase):
    def test_bearer_auth_is_exact(self):
        self.assertEqual(bearer_token('Bearer token-value'), 'token-value')
        self.assertTrue(authorized('Bearer token-value', 'token-value'))
        self.assertFalse(authorized('Basic token-value', 'token-value'))
        self.assertFalse(authorized('Bearer wrong', 'token-value'))
        self.assertFalse(authorized(None, 'token-value'))

    def test_request_requires_images_and_finite_six_joint_state(self):
        result = validate_request(
            {
                'head_image_base64': 'abc',
                'wrist_image_base64': 'def',
                'state': [0, 1, 2, 3, 4, 5],
            }
        )
        self.assertEqual(result['state'], [0.0, 1.0, 2.0, 3.0, 4.0, 5.0])
        with self.assertRaises(ProtocolError):
            validate_request({'state': [0.0] * 6})
        wrist_only = validate_request(
            {'wrist_image_base64': 'def', 'state': [0.0] * 6},
            image_fields=('wrist_image_base64',),
        )
        self.assertNotIn('head_image_base64', wrist_only)
        with self.assertRaises(ProtocolError):
            validate_request(
                {
                    'head_image_base64': 'abc',
                    'wrist_image_base64': 'def',
                    'state': [0.0] * 5,
                }
            )
        with self.assertRaises(ProtocolError):
            validate_request(
                {
                    'head_image_base64': 'abc',
                    'wrist_image_base64': 'def',
                    'state': [0.0] * 5 + [float('nan')],
                }
            )

    def test_action_chunk_is_exact_finite_and_bounded(self):
        self.assertEqual(
            validate_actions([[0, 1, 2, 3, 4, 5]]),
            [[0.0, 1.0, 2.0, 3.0, 4.0, 5.0]],
        )
        with self.assertRaises(ProtocolError):
            validate_actions([])
        with self.assertRaises(ProtocolError):
            validate_actions([[0.0] * 5])
        with self.assertRaises(ProtocolError):
            validate_actions([[0.0] * 6] * 501)


if __name__ == '__main__':
    unittest.main()

