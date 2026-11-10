import copy
import struct
import unittest
from types import SimpleNamespace as NS

from uav_inference.yopo_contract import request_payload, validate_result


def observation():
    stamp = NS(sec=10, nanosec=0)
    header = NS(stamp=stamp, frame_id="planning")
    return NS(
        request_id="test", state_header=header, observation_header=header,
        depth=NS(header=NS(stamp=stamp, frame_id="depth_optical"), width=2, height=1,
                 encoding="16UC1", is_bigendian=True, step=6,
                 data=struct.pack(">HH", 1000, 2000) + b"xx"),
        velocity=NS(x=1., y=2., z=3.), acceleration=NS(x=0., y=0., z=0.),
        relative_goal=NS(x=10., y=0., z=0.),
    )


class ContractTest(unittest.TestCase):
    def test_padded_bigendian_depth_in_meters(self):
        payload = request_payload(observation(), 10000000000, 0.5, 0.05)
        self.assertEqual(payload["depth"], [[1., 2.]])
        self.assertEqual(payload["state"], [1., 2., 3., 0., 0., 0., 10., 0., 0.])

    def test_stale_skew_rejected_and_zero_depth_preserved(self):
        request = observation()
        with self.assertRaises(ValueError):
            request_payload(request, 11000000000, 0.5, 0.05)
        request.observation_header = copy.deepcopy(request.observation_header)
        request.observation_header.stamp.nanosec = 60000000
        with self.assertRaises(ValueError):
            request_payload(request, 10060000000, 0.5, 0.05)
        request = observation()
        request.depth.data = struct.pack(">HH", 0, 1000) + b"xx"
        self.assertEqual(request_payload(request, 10000000000, 0.5, 0.05)["depth"][0][0], 0.)

    def test_future_reference_bounded_independently(self):
        request = observation()
        request.state_header = copy.deepcopy(request.state_header)
        request.state_header.stamp.nanosec = 200000000
        request_payload(request, 10000000000, .5, .05)
        request.state_header.stamp.nanosec = 400000000
        with self.assertRaises(ValueError):
            request_payload(request, 10000000000, .5, .05)

    def test_float_depth_invalid_values_are_transport_safe(self):
        request = observation()
        request.depth.width, request.depth.step = 4, 16
        request.depth.encoding, request.depth.is_bigendian = '32FC1', False
        request.depth.data = struct.pack('<4f', float('nan'), float('inf'), -float('inf'), -1.)
        payload = request_payload(request, 10000000000, .5, .05)
        self.assertEqual(payload['depth'], [[0.,20.,0.,-1.]])
        import json
        json.dumps(payload, allow_nan=False)

    def test_future_measurement_rejected_with_age_limit_disabled(self):
        request = observation()
        with self.assertRaises(ValueError):
            request_payload(request, 9999999999, 0., .05)

    def test_result_mismatch_and_nonfinite_rejected(self):
        result = dict(request_id="test", endstates=[[0.] * 9], scores=[1.],
                      best_index=0, duration_s=1., model_version="v1", device="cpu")
        validate_result(result, "test")
        with self.assertRaises(ValueError):
            validate_result(result, "other")
        result["scores"] = [float("nan")]
        with self.assertRaises(ValueError):
            validate_result(result, "test")


if __name__ == "__main__":
    unittest.main()
