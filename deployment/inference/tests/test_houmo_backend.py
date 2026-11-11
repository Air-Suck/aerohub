import tempfile
import unittest
from pathlib import Path

import numpy as np

from aerohub_houmo import HmmBackend


class FakeInfo:
    def __init__(self, shape, dtype):
        self.shape = shape
        self.dtype = dtype
        self.format = "FMT_ND"


class FakeTensor:
    def __init__(self, value):
        self.value = value

    def numpy(self):
        return self.value

    def astype(self, dtype):
        if not isinstance(dtype, type):
            raise ValueError("tcim_lite expects a NumPy scalar type")
        return FakeTensor(self.value.astype(dtype))


class FakeModule:
    def __init__(self):
        self.received = {}
        self.ran = False

    def get_num_inputs(self):
        return 2

    def get_num_outputs(self):
        return 1

    def get_input_name(self, index):
        return ("depth", "obs")[index]

    def get_output_name(self, index):
        return "scores"

    def get_input_info(self, name):
        return FakeInfo([1, 2] if name == "depth" else [1, 1], np.float16)

    def get_output_info(self, name):
        return FakeInfo([1, 1], np.float16)

    def get_model_version(self):
        return "test-model"

    def set_input(self, name, value):
        self.received[name] = value

    def run(self):
        self.ran = True

    def sync(self):
        pass

    def get_output(self, name):
        return FakeTensor(np.array([[self.received["depth"].sum()]], np.float16))


class HmmBackendTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.hmm = Path(self.temporary.name) / "model.hmm"
        self.hmm.write_bytes(b"hmm")
        self.module = FakeModule()
        self.backend = HmmBackend(
            self.hmm,
            "test",
            module_factory=lambda path, device: self.module,
        )

    def tearDown(self):
        self.temporary.cleanup()

    def test_named_inputs_are_cast_and_outputs_are_returned(self):
        outputs, timings = self.backend.infer(
            {
                "depth": np.array([[1.0, 2.0]], dtype=np.float32),
                "obs": np.array([[3.0]], dtype=np.float32),
            },
            output_dtypes={"scores": np.float32},
        )
        self.assertTrue(self.module.ran)
        self.assertEqual(self.module.received["depth"].dtype, np.float16)
        self.assertEqual(outputs["scores"].dtype, np.float32)
        np.testing.assert_allclose(outputs["scores"], [[3.0]])
        self.assertEqual(
            set(timings),
            {"input_transfer", "forward_and_sync", "output_transfer"},
        )

    def test_input_name_and_shape_mismatches_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "missing"):
            self.backend.infer({"depth": np.zeros((1, 2))})
        with self.assertRaisesRegex(ValueError, "shape"):
            self.backend.infer(
                {"depth": np.zeros((1, 3)), "obs": np.zeros((1, 1))}
            )
        with self.assertRaisesRegex(ValueError, "extra"):
            self.backend.infer(
                {
                    "depth": np.zeros((1, 2)),
                    "obs": np.zeros((1, 1)),
                    "other": np.zeros(1),
                }
            )

    def test_close_is_idempotent_and_rejects_execution(self):
        self.backend.close()
        self.backend.close()
        with self.assertRaisesRegex(RuntimeError, "closed"):
            self.backend.infer({"depth": np.zeros((1, 2)), "obs": np.zeros((1, 1))})

    def test_metadata_identifies_hmm_and_tensor_contract(self):
        self.assertEqual(
            self.backend.metadata["model_version"],
            "sha256:" + __import__("hashlib").sha256(b"hmm").hexdigest(),
        )
        self.assertEqual(self.backend.metadata["inputs"]["depth"]["shape"], [1, 2])
        self.assertEqual(self.backend.metadata["outputs"]["scores"]["dtype"], "float16")


if __name__ == "__main__":
    unittest.main()
