import json
from pathlib import Path
import tempfile
import unittest

from voice_models import digest, load_manifest, verify_files


class VoiceModelTest(unittest.TestCase):
    def test_manifest_and_runtime_files_are_hash_checked(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            model = root / "model"
            model.mkdir()
            (model / "runtime.bin").write_bytes(b"verified runtime")
            checksum = digest(model / "runtime.bin")
            manifest_path = root / "manifest.json"
            manifest_path.write_text(json.dumps({
                "schema": "xlerobot_voice_models/v1",
                "kws": {
                    "files": {"runtime.bin": checksum},
                },
                "whisper": {
                    "revision": "1" * 40,
                    "files": {"runtime.bin": checksum},
                },
            }), encoding="utf-8")

            manifest = load_manifest(manifest_path)
            self.assertEqual(verify_files(model, manifest["kws"]["files"]), [])
            (model / "runtime.bin").write_bytes(b"changed")
            self.assertIn(
                "checksum mismatch",
                verify_files(model, manifest["kws"]["files"])[0],
            )


if __name__ == "__main__":
    unittest.main()
