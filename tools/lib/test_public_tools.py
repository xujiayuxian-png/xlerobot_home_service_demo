"""Small software-only regressions for the public setup/doctor entry points."""

from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from python_versions import check_versions


ROOT = Path(__file__).resolve().parents[2]


class PublicToolsTest(unittest.TestCase):
    def test_person_extra_keeps_base_constraints(self):
        extra = ROOT / 'requirements/robot-person-agpl.txt'
        self.assertIn('-c robot.txt', extra.read_text().splitlines())
        self.assertIn('scipy==1.11.4', (extra.parent / 'robot.txt').read_text())

    def test_torch_runtime_version_retains_cpu_or_cuda_suffix(self):
        with patch('python_versions.import_module', return_value=SimpleNamespace(__version__='2.12.0+cpu')):
            self.assertEqual(check_versions('', [('torch', '2.12.0+cpu')]), [])

    def test_missing_torch_in_empty_python_has_no_traceback(self):
        result = subprocess.run(
            [sys.executable, '-S', str(ROOT / 'tools/lib/python_versions.py'), '', 'torch', '2.12.0+cpu'],
            capture_output=True, text=True, check=False,
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('torch=missing', result.stderr)
        self.assertNotIn('Traceback', result.stderr)

    def test_import_failure_identifies_broken_dependency(self):
        for error in (ModuleNotFoundError("No module named 'numpy'", name='numpy'),
                      OSError('missing shared library'), RuntimeError('operator unavailable')):
            with self.subTest(error=error), patch('python_versions.import_module', side_effect=error):
                errors = check_versions('', [('torchvision', '0.27.0+cpu')])
                self.assertIn('torchvision import failed:', errors[0])
                self.assertIn(str(error), errors[0])

    def test_malformed_yaml_is_not_reported_as_missing_pyyaml(self):
        with tempfile.TemporaryDirectory() as temporary:
            config = Path(temporary) / 'local.yaml'
            config.write_text('schema: [\n')
            result = subprocess.run(
                [str(ROOT / 'tools/doctor'), 'robot', '--config', str(config)],
                capture_output=True, text=True, timeout=10, check=False,
            )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('cannot read config', result.stderr)
        self.assertNotIn('PyYAML is required', result.stderr)
        self.assertNotIn('Traceback', result.stderr)


if __name__ == '__main__':
    unittest.main()
