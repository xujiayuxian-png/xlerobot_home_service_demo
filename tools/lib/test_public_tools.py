"""Small software-only regressions for public tools and documentation."""

from pathlib import Path
import re
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from urllib.parse import unquote, urlsplit

from python_versions import check_versions


ROOT = Path(__file__).resolve().parents[2]


class PublicToolsTest(unittest.TestCase):
    def test_documentation_image_links_and_languages(self):
        # Include package READMEs, examples and assets, not just docs/en.
        files = subprocess.check_output(
            ['git', 'ls-files', '-z', '--cached', '--others', '--exclude-standard'],
            cwd=ROOT, text=True,
        ).split('\0')
        screenshots = {
            'demo-ui', 'mapping-ui', 'collection-ui',
            'calibration-head_camera', 'calibration-right_handeye',
            'calibration-hover',
        }
        for name in screenshots:
            for suffix in ('', '-en'):
                self.assertTrue((ROOT / f'docs/images/{name}{suffix}.png').is_file())
        for name in sorted(set(files)):
            path = ROOT / name
            if (path.suffix not in {'.md', '.mdx', '.html', '.rst'}
                    or name.startswith('third_party/') or not path.is_file()):
                continue
            text = path.read_text(encoding='utf-8')
            # Covers inline images, HTML images and linked image resources.
            references = re.findall(
                r'''(?:\]\(<?([^\s)>]+)|(?:src|href)=["']([^"']+)'''
                r'|^\s*\[[^\]]+\]:\s*<?([^\s>]+)'
                r'|^\s*\.\.\s+(?:image|figure)::\s+(\S+))',
                text, re.MULTILINE,
            )
            for match in references:
                url = urlsplit(next(value for value in match if value))
                if url.scheme or url.netloc:
                    continue
                target = (path.parent / unquote(url.path)).resolve()
                if target.suffix.lower() not in {
                    '.png', '.jpg', '.jpeg', '.svg', '.webp', '.gif', '.pdf',
                }:
                    continue
                with self.subTest(document=name, image=url.path):
                    self.assertTrue(target.is_file(), f'Missing image: {url.path}')
                    # The image catalog intentionally links both languages.
                    if name.startswith(('docs/images/', 'docs/artwork/')):
                        continue
                    stem = target.stem.removesuffix('-en')
                    if stem in screenshots:
                        chinese = name == 'README.zh-CN.md' or name.startswith('docs/zh-CN/')
                        expected = stem if chinese else f'{stem}-en'
                        self.assertEqual(target.stem, expected,
                                         f'Wrong screenshot language in {name}: {url.path}')

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
