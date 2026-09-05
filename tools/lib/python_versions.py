"""Read-only dependency checks for doctor, including PyTorch's CUDA suffix."""

from importlib import import_module
from importlib.metadata import PackageNotFoundError, version
import platform
import sys


def check_versions(expected_python, packages):
    errors = []
    if expected_python and platform.python_version() != expected_python:
        errors.append(f"python={platform.python_version()} (expected {expected_python})")
    for package, expected in packages:
        try:
            actual = (
                import_module(package).__version__
                if package in ('torch', 'torchvision') else version(package)
            )
        except PackageNotFoundError:
            errors.append(f"{package}=missing (expected {expected})")
        except (ImportError, OSError, RuntimeError, AttributeError) as error:
            if isinstance(error, ModuleNotFoundError) and error.name == package:
                errors.append(f"{package}=missing (expected {expected})")
            else:
                reason = ' '.join(str(error).splitlines())
                errors.append(f"{package} import failed: {reason}")
        else:
            if actual != expected:
                errors.append(f"{package}={actual} (expected {expected})")
    return errors


def main(argv=None):
    arguments = sys.argv[1:] if argv is None else argv
    if not arguments or len(arguments) % 2 != 1:
        raise SystemExit('expected Python version followed by package/version pairs')
    errors = check_versions(arguments[0], zip(arguments[1::2], arguments[2::2]))
    if errors:
        raise SystemExit('; '.join(errors))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
