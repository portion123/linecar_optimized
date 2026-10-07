"""Single-command FIX5 verification of the delivered C source.

Run from any directory: python3 /path/to/project/tests/run_verification.py
The original FIX4 entry point is retained in the fix4_baseline Git revision.
"""
from pathlib import Path
import sys


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from fix5_verify import main


if __name__ == '__main__':
    raise SystemExit(main())
