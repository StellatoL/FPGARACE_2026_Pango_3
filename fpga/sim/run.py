"""Project ModelSim entry; forwards options from any working directory."""
from pathlib import Path
import subprocess
import sys

if __name__ == '__main__':
    workspace = Path(__file__).resolve().parents[1]
    raise SystemExit(subprocess.run(
        [sys.executable, '-X', 'utf8', str(workspace / 'tools/avm/run_modelsim.py'), *sys.argv[1:]],
        cwd=workspace).returncode)
