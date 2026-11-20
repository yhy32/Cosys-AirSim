"""Run the repository's Hybrid demonstration."""

import runpy
import sys
from pathlib import Path


if __name__ == "__main__":
    client_dir = Path(__file__).resolve().parents[1] / "PythonClient"
    sys.path.insert(0, str(client_dir))
    runpy.run_path(str(client_dir / "hybrid_demo.py"), run_name="__main__")
