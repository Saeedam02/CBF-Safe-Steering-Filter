"""Regenerate the README visual assets for the default scenario."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from cbf_safe_steering.cli import main

if __name__ == "__main__":
    raise SystemExit(main(["--scenario", "straight", "--output", str(ROOT / "assets" / "generated")]))
