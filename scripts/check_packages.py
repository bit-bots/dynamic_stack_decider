"""Build distributions and check their contents without modifying the environment."""

import subprocess
import sys
import tempfile
import zipfile
from pathlib import Path


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    for name in ("dynamic_stack_decider", "dynamic_stack_decider_visualization"):
        with tempfile.TemporaryDirectory() as directory:
            subprocess.run(
                [sys.executable, "-m", "build", "--no-isolation", "--outdir", directory, str(root / name)],
                check=True,
            )
            wheel = next(Path(directory).glob("*.whl"))
            with zipfile.ZipFile(wheel) as archive:
                files = archive.namelist()
                assert f"{name}/__init__.py" in files
                assert not any(path.startswith(("test/", "tests/")) for path in files)
                assert any(path.endswith("/package.xml") for path in files)
                if name.endswith("visualization"):
                    assert any(path.endswith("/resource/StackmachineViz.ui") for path in files)
                    assert any(path.endswith("/plugin.xml") for path in files)


if __name__ == "__main__":
    main()
