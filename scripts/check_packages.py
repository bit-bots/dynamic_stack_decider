"""Build distributions and check their contents without modifying the environment."""

import email
import subprocess
import sys
import tarfile
import tempfile
import xml.etree.ElementTree as ET
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
                assert f"{name}/py.typed" in files
                assert not any(path.startswith(("test/", "tests/")) for path in files)
                package_xml = next(path for path in files if path.endswith("/package.xml"))
                manifest = ET.fromstring(archive.read(package_xml))
                metadata_file = next(path for path in files if path.endswith(".dist-info/METADATA"))
                metadata = email.message_from_bytes(archive.read(metadata_file))
                assert metadata["Version"] == manifest.findtext("version")
                assert any(path.endswith("/LICENSE") for path in files)
                imported = Path(directory) / "imported"
                archive.extractall(imported)
                if name.endswith("visualization"):
                    assert any(path.endswith("/resource/StackmachineViz.ui") for path in files)
                    assert any(path.endswith("/plugin.xml") for path in files)
            subprocess.run(
                [
                    sys.executable,
                    "-I",
                    "-c",
                    "import importlib, pathlib, sys; sys.path.insert(0, sys.argv[1]); "
                    "module = importlib.import_module(sys.argv[2]); "
                    "assert pathlib.Path(module.__file__).is_relative_to(sys.argv[1])",
                    str(imported),
                    name,
                ],
                check=True,
            )
            with tarfile.open(next(Path(directory).glob("*.tar.gz"))) as archive:
                sources = archive.getnames()
                assert any(path.endswith("/README.md") for path in sources)
                assert any(path.endswith("/LICENSE") for path in sources)
                if not name.endswith("visualization"):
                    assert any(path.endswith("/test/test.dsd") for path in sources)
                    assert any(path.endswith("/test/fixture_elements/actions.py") for path in sources)


if __name__ == "__main__":
    main()
