"""Download pinned robot-terrain trajectories and their matching terrain assets."""

import argparse
import zipfile
from pathlib import Path

from huggingface_hub import snapshot_download

REPO = "omniretarget/OmniRetarget_Dataset"
REVISION = "135259130158f1cb1799e4ab9b6566c938285fc9"


def download(output: Path):
    snapshot_download(
        REPO,
        repo_type="dataset",
        revision=REVISION,
        local_dir=output,
        allow_patterns=["README.md", "robot-terrain.zip", "models/g1/g1_29dof_spherehand.urdf", "models/terrain/**"],
    )
    with zipfile.ZipFile(output / "robot-terrain.zip") as archive:
        for member in archive.infolist():
            path = output / member.filename
            if not path.resolve().is_relative_to(output.resolve()):
                raise ValueError(f"Archive member escapes destination: {member.filename}")
            if member.is_dir():
                continue
            if path.exists() and path.stat().st_size == member.file_size:
                continue
            archive.extract(member, output)
    return output


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, default=Path("vendor/omniretarget"))
    download(p.parse_args().output)


if __name__ == "__main__":
    main()
