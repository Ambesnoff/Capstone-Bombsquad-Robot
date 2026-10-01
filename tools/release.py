"""Create a source release and a portable Git history without moving working files."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
from zipfile import ZIP_DEFLATED, ZipFile

ROOT = Path(__file__).resolve().parents[1]


def source_paths(root: Path) -> list[Path]:
    paths = [root / ".gitignore", root / "requirements.txt"]
    for pattern in ("*.py", "*.md", "*.example.json"):
        paths.extend(root.glob(pattern))
    for directory in ("hat_firmware", "tests", "tools", "deploy", "protocol", "commissioning"):
        base = root / directory
        if base.exists():
            paths.extend(p for p in base.rglob("*") if p.is_file()
                         and "__pycache__" not in p.parts and "runs" not in p.relative_to(base).parts
                         and p.suffix not in (".pyc", ".bin", ".elf", ".map", ".csv")
                         and p.name != ".DS_Store")
    return sorted(set(p for p in paths if p.is_file()))


def run_git(*arguments: str, cwd: Path | None = None) -> str:
    return subprocess.run(["git", *arguments], cwd=cwd, check=True, capture_output=True, text=True).stdout.strip()


def commit_snapshot(path: Path, message: str) -> str:
    run_git("add", "--all", cwd=path)
    run_git("-c", "user.name=Robot Release", "-c", "user.email=robot-release@localhost",
            "commit", "--allow-empty", "-m", message, cwd=path)
    return run_git("rev-parse", "HEAD", cwd=path)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--canonical-repo", type=Path,
                        help="Create a new bare Git repository outside the sync mirror; refuses an existing destination")
    args = parser.parse_args(argv)
    paths = source_paths(ROOT)
    hashes = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    identity = hashlib.sha256(json.dumps(hashes, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
    out = ROOT / "output/releases"
    out.mkdir(parents=True, exist_ok=True)
    manifest = {"release_sha256": identity, "hardware_qualification": "NOT_QUALIFIED",
                "toolchain": {"board": "esp32:esp32:esp32", "arduino_esp32": "3.3.12", "python_minimum": "3.10"},
                "files_sha256": hashes}
    baseline = out / "robot-v1-baseline.zip"
    bundle = out / "robot-history.bundle"
    archive = out / "robot-v2-source.zip"
    with tempfile.TemporaryDirectory(prefix="robot-release-") as temporary:
        checkout = Path(temporary) / "checkout"
        checkout.mkdir()
        run_git("init", "-b", "main", str(checkout))
        if baseline.exists():
            with ZipFile(baseline) as saved:
                for entry in saved.infolist():
                    relative = Path(entry.filename)
                    if relative.is_absolute() or ".." in relative.parts:
                        raise ValueError("Unsafe baseline archive member")
                saved.extractall(checkout)
            commit_snapshot(checkout, "Preserve original v1 fast and factory-firmware baseline")
            run_git("tag", "robot-v1-baseline", cwd=checkout)
        for item in list(checkout.iterdir()):
            if item.name != ".git":
                if item.is_dir(): shutil.rmtree(item)
                else: item.unlink()
        for source in paths:
            target = checkout / source.relative_to(ROOT)
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, target)
        manifest["git_commit"] = commit_snapshot(checkout, "Implement protocol v2 modes, protections, telemetry and commissioning tools")
        run_git("tag", "robot-v2-commissioning", cwd=checkout)
        if bundle.exists(): bundle.unlink()
        run_git("bundle", "create", str(bundle), "--all", cwd=checkout)
        if args.canonical_repo is not None:
            destination = args.canonical_repo.expanduser().resolve()
            if destination == ROOT or ROOT in destination.parents:
                raise ValueError("Canonical repository must be outside the sync mirror")
            if destination.exists():
                raise ValueError("Canonical destination already exists; refusing to replace it")
            destination.parent.mkdir(parents=True, exist_ok=True)
            run_git("clone", "--bare", str(checkout), str(destination))
            print(f"Canonical Git repository: {destination}")
    (out / "release-manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    with ZipFile(archive, "w", ZIP_DEFLATED) as zipped:
        for source in paths: zipped.write(source, source.relative_to(ROOT))
        zipped.writestr("release-manifest.json", json.dumps(manifest, indent=2) + "\n")
    print(f"Source release: {archive}\nGit history: {bundle}\nRelease SHA256: {identity}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
