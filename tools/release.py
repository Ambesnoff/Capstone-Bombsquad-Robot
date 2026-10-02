"""Export the current Git release, checked source hashes, and portable history."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
from zipfile import ZIP_DEFLATED, ZipFile

ROOT = Path(__file__).resolve().parents[1]


def run_git(*arguments: str) -> str:
    return subprocess.run(['git', *arguments], cwd=ROOT, check=True,
                          capture_output=True, text=True).stdout.strip()


def source_paths(root: Path) -> list[Path]:
    paths = [root / '.gitignore', root / 'requirements.txt', root / 'robot_protocol.json']
    for pattern in ('*.py', '*.md', '*.example.json'):
        paths.extend(root.glob(pattern))
    for directory in ('hat_firmware', 'tests', 'tools', 'deploy', 'protocol', 'commissioning', '.github', 'releases'):
        base = root / directory
        if base.exists():
            paths.extend(p for p in base.rglob('*') if p.is_file()
                and not any(part in ('__pycache__', 'build', 'runs') for part in p.relative_to(base).parts)
                and p.suffix not in ('.pyc', '.bin', '.elf', '.map', '.csv')
                and p.name != '.DS_Store')
    return sorted(set(p for p in paths if p.is_file()))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--allow-dirty', action='store_true',
                        help='Export an explicitly marked development snapshot instead of a clean release')
    args = parser.parse_args(argv)
    dirty = bool(run_git('status', '--porcelain'))
    if dirty and not args.allow_dirty:
        raise SystemExit('Commit the complete checked source first, or explicitly use --allow-dirty for a development snapshot.')
    paths = source_paths(ROOT)
    hashes = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    identity = hashlib.sha256(json.dumps(hashes, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
    out = ROOT / 'output/releases'
    out.mkdir(parents=True, exist_ok=True)
    manifest = {
        'release_sha256': identity,
        'hardware_qualification': 'NOT_QUALIFIED',
        'git_commit': run_git('rev-parse', 'HEAD'),
        'git_branch': run_git('branch', '--show-current'),
        'git_remote': run_git('remote', 'get-url', 'origin'),
        'development_snapshot': dirty,
        'toolchain': {'board': 'esp32:esp32:esp32', 'arduino_esp32': '3.3.12', 'python': '3.14.8', 'pyserial': '3.5'},
        'files_sha256': hashes,
    }
    bundle = out / 'robot-history.bundle'
    if bundle.exists():
        bundle.unlink()
    run_git('bundle', 'create', str(bundle), '--all')
    archive = out / 'robot-v2-source.zip'
    text = json.dumps(manifest, indent=2) + '\n'
    (out / 'release-manifest.json').write_text(text)
    with ZipFile(archive, 'w', ZIP_DEFLATED) as zipped:
        for source in paths:
            zipped.write(source, source.relative_to(ROOT))
        zipped.writestr('release-manifest.json', text)
    print(f'Source release: {archive}\nGit history: {bundle}\nRelease SHA256: {identity}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
