"""Fetch pinned upstream sources into this project's ignored baseline workspace."""
import argparse
import os
import shutil
import hashlib
import json
from pathlib import Path
import subprocess
import tarfile
import time
import urllib.request

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
SOURCES = Path(os.environ.get('LIUO_DATA_ROOT', '/home/dev/ws_iap/liuo_observation')) / 'baselines/sources'
parser = argparse.ArgumentParser()
parser.add_argument('--algorithm', choices=['fastlio', 'liosam'], default='fastlio')
args = parser.parse_args()
repositories = dict(fastlio='hku-mars/FAST_LIO', liosam='TixiaoShan/LIO-SAM', ikdtree='hku-mars/ikd-Tree')
versions = json.loads((HERE / 'sources.json').read_text())
SOURCES.mkdir(parents=True, exist_ok=True)
for name, repo in repositories.items():
    if name not in ({"fastlio", "ikdtree"} if args.algorithm == "fastlio" else {"liosam"}):
        continue
    commit = versions[name]['commit']
    archive = SOURCES / (name + '-' + commit + '.tar.gz')
    legacy = ROOT / 'log/baseline_sources' / archive.name
    if not archive.exists() and legacy.is_file():
        shutil.copyfile(legacy, archive)
    if not archive.exists():
        url = f'https://codeload.github.com/{repo}/tar.gz/{commit}'
        start = time.monotonic()
        partial = archive.with_suffix('.partial')
        with urllib.request.urlopen(url, timeout=30) as response, partial.open('wb') as out:
            while chunk := response.read(1024 * 1024):
                if time.monotonic() - start > 300:
                    raise RuntimeError('source download exceeded 300 s')
                out.write(chunk)
        partial.rename(archive)
    if hashlib.file_digest(archive.open('rb'), 'sha256').hexdigest() != versions[name]['archive_sha256']:
        raise RuntimeError('archive hash differs from pinned source')
    destination = SOURCES / name if name != 'ikdtree' else SOURCES / 'fastlio/include/ikd-Tree'
    with tarfile.open(archive, 'r:gz') as tar:
        if tar.pax_headers.get('comment') != commit:
            raise RuntimeError('upstream archive commit does not match pinned identity')
        for member in tar.getmembers():
            if '/' not in member.name:
                continue
            member.name = member.name.split('/', 1)[1]
            if member.name:
                tar.extract(member, destination, filter='data')
    if name != 'ikdtree':
        with (HERE / 'patches' / (name + '.patch')).open() as patch:
            subprocess.run(['patch', '--batch', '-p1'], stdin=patch, cwd=destination, check=True, timeout=30)
    print(name, commit, hashlib.file_digest(archive.open('rb'), 'sha256').hexdigest())
