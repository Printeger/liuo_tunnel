"""Inventory retained A/B inputs and evidence without deleting any artifacts."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

BASE=Path('/home/dev/ws_iap/.runtime/0/liuo')
DIAGNOSTICS=Path('/home/dev/ws_iap/conditional_diagnostics_20261027')
INPUTS=['20261027T031155Z_389','20261027T031819Z_304','20261027T190229Z_933','20261027T134947Z_968']
REPORTS={
    '20261027T230820Z_728':'export/map/calibrated_map.json',
    '20261027T231612Z_174':'export/analysis/legacy_static_loop_audit.json',
    '20261027T232558Z_196':'export/analysis/conditional_vehicle_audit.json',
    '20261027T234156Z_163':'export/analysis/conditional_vehicle_audit.json',
    '20261027T234440Z_254':'export/analysis/candidate_input_audit.json',
    '20261027T235022Z_325':'export/analysis/candidate_input_audit.json',
    '20261028T002916Z_591':'export/analysis/static_lifecycle_load_audit.json',
    '20261028T003902Z_728':'export/analysis/boundary_input_audit.json',
}


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream,'sha256').hexdigest()


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    assert not args.output.exists(), 'never overwrite an evidence inventory'
    roots=[]
    for name in INPUTS:
        run=BASE/name
        raw=list((run/'export/capture/raw').glob('*.mcap'))
        assert raw and (run/'export/simulation').is_dir()
        capture_manifest=run/'metadata/manifests/captured_inputs.json'
        if capture_manifest.exists():
            captured=json.loads(capture_manifest.read_text())
        else:
            # The independently generated road scene uses its own artifact
            # schema, not the rail capture owner's manifest schema.
            capture_manifest=run/'metadata/manifests/scene_artifacts.json'
            scene=json.loads(capture_manifest.read_text())
            captured={name:value['sha256'] for name,value in scene['files'].items()}
        roots.append(dict(role='retained_original_input',run=str(run),capture_manifest=str(capture_manifest),
            raw=[dict(path=str(f),bytes=f.stat().st_size,recorded_sha256=captured[str(f.relative_to(run))]) for f in raw],
            checksum_scope='Recorded original capture digest; current replay owners separately verified bytes. This inventory checks availability, not a fresh full-bag digest.'))
    for name,required in REPORTS.items():
        run=BASE/name
        assert (run/required).is_file(), str(run/required)
        manifest=json.loads((run/'metadata/run_manifest.json').read_text())
        assert manifest['source']['git_worktree_clean']
        files=[run/'metadata/run_manifest.json']
        files+=list((run/'metadata/manifests').glob('*.json'))
        files+=list((run/'export/analysis').rglob('*'))
        files+=list((run/'export/map').glob('*.json'))
        roots.append(dict(role='engineering_or_diagnostic_run',run=str(run),source=manifest['source'],
            original_acceptance=manifest.get('acceptance'),
            artifacts=[dict(path=str(f),bytes=f.stat().st_size,sha256=digest(f)) for f in sorted(set(files)) if f.is_file()]))
    for name in ('S_observations.json','L_observations.json','S_oracle_anchors.json','L_oracle_anchors.json'):
        assert (DIAGNOSTICS/name).is_file()
    for name in ('S_D1_234440_normalized','L_D1_235022'):
        folder=DIAGNOSTICS/name
        assert (folder/'report.json').is_file() and len(list(folder.glob('candidate_*.json')))==32
        assert list(folder.glob('*.png'))
    files=[f for f in DIAGNOSTICS.rglob('*') if f.is_file()]
    roots.append(dict(role='oracle_conditional_only_never_deploy',directory=str(DIAGNOSTICS),
        artifacts=[dict(path=str(f),bytes=f.stat().st_size,sha256=digest(f)) for f in sorted(files)]))
    result=dict(scope='revised A engineering and B conditional diagnostics',
        source_commit=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        worktree_clean=not subprocess.check_output(['git','status','--porcelain'],text=True).strip(),
        result='PASS_REQUIRED_ARTIFACTS_AVAILABLE',roots=roots,
        disk_free_bytes=shutil.disk_usage(BASE).free,
        retention='No active input, failed run, required truth, map, candidate or report retired. New bounded runs reused existing raw MCAP without making duplicate captures.',
        localization_effectiveness='NOT_ACCEPTED')
    assert result['worktree_clean']
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(result=result['result'],output=str(args.output),roots=len(roots),disk_free_bytes=result['disk_free_bytes'])))


if __name__=='__main__':main()
