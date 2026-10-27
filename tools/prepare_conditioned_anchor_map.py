#!/usr/bin/env python3
"""Prepare D2-only fixed Anchor constants. Does not read vehicle trajectory."""
import argparse
import hashlib
import json
from pathlib import Path
from conditional_contract import validate_oracle_map, vehicle_condition_config


def prepare(source, scene, base, output):
    config = vehicle_condition_config(json.loads(base.read_text()))
    if not output.resolve().is_relative_to(Path('/home/dev/ws_iap')):
        raise ValueError('diagnostic artifacts must stay on the data disk')
    if output.exists():
        raise FileExistsError('do not overwrite a frozen oracle diagnostic artifact')
    if scene == 'straight':
        path = Path(__file__).resolve().parents[1]/'packages/liuo_tunnel/config/maps/oracle_map.json'
        fixture = json.loads(path.read_text())
        scenario = json.loads((source/'metadata/config/scenario.json').read_text())
        if scenario['anchor_layout'] != fixture['simulation_anchor_layout']:
            raise ValueError('straight Anchor fixture does not match frozen scenario')
        anchors = fixture['anchors']
        hashes = json.loads((source/'metadata/manifests/captured_inputs.json').read_text())
        raw_sha = hashes['export/capture/raw/raw_0.mcap']
    else:
        path = source/'export/simulation/anchors.json'
        anchors = [dict(id=row['id'], mean_m=row['position']) for row in json.loads(path.read_text())]
        entries = json.loads((source/'metadata/manifests/scene_artifacts.json').read_text())['files']
        raw_sha = entries['export/capture/raw/raw_0.mcap']['sha256']
    document = dict(schema_version='liuo_known_map_v1', mode='oracle_map', units='m', axis_convention='ENU',
                    valid=True, frame='map', map_id='D2_ORACLE_NOT_DEPLOYABLE_'+scene,
                    provenance='synthetic true Anchor constants for independent offline D2 only',
                    coordinate_reference='simulation world; no vehicle pose or heading observation',
                    diagnostic_role='D2_ORACLE_ANCHORS_NOT_DEPLOYABLE', input_raw_sha256=raw_sha,
                    source_anchor_file=str(path), source_anchor_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                    tag_lever_m=config['sensors']['T_BU_translation'], anchors=anchors)
    validate_oracle_map(document, config, raw_sha)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(document, indent=2)+'\n')
    return dict(role=document['diagnostic_role'], path=str(output), anchors=len(anchors),
                sha256=hashlib.sha256(output.read_bytes()).hexdigest(), raw_sha256=raw_sha)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-run', type=Path, required=True)
    parser.add_argument('--scene', choices=['straight', 'road'], required=True)
    parser.add_argument('--base-config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(prepare(args.source_run, args.scene, args.base_config, args.output), indent=2))
