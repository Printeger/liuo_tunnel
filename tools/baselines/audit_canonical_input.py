"""Reconstruct each original committed converter independently; keep old CDR evidence."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import types

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools'))
from raw_identity import hash_message


def audit(relative):
    run=ROOT/relative
    manifest=json.loads((run/'metadata/run_manifest.json').read_text())
    identity=json.loads((run/'metadata/manifests/baseline_input.json').read_text())
    result=json.loads((run/'metadata/manifests/baseline_result.json').read_text())
    if not manifest['source']['git_worktree_clean']:
        raise RuntimeError('cannot reconstruct an uncommitted converter')
    commit=manifest['source']['git_commit']
    code=subprocess.check_output(['git','show',commit+':tools/baselines/run_external.py'],cwd=ROOT)
    mod=types.ModuleType('original_converter')
    mod.__file__=str(ROOT/'tools/baselines/run_external.py')
    exec(compile(code,mod.__file__,'exec'),mod.__dict__)
    source=Path(identity['source'])
    capture={p.name:hashlib.file_digest(p.open('rb'),'sha256').hexdigest() for p in (source/'export/capture/raw').glob('*.mcap')}
    if capture!=identity['capture_sha256']:
        raise RuntimeError('frozen source capture changed')
    initial,bias,n=mod.startup(source)
    adapter=mod.Adapter(initial,bias)
    stream=mod.reader(source)
    hashes={k:hashlib.sha256() for k in ('imu','cloud')}
    counts=dict(imu=0,cloud=0,clock=0)
    padding=None
    types_map={'/sensors/imu':('imu',mod.Imu),'/sensors/lidar/points':('cloud',mod.PointCloud2),'/clock':('clock',mod.Clock)}
    while stream.has_next():
        topic,data,_=stream.read_next()
        if topic not in types_map:continue
        key,cls=types_map[topic];counts[key]+=1
        if key=='clock':continue
        msg=mod.deserialize_message(data,cls)
        msg=adapter.imu(msg) if key=='imu' else adapter.cloud(msg)
        hash_message(hashes[key],key,msg)
        if key=='cloud' and padding is None:
            serializations=[mod.serialize_message(msg) for _ in range(30)]
            same=all(mod.deserialize_message(s,cls)==msg for s in serializations)
            if not same:raise RuntimeError('CDR changed message semantics')
            changed=sorted({i for s in serializations[1:] for i,(a,b) in enumerate(zip(serializations[0],s)) if a!=b})
            padding=dict(unique_cdr_hashes=len({hashlib.sha256(s).hexdigest() for s in serializations}),all_deserialized_messages_equal=same,differing_offsets=changed)
    if counts!=result['counts']:
        raise RuntimeError('original run did not publish the entire reconstructed stream')
    document=dict(scope='post-run deterministic reconstruction, NOT a subscriber-side received-message capture',original_source_commit=commit,original_converter_sha256=hashlib.sha256(code).hexdigest(),capture_sha256=capture,counts=counts,adapted_semantic_sha256={k:h.hexdigest() for k,h in hashes.items()},cdr_padding_probe=padding,original_cdr_sha256=result['adapted_sha256'])
    (run/'metadata/manifests/canonical_input_reconstruction.json').write_text(json.dumps(document,indent=2)+'\n')
    print(relative,document['adapted_semantic_sha256'],flush=True)
    return document


if __name__=='__main__':
    docs=[audit(arg) for arg in sys.argv[1:]]
    assert len(docs)==2
    assert docs[0]['original_converter_sha256']==docs[1]['original_converter_sha256']
    assert docs[0]['adapted_semantic_sha256']==docs[1]['adapted_semantic_sha256']
    print('PASS identical original converter, source capture, counts and reconstructed semantic input')
