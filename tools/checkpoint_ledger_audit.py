"""Check exact leaf membership using ordered callback/graph events, not receipt time."""


def check_checkpoint(audit, stamp, ready_ids, inherited_ids, actual_ids, decode_input):
    matches = [i for i, row in enumerate(audit)
               if row['event'] == 'graph_update' and abs(row['stamp'] - stamp) < 1e-8]
    assert len(matches) == 1, 'checkpoint must identify exactly one graph update'
    index = matches[0]
    expected_ids = set()
    for row in audit[:index]:
        assert row['event'] != 'rejected_before_graph', 'clean fixture contains rejected static records'
        if row['event'] == 'input':
            assert row['reason'] == 'QUEUED'
            for physical_id, endpoints in decode_input(row):
                if set(endpoints).issubset(ready_ids) and physical_id not in inherited_ids:
                    expected_ids.add(physical_id)
    assert set(actual_ids) == expected_ids, 'checkpoint leaf identities differ from delivered ready leaves'
    assert len(expected_ids) == audit[index]['fused_leaves'], 'checkpoint graph count differs from exact identities'
    return audit[index]
