"""Offline oracle-anchor contract. Never imported by the estimator itself."""
import copy


def vehicle_condition_config(original):
    if original['uwb']['mode'] != 'self_calibrate_free':
        raise ValueError('D2 requires the frozen unknown-anchor CAL configuration')
    result = copy.deepcopy(original)
    uwb = result['uwb']
    uwb['mode'] = 'fuse'
    uwb['use_aa'] = False  # fixed-anchor AA has no vehicle derivative
    uwb['local_installation'] = False
    uwb.pop('static_edges', None)
    # Free-calibration already takes bounded bootstrap steps. Preserve that
    # scheduling when invoking the existing fixed-anchor implementation.
    uwb['bounded_bootstrap_work'] = True
    if uwb.get('installation_factors') or uwb.get('continuous_installation_walls'):
        raise ValueError('D2 cannot silently add or remove installation factors')
    result['diagnostic_role'] = 'D2_ORACLE_ANCHORS_NOT_DEPLOYABLE'
    return result


def validate_oracle_map(document, config, raw_sha):
    if document.get('diagnostic_role') != 'D2_ORACLE_ANCHORS_NOT_DEPLOYABLE':
        raise ValueError('explicit independent oracle diagnostic map required')
    if document.get('input_raw_sha256') != raw_sha:
        raise ValueError('oracle map is bound to another raw input')
    if document.get('schema_version') != 'liuo_known_map_v1' or document.get('mode') != 'oracle_map':
        raise ValueError('fixed exact Anchor schema required')
    if any(key in document for key in ('vehicle_pose', 'trajectory', 'joint_covariance_m2', 'installation')):
        raise ValueError('D2 map may not contain vehicle truth, uncertainty or installation priors')
    ids = [row['id'] for row in document['anchors']]
    expected = {row['internal_id'] for row in config['uwb']['input']['nodes'] if row['type'] == 1}
    if len(ids) != len(set(ids)) or set(ids) != expected:
        raise ValueError('oracle Anchor registry mismatch')
    if document['tag_lever_m'] != config['sensors']['T_BU_translation']:
        raise ValueError('oracle tag extrinsic mismatch')
