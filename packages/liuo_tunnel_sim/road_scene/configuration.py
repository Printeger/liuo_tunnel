"""Frozen v1 geometry/scanning profiles fail closed when edited inconsistently."""


def validate_configuration(c):
  fixed = {
      'route_length_m': 7000,
      'lidar': {
          'hz': 10,
          'columns': 512,
          'rings': 40,
          'vertical_deg': [-25, 25],
          'range_m': 40,
          'sigma_m': .02
      },
  }
  for key, value in fixed.items():
    if c[key] != value:
      raise ValueError(f'unsupported frozen v1 {key}; change model and independent fixtures together')
  for key, value in dict(range_m=800, ta_hz=20, aa_hz=1, report_hz=5, budget_hz=250, delivery_s=.012, assembly_s=.001, endpoint_tolerance_m=.002, antenna_inset_m=.08).items():
    if c['uwb'][key] != value:
      raise ValueError(f'unsupported frozen UWB {key}')
  geometry = dict(name='symmetric_road_7km_v1',
                  lane_width_m=3.75,
                  lateral_clearance_m=.75,
                  walkway_m=.75,
                  basic_clearance_width_m=10.5,
                  wall_half_width_m=5.6,
                  wall_spring_height_m=5.2,
                  crown_height_m=7.2,
                  grade=.005,
                  crossfall=.02,
                  radius_m=3000,
                  clothoid_m=150,
                  circular_m=1200,
                  bend_start_stations_m=[1000, 4000],
                  crown_panels=64)
  vehicle = dict(name='continuous_chassis_offset_v1',
                 lane_offset_m=-1.875,
                 size_m=[4.6, 1.8, 1.5],
                 sensor_height_at_portal_m=1.7,
                 max_passive_height_change_m=.00421875,
                 friction_coefficient=.6,
                 speed_limit_mps=80 / 3.6,
                 acc_limits_mps2=[-1.5, 1],
                 jerk_limit_mps3=.5,
                 holds_s=5,
                 speed_targets_kmh=[60, 70, 80, 60, 70, 80, 0])
  for name, profile in [('geometry_profile', geometry), ('vehicle_profile', vehicle)]:
    for key, value in profile.items():
      if c[name][key] != value:
        raise ValueError(f'unsupported frozen {name}/{key}')
  if not 0 <= c['seed'] < 2**64:
    raise ValueError('seed outside uint64')
  if c['mesh_step_m'] not in [.5, 1.]:
    raise ValueError('unvalidated mesh resolution')
