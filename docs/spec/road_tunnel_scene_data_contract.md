# Road tunnel scene data contract (v1)

Scope: independent simulation only. No estimator execution, initialization, mapping,
calibration, deskew output, or localization accuracy acceptance. Old scenarios remain intact.

## Coordinates and geometry

SI units, right-handed world ENU-like axes, gravity (0,0,-9.80665). Design station s is
horizontal axis arc length, 0–7000 m. Positive transverse y is left, measured horizontally
normal to plan tangent; z is vertical above axis grade .005*s. Pavement z=.02*y;
transverse pavement distance is horizontal distance times sqrt(1+.02²). Neither that
slope distance nor a 3D range is an installation lateral offset. Lane center y=-1.875.
Basic clearance width 10.50 m is a symmetric experimental widening of the referenced
10.25 m section; it is not a wall spacing. Walls at ±5.6 m extend to z=5.2 m;
elliptic crown reaches 7.2 m. This contains the full 5 m driving clearance over
±4.5 m and the raised inspection walkways outside it. No whole-engineering certification.

Full route triangle surface, 1 m longitudinal sections, 64 crown panels, open portals,
60 m pavement extensions. Explicit drainage notches, raised walkways, closed recessed
wall surfaces and solid lamp brackets are collision geometry. Facility intervals,
dimensions and recess profiles are experimental parameters, exported as truth inventory.
Mesh first-hit queries use Embree; no world-X cache, occupancy holes, or capped-range returns.
Static full mesh memory is measured. Recesses do not redefine nominal axis.

Vehicle reference and collocated synthetic IMU/LiDAR/UWB phase center are vertically
1.7 m above lane pavement at the portals. Chassis vertical grade is .005 relative to
its offset horizontal arc, allowing 0–4.22 mm passive ride-height variation above
the swept pavement. This keeps p/R/v/a/omega continuous at the prescribed exact
clothoid boundaries (whose curvature derivative jumps); it is not a calibration
oscillation. Body x follows this continuous chassis tangent; body y is
the orthogonalized crossfall direction. Sensor extrinsics are identity. Vehicle envelope
4.6×1.8×1.5 m has top at the reference point; support/clearance checks are separate from
rigid suspension dynamics. No ego vehicle self-returns. Quintic velocity ramps follow
actual lane arc length and use exact integrated distance; start/finish holds >=5 s.
Speed, acceleration and jerk limits are 80 km/h, [-1.5,1], ±.5 in SI units.

## Sensors and clocks

Simulation epoch is ROS 1 s plus elapsed scene seconds, shared exact synthetic clock.
PointCloud2 `/sensors/lidar`, frame `lidar`: x/y/z/intensity float32 at offsets 0/4/8/12,
ring uint16 at 16, padding at 18, time float32 seconds at 20; point_step=24.
10 Hz, 512 columns ×40 rings, azimuth increases from 0, elevation -25…25 degrees.
Ray i=(column*40+ring) emits at frame_start+i/(20480*10). Every ray uses its own
pose; raw xyz is in the instantaneous sensor frame. Header is frame start; bag delivery
is frame end. No-return rays are omitted; ring and relative time retain original indices.
Intensity=1 is a synthetic constant. Gaussian radial noise .02 m is not clipped into
fake maximum-range points. No published truth compensation. This is not a Mid-360 model.

IMU `/sensors/imu`, frame `imu`, 200 Hz: Rᵀ(a-g) and body angular velocity from the
same pose. orientation_covariance[0]=-1, no orientation observation. Independent sample
noise and bias random walks are specified in scene.json; walks scale with sqrt(dt).
IMU spans every scan, including final hold and any rounded scan boundary.
`/clock` follows delivery time. Only these topics plus UWB occur in the raw MCAP.

## UWB

Use original installed liuo_uwb_msgs/msg/UwbObservationBatch on `/liuo/uwb/observations`.
No copied message definitions. Source-frame, receive and batch-publication stamps differ.
Exact shared synthetic time permits valid local/system raw clocks and mapped time_basis;
uncertainty=0 describes the synthetic clock, not radio timing. Unknown RSSI=NaN/false.
Freshness is true only for independently simulated physical exchanges.

18 stations at 100+400k, alternating walls, heights 2.5/3.5 m above grade datum.
Antenna phase centers inset .08 m from wall: signed horizontal offset ±5.52 m.
Explicit shuffled logical identity assignment; optional installation file sorted by ID
contains only identity, side, offset and synthetic definition/source, no station, height,
world coordinates or ordering. Installation information is not consumed by an algorithm.

TA 20 Hz and AA 1 Hz are eligible-edge targets. Each unordered AA edge is one physical
exchange; only one designated endpoint reports it, avoiding double-counting reciprocal
reports. 250 exchanges/s cap includes attempts, with deterministic scheduling and dropped
budget requests logged. Range<=800 m and full-scene line-segment LOS are separate gates.
Endpoint tolerance .002 m, not a bypass for intervening walls. Reporters separately need
LOS/range to N0 at transmission and reception. Software reports at 5 Hz; this sparse scene uses one independent pair frame per single-shard envelope (protocol limit <=5 edges/shard). Different source frames are never combined into one envelope;
unchanged source frame/peer identity is deduplicated, not resampled. Bounded queue 256,
fixed .012 s transmission plus .001 s assembly, snapshot age<=1.2 s. This is a simplified
transport model, not RF/MAC simulation. Physical and delivery logs retain failed visibility,
deduplication, actual frequencies and coverage gaps.

## Isolation and acceptance

Configuration, mesh, facilities, station coordinates and 200 Hz truth belong only to
metadata/config and export/simulation. Optional installation metadata is separate.
Raw sensor MCAP contains no truth, axis, map, station coordinates or static transforms.
Pure replay/conversion must read only sensor bag and field contract. Existing LIUO and
snapshot-only FAST-LIO adapter are not certified for this scene. Future FAST-LIO target
uses native ring and float seconds `time` without geometry-based ring recovery; time-unit
configuration and algorithm support remain future integration work.

Before execution, scene.json fixes numerical tolerances: route position 1e-4 m,
mesh/noiseless reconstruction 5 mm, finite-difference acceleration .002 m/s²,
angular velocity 1e-5 rad/s, timestamps 1 µs. Bound checks tolerate 1e-5 SI floating error.
Independent quadrature, finite differences, analytic straight-section intersections,
portal rays and full-geometry segment cases validate implementation. Friction assumption
mu=.6 is experimental. Failures are retained; tolerances cannot be relaxed after failure.

All tests/generation/replay/analysis through data-run. One owner uses run_directory.py;
all output under /home/dev/ws_iap, reserve 8 GiB, finite 7200 s wall deadline. Final raw bag
and necessary truth are retained as future experiment inputs. Source/config/dependencies,
counts, digests and resource use are recorded, including actual sub-real-time speed.
