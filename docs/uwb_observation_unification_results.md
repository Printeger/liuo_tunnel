# UWB 接口统一软件验收结果

2026-10-27 UTC：T01–T14 在下述声明范围内全部通过。

**统一 UWB 接口已通过声明范围的软件集成、负载及代表性仿真融合/地图复用验收。**

本报告保留下方历史进度与失败；历史“pending / not accepted”只描述当时状态。
本次从实际干净 HEAD `3dbe34b` 继续，没有回退参考提交、创建分支/worktree、修改驱动或 push。

## 证据索引与版本

数据根 `D=/home/dev/ws_iap/liuo_observation`，运行根
`R=/home/dev/ws_iap/.runtime/0/liuo`。以下所有 run ID 均为 `R/<ID>`。
机器可审计总表：`D/evidence/final_acceptance.json`；初次汇总
`D/evidence/final_acceptance-1.json` 已 PASS。总表逐项绑定完整源码提交、干净状态、
配置 SHA256、消息接口摘要、实际二进制路径/摘要/ldd、各进程完整命令、审计文件摘要。
每个运行同时保留 `metadata/run_manifest.json`、`metadata/manifests/input_identity.json`、
`metadata/config/`、`runtime/`、`export/analysis/`；负向运行的 nominal FAIL 不改写。
下表配置摘要为前12位；完整值在上述总表与原始 identity 中。

| 证据角色 | run ID | 干净源码 | 配置 SHA256 前缀 |
|---|---|---|---|
| default_live | `20261027T035815Z_500` | `bd253fe` | `8d17f684edac` |
| default_replay | `20261027T040815Z_067` | `4c62fcd` | `e43c2cf10101` |
| contract_capture | `20261027T040326Z_895` | `183ae13` | `21a1dbbea5ff` |
| contract_replay | `20261027T040646Z_535` | `4c62fcd` | `abdff6f578aa` |
| boundary | `20261027T033259Z_396` | `08af3ec` | `d0c910fca3fe` |
| load | `20261027T033415Z_020` | `08af3ec` | `d6e407230a03` |
| fault_on | `20261027T034312Z_074` | `01807b5` | `b1380b99b6a6` |
| fault_off | `20261027T034428Z_811` | `01807b5` | `fca96e6ff1ad` |
| calibration | `20261027T031155Z_389` | `62fbd12` | `dd35b5e5fef1` |
| reuse | `20261027T031819Z_304` | `c653834` | `ab1453269c85` |
| legacy_reference | `20261027T034937Z_909` | `f1f16f5` | `d0c910fca3fe` |
| missing_model | `20261027T034841Z_670` | `2f15ba4` | `a607f6bab039` |
| invalid_model | `20261027T034853Z_096` | `2f15ba4` | `4ae65d2f7d53` |
| invalid_registry | `20261027T034855Z_623` | `2f15ba4` | `b53ded0614e1` |

只读接口版本：driver `a13dc6dc26ff7fed11272c2ec9a9f29020805b9a`；
`UwbObservation.msg` SHA256 `ddac200caf05c0ad087a4c884bb31d64c4418e159584c7576eea536878d6a188`；
`UwbObservationBatch.msg` SHA256 `d3830569846c6f67bd5f1726acf960137d06686a3d27cb9d68abf7e29aa9cc6d`。
未复制或修改共享消息。

实际新生产节点 SHA256 为 `e5b8286555e7a13df77417bf4a675ab03a4dc41573a4b81e879c73f35280b738`。
两次360秒代表性运行使用 `dd4878b0fcd206d905e4a05e789cef48130881f8d9c09b5216ab789ebe2a064e`。
汇总脚本逐字验证这之后生产节点唯一差异是“一倍 lag 的未来时间准入上限”，并逐条扫描
两次运行的时间：最大提前 .095/.05 秒，低于10秒上限，受影响条目为0。
其余生产路径未变；新 guard 已由 boundary、load、fault、默认链路实际运行验证。
不能将旧代表性二进制写成最终二进制；这里复用的是有明确不受影响证明的证据。

## T01–T14 最终矩阵

| 项目 | 结果 | 测试与证据摘要（角色见上表） |
|---|---|---|
| T01 | PASS | 数据盘消息 underlay/三包构建；实际默认 live/replay 与非默认 topic/depth37 的 fault_on，全链路 best effort/volatile；C++ input/UWB 与 Python58项通过 |
| T02 | PASS | contract_capture/replay：多 peer/shard、缺尾、交错，预登记每条身份与理由；load 每条边数量一致 |
| T03 | PASS | 显式 N1→内部0、未知节点、自环/reporter、混合合法项；invalid_registry 启动拒绝重复映射；地图内部0–3不变 |
| T04 | PASS | 173 batches/194条决策，空/超限/非法批、NaN/Inf/非正距离/shard/enum；非法条目不连带丢弃合法项；boundary 达实际队列上限 |
| T05 | PASS | 两种 mapped、零源时间的 unknown/receive-only、NaN uncertainty、非法时间、无效 freshness；源时间不被接收时间替换；负向原包回放语义 hash 相等 |
| T06 | PASS | 重复/跨轮/同值新帧/乱序/回绕/重启/退休迟到；source与dataset退休容量64；boundary 时钟回退清空并恢复，不重复入图 |
| T07 | PASS | missing_model 384条 MODEL_UNAVAILABLE，invalid_model 启动 INVALID_UWB_MODEL；有模型TA/AA融合；双向身份与单端大ID报告；未知独立性不增信息 |
| T08 | PASS | contract UWB OFF 图信息为0；fault_off AA summary/累计信息为0且TA继续；fault_on/calibration/reuse AA 不带车辆测距键，复用无重复安装因子 |
| T09 | PASS | default live→replay；contract_capture→contract_replay 4类传感器 hash/数量一致，NaN/false/unknown原样保存；旧包默认明确拒绝 |
| T10 | PASS | fault_on/off 延迟、重复、1秒断流、批内单anchor过滤、距离扰动；boundary 实际 seek/clock reset、未来拒绝、过期、queue满及恢复 |
| T11 | PASS | 公开ROS消息进入真实liuo_tunnel callback→审计/队列→消费→图；driver-like负向不补造质量；软件模型TA/AA实际融合 |
| T12 | PASS | load 完整60秒3570 batches/9570条全部生产接收融合；逐边六个10秒窗；独立慢订阅者真实丢失及恢复；boundary/contract另测容量 |
| T13 | PASS | calibration/reuse 360秒GPU联合图持续输出；物理CSV逐样本对应；legacy_reference 同一冻结传感器/地图下接受集合、关联、信息相等 |
| T14 | PASS | calibration 首次无坐标自标定4站/14维相关联合地图→导出→reuse 独立起点/朝向/seed持续定位；地图字节相等，无旧车辆状态先验 |

本轮必需项无未执行项。未执行的是范围外的真实无线、硬件同步、数小时稳定性及全场景部署认证。
预登记 fixture 在 `tools/uwb_{contract,load,boundary}_fixture.py` 和对应运行
`metadata/config/`；逐条生产证据为 `export/odometry/uwb_input_decisions.csv`，
消费及图证据由 `audit_uwb_{contract,load,boundary,graph}.py` 核对。
身份接受不是入图的替代；模型/时间/图侧理由分别保留。

## 60秒负载与边界

load 明确为软件合成：本地3边各50 Hz、各3000条；远端完整报告轮次5 Hz，
两边分别300/270条，后一边预声明每10轮缺尾，共30条未发送。
每个10秒窗口本地每边500、远端50/45。发送/实际生产接收均3570批、9570条唯一报告，
输入拒绝0，融合9000 TA/570 AA。它不代表增加了真实独立TWR频率。

回调总处理 p50/p95/max 为53.337/121.189/3217.753 μs；真实发布到回调墙钟
.279293/66.252169/280.980759 ms，包含初始化积压。队列峰324，去重状态峰628，
退休峰0。owner 从启动采71次：估计器 RSS 133276–301776 KiB，启动到末尾增长156684 KiB；
初始化后消费采样 RSS 增长28164 KiB，峰302464 KiB。短时增长不能证明长期无泄漏。
独立慢订阅者接收1096、缺2474批，恢复窗535/535；生产端丢失0是本次测量结果，
不是 best-effort 保证。它与生产端初始化积压分别统计。

boundary 单独预登记4100条：4096个近未来条目占满真实队列；超限明确拒绝，
远未来返回 FUTURE_TIME_LIMIT。实际时钟回退将4096条以 CLOCK_RESET 清理，
这些未来条目入图0；新会话随后持续246个状态/2082条融合。旧dataset迟到拒绝，
过期条目不入图。容量边界与60秒正常负载没有混为一次实验。

## 代表性GPU、物理等价与地图闭环

预检 data-run ABI7/数据盘及 RTX4070 Ti SUPER、driver580.126.09 通过。
以下均使用 GPU VGICP/IMU/UWB 原联合图，未用 CPU 或 record-only 代替。
先完成70秒最小有效首轮/复用，再使用已有360秒路线验证完整4站地图。

| 指标 | 首次 calibration | 独立 reuse |
|---|---:|---:|
| seed；起点 x,y,z,yaw | 0；200,0,1.2,0 | 1；220,.2,1.6,π/2 |
| 初始化耗时 / 首输出时刻(s) | 41.8 / 42.8 | 19.4 / 20.4 |
| TA / AA 接受并消费 | 22117 / 1080 | 22424 / 1080 |
| 状态 DEGRADED / LOST | 3178 / 4 | 3395 / 11 |
| x/y/z RMSE(m)，固定原坐标 | .015542 / .029610 / .741167 | .035755 / .040440 / .706823 |
| 处理p95(ms) / 总墙钟p95(s) | 35.0993 / .062355 | 33.8915 / .057607 |
| RSS峰(KiB) | 543604 | 349680 |

原始物理调度、量程、LOS、噪声与随机流未改变。首轮26614、复用26685样本与
历史冻结CSV逐条核对端点映射、物理时刻、样本身份、原始double距离和权重来源；
float32最大误差约3.05146e-5m，单独记录。不是仅凭 seed 一致宣称同输入。
早期阶段三场景包含迁移前已移除的LiDAR特征，因此它只能作物理测距对照：
旧首轮多接受37个TA，由anchor3确认时刻190.4→192.3导致37条过期；
该历史图比较报告保留差异，未用它宣称严格同输入等价。

为消除该限制，`legacy_reference` 运行保留的真实迁移前节点与core，
来源 `73d5f31a98ae2b639235b694efefff4956a5dce0`、历史身份
`log/20261023T061415Z_161`；节点SHA256
`afe6ae1ea0fb7be632e679d9fd60612155fc70521594ec842799ae0e473b962b`，core
`a75ad893e8013e2b617fe51c7237f1397fdfd521c77d69933bf5c67e419c41c1`。
它读取与reuse完全相同的冻结IMU/LiDAR/clock与地图；仅隔离参考进程使用旧接口表示，
以原始16有效数字CSV恢复double测距，保留完整双射。精度限于CSV，不宣称double逐bit还原。
该入口要求可审计原仿真单条样本，不接受driver未知质量；新生产节点始终直接订阅batch。
无旧包覆盖、无当前工作树回退。

严格同输入结果：26685条交付，两版本均23504条接受，关联差异0；
TA/AA信息分别2242400/108000 m⁻²；时刻表示误差≤1.777e-15s。
3406个共同轨迹状态的版本间分轴RMSE为
1.1344e-5 / 3.8421e-5 / 1.4699e-4m，最大差 .000649/.002237/.003000m；
9维协方差相对Frobenius差p95为1.15795e-5。
依据 `same_input_migration_audit.json`，并未把当前输出定义为自己的正确性基准。

首轮只用无坐标安装合同，不读 surveyed/oracle 或按ID/间距推位置；真值仅生成与评估。
导出地图内部ID `[0,1,2,3]`，14维 A/L 联合协方差正定（最小特征值1.3454e-6），
anchor/reference交叉协方差范数1.3099，SHA256
`ac3aeeb451f290575492f5e3cb0ac8402f708ddadc369dbf7e08bcfa3ba98376`。
第二轮加载完全相同字节，保留坐标规范、相关性与来源；每帧地图先验1，旧安装因子0。
没有带入上一轮车辆位姿/偏置，没有再次加入地图吸收的旧观测。
`deployment_validated=false` 保持原样：高度弱、局部协方差和间歇LOST/DEGRADED均未隐藏，
没有新增或放宽精度硬门槛。

## 可复现入口

从项目根运行，所有产物经 data-run；不将输出写入源码树。
README 顶部给出默认、非默认topic/depth、360秒首次/独立复用完整命令。
原次运行的精确进程命令及配置在总表 `runs.<role>.commands` 与原始input identity，
顺序套件原owner命令在 `D/evidence/remaining-5/suite.json`。

```bash
data-run --check
data-run -- tools/build.sh
data-run -- tools/test.sh
data-run -- python3 tools/summarize_uwb_acceptance.py --output /home/dev/ws_iap/liuo_observation/evidence/final_acceptance.json
# 重新执行公开ROS合同、时间/容量、60秒负载及AA开关故障场景；使用新的输出目录。
data-run -- python3 tools/run_uwb_acceptance_remaining.py --output /home/dev/ws_iap/liuo_observation/evidence/reproduction-1
# 原包审计，参数替换为上表完整run路径。
data-run -- python3 tools/audit_uwb_contract.py /home/dev/ws_iap/.runtime/0/liuo/20261027T040646Z_535
data-run -- python3 tools/audit_uwb_load.py /home/dev/ws_iap/.runtime/0/liuo/20261027T033415Z_020
data-run -- python3 tools/audit_uwb_boundary.py /home/dev/ws_iap/.runtime/0/liuo/20261027T033259Z_396
```

负向原包回放需要显式 `--replay-declared-contract`，仅允许已有PASS合同审计、计划hash与
消息计数匹配的完成录包，不允许叠加变换；不会静默替换NaN/unknown。
普通旧UwbRange包默认拒绝：`20261027T040858Z_125` 与
`D/evidence/legacy-bag-rejection.log`。隔离旧版本参考入口为 `--legacy-reference-run`，
只用于上述迁移对照，不是统一生产输入。
最终Python58项见 `D/evidence/final-python-tests.log`；构建与CTest证据沿用下方记录，
未来上限变更构建见 `D/evidence/future-bound-build.log`。组件通过不替代上表生产运行。

## 保留的失败与限制

- 慢订阅depth2首次恢复525/535失败：`20261027T030742Z_145`。改为预声明depth37后恢复535/535，仍报告真实丢失。
- boundary现场构造大批导致回放deadline：`20261027T032647Z_142`；预构造后静态重启片段初始化超时：`20261027T032909Z_359`。最终真实seek重放启动激励，未把跨重置样本说成新的独立物理测距。
- 0.2秒断流低于状态debounce未达场景断流判据：`20261027T033532Z_054`；改为1秒后首次启动clock deadline：`20261027T033909Z_948`；一次同配置重试通过。仍有启动调度敏感性，未弱化deadline。
- 负向原包回放工具错误地索引未知节点255：`20261027T040346Z_326`；修复为原样传递到生产校验后复验通过。
- load变换输入、boundary时间seek、contract非法输入的nominal评价FAIL保留；它们以预登记专用审计判定。模型/registry启动失败是期望拒绝，不能解释为正常融合通过。
- 全部历史失败和早期不充分smoke证据继续保留。60秒仅为声明负载验证；结果不涵盖真实无线、硬件同步、数小时稳定性或全场景部署。

---

## 历史工作记录（以下状态为当时快照）

### UWB interface unification — historical work in progress

Started 2026-10-26. **Not accepted; Goal remains active.** Historical phase-two
and phase-three results and failures are unchanged.

Baseline LIUO: clean master `1330b6e3fb1b6cc536a04a4833bd36355c4738f5`.
Read-only driver: `a13dc6dc26ff7fed11272c2ec9a9f29020805b9a`.
No new branch/worktree, no push. Data root for builds and test evidence:
`/home/dev/ws_iap/liuo_observation`; run root is supplied by data-run
(`/home/dev/ws_iap/.runtime/0/liuo` on this host).

## Evidence so far

All artifact-generating commands use `data-run --`. `data-run --check` passed
Landlock ABI 7 and data-mount verification. nvidia-smi found an RTX 4070-class GPU
with driver 580.126.09; unrelated graphics/compute load was present and untouched.
This is device preflight only, not GPU regression acceptance.

- `tools/build.sh`: isolated message underlay plus three LIUO packages built on
  the data disk. `evidence/m0-build.log`, `m1-build-2.log` passed. Initial M1 build
  failed on run-info string typing; `m1-build.log` retained.
- `test_observation_input`: 3 C++ tests passed, covering peer identity, wrap,
  retransmission, reorder retirement, unknown evidence, invalid fields, source
  restart and first-available AA reporter. `evidence/input-unit.{log,xml}`.
- `tools/check_pressure_message.py`: actual generated ROS CDR/hash round trip,
  NaN preservation, float32 distance-only stress passed in
  `evidence/pressure-message-2.log`. First fixture incorrectly compared a Python
  double with its float32 serialized value; failure retained in pressure-message.log.
- `python3 -m unittest discover -s tools -p 'test_*.py'`: 57 passed in
  `evidence/python-tests-3.log`. Earlier attempts found a source-tree temporary
  path, missing explicit message environment and stale delivery fixtures; retained.
- Targeted CTest initially passed the input tests and 20/21 existing UWB tests;
  the remaining test wrote a hardcoded source-tree build path. Failure is in
  `evidence/uwb-ctest.log`; temporary-path repair is being verified.
- First capture attempt was correctly refused before execution due to uncommitted
  work: `evidence/capture-smoke.log`, failed run `20261026T130138Z_616`.

## Acceptance matrix

T01 partial (build/type; live QoS pending). T02–T07 partial component evidence.
T08 graph integration pending. T09 actual capture/replay pending. T10 partial
serialization/scheduler evidence; ROS reset integration pending. T11 production
input integration pending. T12 60-second load pending. T13 representative GPU
simulation and physical baseline comparison pending. T14 independent map reuse
pending. No item is marked fully passed from component evidence alone.

Remaining work includes full topic/QoS propagation, bounded observer audit state,
complete raw identity and graph-count assertions, public ROS fixtures and load,
float32 physical-equivalence capture, representative self-calibration/export/reuse,
README final commands and final clean revision/evidence binding. No real wireless,
hardware clock or deployment claim is made.

## First committed live smoke evidence

Implementation slice `bd2572a`; clean-runtime fixes `46a6306`, `59de051`.
These are intermediate commits, not final acceptance revisions.

- Targeted CTest after temporary-path repair: input adapter, existing UWB suite,
  calibrated-map suite all passed (`evidence/uwb-ctest-2.log`).
- GPU preflight failures `20261026T130616Z_078` and `20261026T130706Z_057`
  reported cuInit=304. Direct data-run initialization succeeded. Diagnosis showed
  env isolation discarded the launcher's thread-name broker and preload, needed
  for CUDA's procfs thread naming under Landlock. Preserving that existing broker
  fixed preflight; storage protection was not changed. Cache/ROS directories are
  also preserved. The failed logs are capture-smoke-2.log and capture-smoke-3.log.
- Clean commit `59de051`, capture run `20261026T130803Z_515`: 6 s, 1200 IMU,
  1200 clocks, 60 clouds, 384 batches/observations/unique reports, zero violations,
  raw_input_valid=true. Actual MCAP retained under export/capture/raw. Command:
  `data-run -- tools/run.sh --duration 6 --capture-only --capture --wall-timeout 60`.
  This run did not execute localization.
- Replay run `20261026T130841Z_172` with default phase1 configuration failed on
  its pre-existing missing max_keyframes setting; error retained. The documented
  default 15 must be used consistently by the matching code.
- Replay run `20261026T130927Z_930` with phase2_continuous_walls_off.json consumed
  the same captured input through the production callback, produced 39 DEGRADED
  states, and verified all raw hashes/counts. Evaluation FAIL solely because the
  new input metadata write replaced the earlier GPU identity fields. Consolidating
  those fields fixes provenance; this failed run is not relabeled PASS.
  Command: `data-run -- tools/run.sh --replay
  /home/dev/ws_iap/.runtime/0/liuo/20261026T130803Z_515 --config
  packages/liuo_tunnel/config/phase2_continuous_walls_off.json --wall-timeout 60`.

Next: rebuild and repeat the short replay after metadata/default correction;
then public batch fixtures with malformed/mixed/driver-like inputs, 60-second
load with per-edge/window accounting and resource assertions, full topic/depth
propagation, and 360-second representative calibration plus independent reuse.
Physical equivalence must retain exact pre-float32 generated samples and mapped
identities; same seed alone is not accepted. Current adapter component tests are
not a substitute for graph-side assertions in those runs.

Metadata/default correction built successfully (`evidence/m1-build-3.log`).
Clean commit `f7edbfa`, repeated OFF replay `20261026T131233Z_175`:
**single-run contracts PASS**, no failures, frozen_input_verified=true and
GPU execution identity verified. Evidence `evidence/replay-smoke-3.log` and the
run's export/analysis/report.json. This remains six-second record-only smoke
coverage, not TA/AA fusion, load, or representative-route acceptance.

## Continued production acceptance (2026-10-27)

Resumed from clean `3dbe34b`; no existing task process was active. Data-run ABI 7
preflight passed; RTX 4070 Ti SUPER driver 580.126.09, 130 GiB available.
No reset, branch or worktree was created. The active Goal is still incomplete.

- `c792c12`: topic/depth propagation, source/target replay topic distinction and
  configurable fault scheduling. Build passed; 57 Python tests plus 8 targeted
  delivery tests passed (data root `liuo_observation/evidence/topic-*.log`).
  Capture `20261027T025815Z_012`, `/acceptance/uwb`, depth 37; replay
  `20261027T025831Z_074`, `/acceptance/replayed_uwb`, depth 37, visualization on.
  Raw hash/counts matched; GPU record-only run contract PASS. This is configuration
  coverage only, not fusion acceptance. Both runs reside under
  `/home/dev/ws_iap/.runtime/0/liuo/`.
- `d5ce11a`: production per-item input decisions with queue/identity/retirement
  sizes and timing; physical pre-float32 simulator samples; public ROS contract
  fixture on frozen IMU/LiDAR/clock. Build, 58 Python tests and targeted input/UWB
  CTests passed (`callback-*.log`). Run `20261027T030137Z_117` uses
  `/acceptance/contract`, depth 37 and the OFF configuration. All 41 batches and
  61 per-item decisions match the predeclared plan; 40 graph updates have zero
  TA/AA information. `export/analysis/uwb_contract_audit.json` PASS covers only
  this callback/OFF subset of T02–T08/T11. Nominal evaluator FAIL is intentionally
  retained for malformed fields, duplicates, changed UWB stream and rejections.
  The auditor initially hit Python CSV's 128 KiB field limit on the oversized
  batch; its parser now has an explicit 4 MiB bound, without changing production
  limits. This audit-only correction does not change the tested binary.

Commands (all from project root, all outputs on the data disk):

```bash
data-run -- tools/run.sh --duration 6 --capture-only --capture --uwb-topic /acceptance/uwb --uwb-qos-depth 37 --wall-timeout 60
data-run -- tools/run.sh --replay /home/dev/ws_iap/.runtime/0/liuo/20261027T025815Z_012 --config packages/liuo_tunnel/config/phase2_continuous_walls_off.json --uwb-topic /acceptance/replayed_uwb --uwb-qos-depth 37 --visualize --wall-timeout 60
data-run -- tools/run.sh --replay /home/dev/ws_iap/.runtime/0/liuo/20261027T025815Z_012 --config packages/liuo_tunnel/config/phase2_continuous_walls_off.json --uwb-topic /acceptance/contract --uwb-qos-depth 37 --uwb-contract-fixture --wall-timeout 60
data-run -- python3 tools/audit_uwb_contract.py /home/dev/ws_iap/.runtime/0/liuo/20261027T030137Z_117
```

Remaining: fused graph fixture, clock/reset/capacity boundaries, 60 s load and
slow subscribers, transport transforms live verification, representative GPU
self-calibration/export/reuse and historical physical equivalence.
