#!/usr/bin/env python3
"""Offline-only APE evaluation. Truth is never published or passed to estimators."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[2]
NAMES = {
    'fastlio': ('FAST-LIO2 · snapshot 适配', 'FAST-LIO2', '#0072B2'),
    'liosam': ('LIO-SAM · 六轴适配', 'LIO-SAM (6-axis)', '#E69F00'),
    'self': ('本方法 · 无图/UWB 自标定', 'Ours: self-calibration', '#009E73'),
    'reuse': ('本方法 · 有图/UWB 复用', 'Ours: map reuse', '#CC79A7'),
    'off': ('本方法 · LiDAR+IMU / UWB OFF', 'Ours: UWB OFF', '#D55E00'),
}


def load(path):
    rows = list(csv.DictReader(path.open()))
    if not rows:
        return np.zeros((0, 8))
    values = np.array([[float(row[k]) for k in ('stamp','x','y','z','qx','qy','qz','qw')] for row in rows])
    if not np.isfinite(values).all():
        raise RuntimeError('nonfinite trajectory: ' + str(path))
    _, indices = np.unique(values[:, 0], return_index=True)
    return values[np.sort(indices)]


def rigid_fit(estimated, truth):
    e, t = estimated.mean(axis=0), truth.mean(axis=0)
    u, singular, vt = np.linalg.svd((estimated-e).T @ (truth-t))
    sign = np.linalg.det(vt.T @ u.T)
    rotation = vt.T @ np.diag([1., 1., sign]) @ u.T
    translation = t - rotation @ e
    return rotation, translation, singular


def associate(values, grid, tolerance=.051):
    if not len(values):
        return np.zeros(len(grid), dtype=int), np.zeros(len(grid), dtype=bool)
    right = np.clip(np.searchsorted(values[:,0], grid), 0, len(values)-1)
    left = np.maximum(0, right-1)
    indices = np.where(np.abs(values[left,0]-grid) < np.abs(values[right,0]-grid), left, right)
    return indices, np.abs(values[indices,0]-grid) <= tolerance


def stats(error):
    norm = np.linalg.norm(error, axis=1)
    return dict(rmse_m=float(np.sqrt(np.mean(norm**2))), mean_m=float(norm.mean()), median_m=float(np.median(norm)), std_m=float(norm.std()), p95_m=float(np.percentile(norm,95)), max_m=float(norm.max()), axis_rmse_m=np.sqrt(np.mean(error**2,axis=0)).tolist())


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--runs',type=Path,default=ROOT/'tools/baselines/runs.json')
    args=parser.parse_args()
    spec=json.loads(args.runs.read_text())
    source=ROOT/spec['source']
    truth=load(source/'export/simulation/truth.csv')
    truth_rot=Slerp(truth[:,0],Rotation.from_quat(truth[:,4:8]))
    def truth_at(times):
        return np.column_stack([np.interp(times,truth[:,0],truth[:,i]) for i in (1,2,3)])
    grid=np.round(np.arange(61.,360.9001,.1),4)
    trajectories, matched, valid, results={}, {}, {}, {}
    for key, relative in spec['runs'].items():
        run=ROOT/relative
        path=run/'export/odometry/trajectory.csv'
        values=load(path) if path.exists() else np.zeros((0,8))
        trajectories[key]=values
        matched[key],valid[key]=associate(values,grid)
        manifest=json.loads((run/'metadata/run_manifest.json').read_text())
        result=dict(run=relative,lifecycle=manifest['lifecycle'],source=manifest.get('source'),trajectory_sha256=hashlib.file_digest(path.open('rb'),'sha256').hexdigest() if path.exists() else None,poses=len(values),coverage_percent=float(100*valid[key].mean()),window_matches=int(valid[key].sum()))
        identity_file=run/'metadata/manifests'/('baseline_input.json' if key in ('fastlio','liosam') else 'input_identity.json')
        if identity_file.exists():
            identity=json.loads(identity_file.read_text())
            result['input_identity']=identity
            if key in ('self','reuse') and Path(identity.get('frozen_input',{}).get('source_run','')).resolve()!=source.resolve():
                raise RuntimeError('our replay source differs from declared benchmark input: '+key)
        if len(values):
            result.update(first_stamp=float(values[0,0]),last_stamp=float(values[-1,0]),max_output_gap_s=float(np.diff(values[:,0]).max()) if len(values)>1 else None)
        failure=run/'metadata/manifests/input_failure.json'
        if failure.exists():
            result['failure']=json.loads(failure.read_text())
        results[key]=result
    active=[k for k in NAMES if k in results and valid[k].sum() >= 10]
    if len(active)<2:
        raise RuntimeError('fewer than two evaluable trajectories; cannot compare APE')
    common=np.logical_and.reduce([valid[k] for k in active])
    if common.sum()<10:
        raise RuntimeError('insufficient common timestamps; report failures instead of a misleading comparison')
    common_time=grid[common]
    aligned, errors, fixed_errors={}, {}, {}
    for key in active:
        values=trajectories[key]
        chosen=values[matched[key][common]]
        target=truth_at(chosen[:,0])
        r,t,s=rigid_fit(chosen[:,1:4],target)
        aligned[key]=(r@chosen[:,1:4].T).T+t
        errors[key]=aligned[key]-target
        first=values[0]
        r0=(truth_rot(float(first[0])) * Rotation.from_quat(first[4:8]).inv()).as_matrix()
        t0=truth_at([first[0]])[0]-r0@first[1:4]
        fixed_errors[key]=(r0@chosen[:,1:4].T).T+t0-target
        att=(truth_rot(chosen[:,0]).inv() * Rotation.from_matrix(r) * Rotation.from_quat(chosen[:,4:8])).magnitude()
        all_chosen=values[matched[key][valid[key]]]
        all_target=truth_at(all_chosen[:,0])
        results[key].update(ape_se3=stats(errors[key]),ape_first_pose=stats(fixed_errors[key]),se3_rotation=r.tolist(),se3_translation=t.tolist(),fit_singular_values=s.tolist(),attitude_rmse_deg=float(np.rad2deg(np.sqrt(np.mean(att**2)))),common_samples=len(chosen),own_window_ape_using_common_fit=stats((r@all_chosen[:,1:4].T).T+t-all_target))
    external=[]
    for key in ('fastlio','liosam'):
        path=ROOT/spec['runs'][key]/'metadata/manifests/baseline_result.json'
        external.append(json.loads(path.read_text()))
    input_equal=external[0]['adapted_sha256']==external[1]['adapted_sha256'] and external[0]['counts']==external[1]['counts']
    if not input_equal:
        raise RuntimeError('external baseline full adapted inputs do not match; no paired precision claim')
    summary=dict(protocol='same source; body origin; no scale; nearest timestamps <=51ms; common ROS stamps [61,360.9]',source=spec['source'],source_capture_sha256={p.name:hashlib.file_digest(p.open('rb'),'sha256').hexdigest() for p in (source/'export/capture/raw').glob('*.mcap')},requested_grid_samples=len(grid),common_samples=int(common.sum()),common_coverage_percent=float(100*common.mean()),external_adapted_inputs_equal=input_equal,external_input_checks=external,results=results)
    out=ROOT/'docs';figdir=out/'figures';figdir.mkdir(exist_ok=True)
    (out/'ape_five_method_results.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False)+'\n')
    plt.rcParams.update({'font.size':10,'axes.grid':True,'grid.alpha':.25,'figure.dpi':150})
    fig,axes=plt.subplots(1,2,figsize=(13,5))
    target=truth_at(common_time)
    for ax in axes:ax.plot(target[:,0],target[:,1],'k--',lw=2,label='Truth (offline only)');ax.set_xlabel('Tunnel longitudinal X [m]');ax.set_ylabel('Lateral Y [m]')
    for key in active:
        for j,ax in enumerate(axes):
            if j==1 and key=='off':continue
            ax.plot(aligned[key][:,0],aligned[key][:,1],color=NAMES[key][2],lw=1,label=NAMES[key][1])
    axes[0].set_title('All methods: common-time SE(3)-aligned XY')
    axes[1].set_title('Detail: UWB OFF omitted here; shown at left')
    for ax in axes:ax.legend(fontsize=7)
    fig.tight_layout();fig.savefig(figdir/'ape_five_trajectories.png');plt.close(fig)
    fig,axes=plt.subplots(2,2,figsize=(13,8),sharex=True)
    for key in active:
        err=errors[key]
        axes[0,0].semilogy(common_time,np.maximum(np.linalg.norm(err,axis=1),1e-6),color=NAMES[key][2],lw=1,label=NAMES[key][1])
        for ax,i in zip([axes[0,1],axes[1,0],axes[1,1]],range(3)):
            ax.plot(common_time,err[:,i],color=NAMES[key][2],lw=.8)
            ax.set_yscale('symlog',linthresh=.1)
    for ax,title in zip(axes.flat,['Translation APE norm (log scale)','Longitudinal X error','Lateral Y error','Height Z error']):
        ax.set_title(title);ax.set_ylabel('Error [m], log / signed-log');ax.set_xlabel('ROS measurement time [s]')
    axes[0,0].legend(fontsize=7)
    fig.tight_layout();fig.savefig(figdir/'ape_five_errors.png');plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(13,5))
    x=np.arange(len(active))
    axes[0].bar(x-.18,[results[k]['ape_se3']['rmse_m'] for k in active],.36,label='Whole-window SE(3) alignment')
    axes[0].bar(x+.18,[results[k]['ape_first_pose']['rmse_m'] for k in active],.36,label='First-pose alignment only')
    axes[0].set_yscale('log');axes[0].set_ylabel('APE RMSE [m], log scale');axes[0].set_xticks(x,[NAMES[k][1] for k in active],rotation=20,ha='right');axes[0].legend(fontsize=8)
    for k in active:
        err=np.sort(np.linalg.norm(errors[k],axis=1));axes[1].semilogx(np.maximum(err,1e-6),np.arange(1,len(err)+1)/len(err),label=NAMES[k][1],color=NAMES[k][2])
    axes[1].set_xlabel('SE(3)-aligned APE [m], log scale');axes[1].set_ylabel('Cumulative fraction');axes[1].legend(fontsize=8)
    fig.tight_layout();fig.savefig(figdir/'ape_five_summary.png');plt.close(fig)
    def fmt(value):return f'{value:.3f}'
    rows=[];coverage=[];axis=[];sources=[]
    for k in NAMES:
        v=results[k];name=NAMES[k][0]
        if 'ape_se3' in v:
            a=v['ape_se3'];rows.append('| '+name+' | '+' | '.join(fmt(a[f]) for f in ('rmse_m','mean_m','median_m','p95_m','max_m'))+' | '+fmt(v['ape_first_pose']['rmse_m'])+' |')
            axis.append('| '+name+' | '+' | '.join(fmt(x) for x in a['axis_rmse_m'])+' | '+fmt(v['attitude_rmse_deg'])+' |')
        else:
            rows.append('| '+name+' | N/A | N/A | N/A | N/A | N/A | N/A |')
        coverage.append(f"| {name} | {v['poses']} | {v.get('first_stamp','—')} / {v.get('last_stamp','—')} | {v['coverage_percent']:.1f}% | {v.get('max_output_gap_s','—')} | {v['lifecycle']} |")
        sources.append(f"| {name} | `{v['run']}` | `{v['trajectory_sha256']}` |")
    best=min(active,key=lambda k:results[k]['ape_se3']['rmse_m'])
    report=f'''# 轨道隧道仿真：五种配置的 APE 实测对比

本报告比较 FAST-LIO2、LIO-SAM、本方法无图联合自标定、本方法有图复用和本方法关闭 UWB。这是**单个固定种子、snapshot 仿真、指定适配配置**的结果，不是算法普遍排名或部署认证。外部算法不使用 UWB；本方法无图/有图两组使用原始 UWB 紧耦合，OFF 组只用 LiDAR+IMU。

在以下共同样本和 SE(3) 对齐口径下，最低平移 APE RMSE 为 **{NAMES[best][0]}：{results[best]['ape_se3']['rmse_m']:.3f} m**。请同时看固定首姿态对齐、分轴误差及输出覆盖，不能只看单个最优数字。

## 1. 输入与公平性边界

- 评测原始输入：`{spec['source']}`，seed 1，360 s，真实运行起点 (220,0.2,1.6) m / yaw π/2，仅供仿真与离线评估说明，不进入估计器。40 线 × 512 水平射线、40 m LiDAR 量程、10 Hz 点云、200 Hz 六轴 IMU；点云为 snapshot，无滚动扫描验证。
- 有图组地图来自独立 seed 0 首次标定 `log/20261022T160635Z_994`，没有用 seed 1 的评测输入先建地图。这里的“有图”是 UWB 基站/安装参考的相关后验，**不是稠密点云地图重定位**。
- 外部两组适配后 LiDAR/IMU 消息数量与 SHA-256 严格相等：**PASS**。未发布真值、UWB 或 GPS。原始输入文件 SHA-256、全部结果与对齐矩阵见 [机器可读结果](ape_five_method_results.json)。
- FAST-LIO2 保留 CPU 原生后端；本方法使用 GPU 原始因子路径；LIO-SAM 使用 CPU、关闭 GPS/回环。本文不以不同后端的耗时作速度排名。
- LIO-SAM 原接口要求姿态，本次明确是**六轴适配版**：静止段比力初始化 roll/pitch、局部 yaw=0、随后陀螺积分，原始 gyro/acc 不改；不使用真值姿态。关闭独立航向初始化及持续 RPY 融合权重。不能把该结果当作带九轴姿态传感器的原版成绩。
- FAST-LIO2 对全零点时间默认会重建扫描时间，本次显式 snapshot 补丁关闭这种行为及等时点云去畸变；绝不伪造滚动扫描。ring 来自已知雷达射线角度，与隧道几何先验无关。详见 [冻结协议、补丁和来源](../tools/baselines/README.md)。

## 2. APE 定义

所有输出先转换到同一物理 **IMU/body 原点**。预声明主窗口为 ROS stamp 61–360.9 s，避开启动阶段但启动耗时仍单列。按 0.1 s 网格找最近输出，最大时间差 0.051 s；不跨断档插值，不删除大误差点。五组中有可评估轨迹的组采用共同有效时刻，共 **{int(common.sum())}/{len(grid)} 个网格点（{100*common.mean():.1f}%）**。真值在各自测量时间插值；只用于此离线步骤。

主结果采用每组一次全窗口 SE(3) 刚体最小二乘对齐，**不拟合尺度**：`APE_i = ||R p_est,i + t - p_gt,i||`，`RMSE = sqrt(mean(APE_i²))`。该对齐可以消除坐标规范差异，也会吸收部分整体姿态/位置漂移，因此同时报告仅用首个输出姿态建立坐标关联的 APE。分轴指仿真世界的隧道轴向 X、横向 Y、高度 Z。姿态 RMSE 是同一 SE(3) 对齐后的 SO(3) 角误差，不是位置 APE。

## 3. APE 精度表

单位：m；所有主表数字基于相同的共同时间集合。

| 配置 | APE RMSE | 均值 | 中位数 | P95 | 最大值 | 仅首姿态对齐 RMSE |
|---|---:|---:|---:|---:|---:|---:|
{chr(10).join(rows)}

![APE RMSE 与误差累计分布](figures/ape_five_summary.png)

全程 SE(3) 对齐后的低误差不等于已知世界坐标下的绝对定位精度；尤其不能据此宣称 UWB 地图高度已可靠。固定首姿态对齐也不是给估计器增加先验，它仅是离线选择坐标规范。

| 配置 | 轴向 X RMSE (m) | 横向 Y RMSE (m) | 高度 Z RMSE (m) | 姿态 RMSE (deg) |
|---|---:|---:|---:|---:|
{chr(10).join(axis)}

## 4. 轨迹与误差随时间变化

![共同时间的平面轨迹](figures/ape_five_trajectories.png)

左图保留全部方法的尺度；右图单独省略 OFF 以查看其余轨迹细节，未从指标中移除 OFF。坐标为离线对齐后的世界坐标。

![APE 及三个方向误差](figures/ape_five_errors.png)

APE 范数采用对数纵轴，以同时保留小误差和大漂移；分轴曲线采用保留正负号的对称对数轴（±0.1 m 内为线性）。长直、重复纹理的隧道存在几何弱约束，单独的“比 OFF 好”并不能证明部署有效性。

## 5. 输出完整性与失败记录

| 配置 | 总输出数 | 首/末输出 stamp (s) | 主窗口自身覆盖率 | 最大相邻输出间隔 (s) | run 生命周期 |
|---|---:|---|---:|---:|---|
{chr(10).join(coverage)}

`completed` 只代表输入回放和进程收尾完成，不表示定位健康或精度合格。N/A 表示缺少足够轨迹，不能按零误差处理。各组自身窗口指标与共同采样指标均在 JSON 中保留；稀疏输出会缩小共同集合，必须结合覆盖率阅读排名。

## 6. 可追溯运行与复现

| 配置 | 实际 run | 轨迹 SHA-256 |
|---|---|---|
{chr(10).join(sources)}

运行命令、固定源码提交、配置和全部适配说明见 [对比工具说明](../tools/baselines/README.md)。运行目录中的 `metadata/manifests/` 保存输入/二进制身份，`runtime/` 保存控制台日志，`export/odometry/trajectory.csv` 为原始估计轨迹；所有失败尝试保留，未替换成成功结果。

```bash
python3 tools/baselines/evaluate_ape.py --runs tools/baselines/runs.json
```

## 7. 能说明什么、不能说明什么

本报告仅说明这份冻结仿真输入、这些参数与适配条件下的实测差异。无图自标定和地图复用使用额外的 UWB 信息，因此不是三种纯 LIO 算法之间的同信息比较；纯 LIO 应只比较 FAST-LIO2、LIO-SAM 六轴适配版和本方法 OFF。原有本方法 DEGRADED/LOST、地图 provisional、协方差过度自信及高度限制不会被 APE 对齐或本报告覆盖。

未覆盖多种子统计、真实 Mid-360、rolling/去畸变、磁航向 IMU、完整飞行运动、断流故障矩阵或部署认证。没有按这份真值反复搜索参数；两套外部算法的短程 smoke 仅用于接口与生命周期检查，不作为主精度表数据。
'''
    (out/'ape_five_method_comparison.md').write_text(report)
    print(json.dumps({k:v.get('ape_se3') for k,v in results.items()},ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()
