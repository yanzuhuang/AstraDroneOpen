#!/usr/bin/env python3
"""Produce Chinese 300 dpi evaluation figures only from a complete paired batch."""
import argparse
import csv
import json
from pathlib import Path
import statistics

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np

METHODS = ['fixed_0.60', 'fixed_1.00', 'fixed_1.40', 'sac']
LABELS = ['固定 0.60 m/s', '固定 1.00 m/s', '固定 1.40 m/s', 'SAC 确定性策略']


def summarize(root):
    with (root / 'episodes.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    manifest = json.loads((root / 'manifest.json').read_text())
    count = len(manifest['cases'])
    if len(rows) != 8 * count:
        raise ValueError('配对图表要求 {} 个真实闭合 Episode；当前 {}'.format(8 * count, len(rows)))
    result = []
    for environment in ['worksite', 'forest']:
        expected_seeds = {str(c['forest_seed'] if environment == 'forest' else c['reset_seed']) for c in manifest['cases']}
        for method in METHODS:
            group = [r for r in rows if r['environment'] == environment and r['method'] == method]
            if len(group) != count or {r['map_seed'] for r in group} != expected_seeds or {int(r['episode']) for r in group} != set(range(1,count+1)):
                raise ValueError('配对 seed/Episode 合同不完整')
            tracking = []
            speeds = []
            truncated = 0
            for row in group:
                folder = root / 'runs' / environment / method / '{:03d}'.format(int(row['episode']))
                classification = json.loads((folder / 'classification.json').read_text())
                truncated += int(classification['truncated'])
                runtime = json.loads((folder / 'sac_runtime_summary.json').read_text())
                if not (runtime.get('network_parameters_unchanged') and runtime.get('training_counters_unchanged')) or runtime.get('replay_buffer_created') or runtime.get('learner_created'):
                    raise ValueError('evaluation 网络/Replay 隔离失败')
                with (folder / 'sac_transition_audit.jsonl').open() as stream:
                    for line in stream:
                        item = json.loads(line)
                        speeds.append(float(item['actual_speed_mps']))
                        tracking.append(float(item['tracking_error_m']))
            success_times = [float(r['mission_time']) for r in group if int(r['success'])]
            result.append(dict(environment=environment, method=method, episodes=count,
                               success_rate=sum(int(r['success']) for r in group) / count,
                               collision_rate=sum(int(r['collision']) for r in group) / count,
                               planner_failure_rate=sum(int(r['planner_failure']) for r in group) / count,
                               tracking_failure_rate=sum(int(r['tracking_failure']) for r in group) / count,
                               truncated=truncated, success_count=len(success_times),
                               failure_count=count-len(success_times)-truncated,
                               invalid_observation_count=sum("invalid_observation" in r["terminal"] for r in group),
                               mean_task_time=statistics.mean(success_times) if success_times else None,
                               mean_actual_speed=statistics.mean(speeds), tracking_p95=float(np.percentile(tracking,95))))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    rows = summarize(args.output)
    available = {font.name for font in font_manager.fontManager.ttflist}
    font = next((f for f in ['Noto Sans CJK SC','Noto Sans CJK JP','Droid Sans Fallback','WenQuanYi Zen Hei','WenQuanYi Micro Hei','SimHei'] if f in available), None)
    if not font:
        raise RuntimeError('没有可用中文字体，拒绝输出缺字图片')
    plt.rcParams.update({'font.family':font,'axes.unicode_minus':False,'figure.facecolor':'white','axes.facecolor':'white'})
    for environment, name in [('worksite','工地场景'),('forest','随机森林')]:
        group = [r for r in rows if r['environment']==environment]
        for key, suffix, title, ylabel in [('success_rate','success_rate','任务成功率','成功率（%）'),('collision_rate','collision_rate','碰撞终止／代理标记率','碰撞终止／代理标记率（%）'),('mean_task_time','task_time','成功任务平均时间','仿真时间（s）')]:
            values = [r[key] for r in group]
            factor = 100 if key.endswith('rate') else 1
            fig, ax = plt.subplots(figsize=(9,5.4))
            fig.subplots_adjust(bottom=.24, top=.87, left=.11, right=.97)
            for i,v in enumerate(values):
                if v is None:
                    ax.text(i,0,'无成功样本',ha='center',va='bottom')
                else:
                    ax.bar(i,v*factor,color=['#667c95','#7891ae','#93aac2','#d47b38'][i],width=.6)
                    ax.annotate('{:.1f}'.format(v*factor),(i,v*factor),xytext=(0,4),textcoords='offset points',ha='center')
            ax.set_xlim(-0.6, 3.6)
            ax.set_xticks(range(4));ax.set_xticklabels(LABELS)
            ax.set_ylabel(ylabel);ax.set_title(name+'：'+title+'（每种方法 {} Episodes，小样本）'.format(group[0]['episodes']))
            if key.endswith('rate'):ax.set_ylim(0,110)
            else:
                ax.set_ylim(bottom=0)
                ax.margins(y=.18)
                for i,r in enumerate(group):
                    ax.annotate('成功 n={}'.format(r['success_count']), (i,0), xytext=(0,-30), textcoords='offset points', ha='center', fontsize=9)
            ax.spines['top'].set_visible(False);ax.spines['right'].set_visible(False)
            note = '观测异常计为失败；仅描述本次小样本，不代表统计显著优势。' if environment == 'worksite' else '碰撞为终止／代理口径，可与规划失败重叠；非 Gazebo 接触次数。'
            fig.text(.5, .025, note, fontsize=9, ha='center')
            fig.savefig(args.output/(environment+'_sac_vs_fixed_'+suffix+'.png'),dpi=300)
            plt.close(fig)
    (args.output/'comparison_summary.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2))
    lines=['|环境|方法|Episodes|成功率|碰撞率|Planner Failure率|平均任务时间（成功样本）|平均实际速度|P95跟踪误差|','|---|---|---:|---:|---:|---:|---:|---:|---:|']
    for r in rows:
        time_text='N/A' if r['mean_task_time'] is None else '{:.3f}'.format(r['mean_task_time'])
        lines.append('|{}|{}|{episodes}|{:.1%}|{:.1%}|{:.1%}|{} (n={})|{:.3f}|{:.3f}|'.format(r['environment'],r['method'],r['success_rate'],r['collision_rate'],r['planner_failure_rate'],time_text,r['success_count'],r['mean_actual_speed'],r['tracking_p95'], episodes=r['episodes']))
    (args.output/'comparison_table.md').write_text('\n'.join(lines)+'\n')


if __name__=='__main__':
    main()
