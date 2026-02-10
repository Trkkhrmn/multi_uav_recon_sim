#!/usr/bin/env python3

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# Data: gt_x, gt_y, det_x, det_y, scenario
raw = """
-37.5	52.7	-39	53.3	Scenario 1
-5.4	37.01	-5.3	34.2	Scenario 1
7.15	49.63	5.4	49.6	Scenario 1
55.85	30.28	55.5	30.5	Scenario 1
45	20	44.5	20.2	Scenario 1
60	5	59.7	5.2	Scenario 1
30.44	-7.08	30	-7.2	Scenario 1
56.18	-40.37	55.7	-40.5	Scenario 1
10	-35	9.2	-35	Scenario 1
11.9	-52.09	11	-52.2	Scenario 1
-20	-30	-20.7	-30.5	Scenario 1
-49.51	-4.56	-48.6	-4.7	Scenario 1
-50.77	25	-51.6	25	Scenario 1
-38.08	-6.3	-38.6	-6.4	Scenario 2
60.72	-8.8	61.6	-9.2	Scenario 2
64.44	44.06	64.7	42.9	Scenario 2
48.22	14.36	48	14.3	Scenario 2
-35.75	18.17	-36.7	17.5	Scenario 2
60.61	-29.62	60.9	-29.2	Scenario 2
-19.54	-6.35	-19.5	-6	Scenario 2
44.46	9.75	44.3	9.7	Scenario 2
31.3	-59.91	31.6	-59.4	Scenario 2
43.3	-38.21	42.2	-38.2	Scenario 2
-53.66	-33.14	-54.2	-32.4	Scenario 2
28.86	-56.22	27.8	-56.5	Scenario 2
13.9	-5.49	14.7	-5.3	Scenario 2
8.43	-48.61	7.6	-49.3	Scenario 2
2.59	-49.52	2.8	-49.9	Scenario 3
10.33	5.53	10.5	6.2	Scenario 3
-42.8	35.71	-44.3	36.1	Scenario 3
26.96	-39.79	27.6	-39.9	Scenario 3
39.47	33.51	38.6	33.5	Scenario 3
61.93	46.05	60.5	45.6	Scenario 3
38.76	-39.54	38.6	-40.1	Scenario 3
-44.87	-51.18	-44	-51.5	Scenario 3
-57.64	47.61	-58.5	47.6	Scenario 3
-7.83	34.14	-7.7	34	Scenario 3
-51.3	-23.58	-51.9	-23.7	Scenario 3
-35.82	-37.32	-36.3	-36.9	Scenario 3
-51.04	-52.33	-50.7	-52.4	Scenario 3
20.4	-24.87	21.4	-24.8	Scenario 3
21.45	38.88	21.7	38.9	Scenario 4
-32.71	-43.69	-33.3	-43.4	Scenario 4
13.88	-9.01	13.2	-9.1	Scenario 4
-21.09	30	-20.4	29.8	Scenario 4
-25.27	-26.67	-25.3	-27.2	Scenario 4
-35.48	-48.52	-34.5	-49.1	Scenario 4
41.14	-55.21	41.8	-55.3	Scenario 4
32.45	35.66	32.1	36	Scenario 4
4.27	46.55	2.8	46.6	Scenario 4
-31.91	-45.21	-33	-45	Scenario 4
14.28	32.14	14.1	32.1	Scenario 4
-38.55	-22.22	-39.4	-22.1	Scenario 4
23.13	48.24	23.3	48.7	Scenario 4
-14.63	-42.69	-14	-42.1	Scenario 4
"""

lines = [l.strip() for l in raw.strip().split('\n') if l.strip()]
data = []
for line in lines:
    parts = line.split('\t')
    gt_x, gt_y = float(parts[0]), float(parts[1])
    det_x, det_y = float(parts[2]), float(parts[3])
    scenario = parts[4].strip()
    data.append((gt_x, gt_y, det_x, det_y, scenario))

gt_x = np.array([d[0] for d in data])
gt_y = np.array([d[1] for d in data])
det_x = np.array([d[2] for d in data])
det_y = np.array([d[3] for d in data])
scenarios = np.array([d[4] for d in data])

# Errors
err_x = det_x - gt_x
err_y = det_y - gt_y
err_euclidean = np.sqrt(err_x**2 + err_y**2)

scenario_labels = sorted(np.unique(scenarios))
colors = plt.cm.tab10(np.linspace(0, 1, len(scenario_labels)))
scenario_to_color = {s: colors[i] for i, s in enumerate(scenario_labels)}

out_dir = 'output/figures'
import os
os.makedirs(out_dir, exist_ok=True)

# ----- 1. Ground truth vs Detected (X and Y) with identity line -----
fig, axes = plt.subplots(1, 2, figsize=(10, 5))
for sc in scenario_labels:
    mask = scenarios == sc
    axes[0].scatter(gt_x[mask], det_x[mask], label=sc, c=[scenario_to_color[sc]], alpha=0.8, edgecolors='k', linewidths=0.5)
    axes[1].scatter(gt_y[mask], det_y[mask], label=sc, c=[scenario_to_color[sc]], alpha=0.8, edgecolors='k', linewidths=0.5)
xlim = (min(gt_x.min(), det_x.min()) - 2, max(gt_x.max(), det_x.max()) + 2)
ylim = (min(gt_y.min(), det_y.min()) - 2, max(gt_y.max(), det_y.max()) + 2)
axes[0].plot(xlim, xlim, 'k--', alpha=0.6, label='y=x')
axes[0].set_xlabel('Ground truth X (m)')
axes[0].set_ylabel('Detected X (m)')
axes[0].set_title('X coordinate: GT vs Detected')
axes[0].legend()
axes[0].set_aspect('equal', adjustable='box')
axes[0].grid(True, alpha=0.3)
axes[1].plot(ylim, ylim, 'k--', alpha=0.6, label='y=x')
axes[1].set_xlabel('Ground truth Y (m)')
axes[1].set_ylabel('Detected Y (m)')
axes[1].set_title('Y coordinate: GT vs Detected')
axes[1].legend()
axes[1].set_aspect('equal', adjustable='box')
axes[1].grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(f'{out_dir}/01_gt_vs_detected_xy.png', dpi=150, bbox_inches='tight')
plt.close()

# ----- 2. 2D map: GT points and arrows to detected -----
fig, ax = plt.subplots(figsize=(8, 8))
for sc in scenario_labels:
    mask = scenarios == sc
    ax.quiver(gt_x[mask], gt_y[mask], err_x[mask], err_y[mask],
              color=scenario_to_color[sc], alpha=0.7, scale=1, scale_units='xy', angles='xy', width=0.003, label=sc)
ax.set_xlabel('Ground truth X (m)')
ax.set_ylabel('Ground truth Y (m)')
ax.set_title('Position error vectors (GT → Detected)')
ax.legend()
ax.set_aspect('equal')
ax.grid(True, alpha=0.3)
ax.axhline(0, color='k', linewidth=0.5)
ax.axvline(0, color='k', linewidth=0.5)
plt.tight_layout()
plt.savefig(f'{out_dir}/02_error_vectors_2d.png', dpi=150, bbox_inches='tight')
plt.close()

# ----- 3. Euclidean error distribution (histogram) -----
fig, ax = plt.subplots(figsize=(6, 4))
ax.hist(err_euclidean, bins=15, color='steelblue', edgecolor='black', alpha=0.7)
ax.axvline(np.mean(err_euclidean), color='red', linestyle='--', linewidth=2, label=f'Mean = {np.mean(err_euclidean):.3f} m')
ax.axvline(np.median(err_euclidean), color='orange', linestyle=':', linewidth=2, label=f'Median = {np.median(err_euclidean):.3f} m')
ax.set_xlabel('Euclidean error (m)')
ax.set_ylabel('Count')
ax.set_title('Distribution of localization error')
ax.legend()
ax.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(f'{out_dir}/03_error_histogram.png', dpi=150, bbox_inches='tight')
plt.close()

# ----- 4. Boxplot: error by scenario -----
fig, ax = plt.subplots(figsize=(6, 4))
box_data = [err_euclidean[scenarios == sc] for sc in scenario_labels]
bp = ax.boxplot(box_data, tick_labels=scenario_labels, patch_artist=True)
for i, patch in enumerate(bp['boxes']):
    patch.set_facecolor(colors[i])
ax.set_ylabel('Euclidean error (m)')
ax.set_xlabel('Scenario')
ax.set_title('Localization error by scenario')
ax.grid(True, alpha=0.3, axis='y')
plt.tight_layout()
plt.savefig(f'{out_dir}/04_error_by_scenario_boxplot.png', dpi=150, bbox_inches='tight')
plt.close()

# ----- 5. Bar chart: RMSE and Mean error per scenario -----
rmse_per = [np.sqrt(np.mean(err_euclidean[scenarios == sc]**2)) for sc in scenario_labels]
mean_per = [np.mean(err_euclidean[scenarios == sc]) for sc in scenario_labels]
x = np.arange(len(scenario_labels))
width = 0.35
fig, ax = plt.subplots(figsize=(6, 4))
ax.bar(x - width/2, mean_per, width, label='Mean error (m)', color='steelblue', alpha=0.8)
ax.bar(x + width/2, rmse_per, width, label='RMSE (m)', color='coral', alpha=0.8)
ax.set_xticks(x)
ax.set_xticklabels(scenario_labels)
ax.set_ylabel('Error (m)')
ax.set_xlabel('Scenario')
ax.set_title('Mean error and RMSE per scenario')
ax.legend()
ax.grid(True, alpha=0.3, axis='y')
plt.tight_layout()
plt.savefig(f'{out_dir}/05_rmse_mean_per_scenario.png', dpi=150, bbox_inches='tight')
plt.close()

# ----- 6. X and Y error by scenario (boxplot) -----
fig, axes = plt.subplots(1, 2, figsize=(10, 4))
for ax, err, title in zip(axes, [err_x, err_y], ['X error (m)', 'Y error (m)']):
    box_data = [err[scenarios == sc] for sc in scenario_labels]
    bp = ax.boxplot(box_data, tick_labels=scenario_labels, patch_artist=True)
    for i, patch in enumerate(bp['boxes']):
        patch.set_facecolor(colors[i])
    ax.axhline(0, color='gray', linestyle='-', linewidth=0.8)
    ax.set_ylabel(title)
    ax.set_xlabel('Scenario')
    ax.set_title(title)
    ax.grid(True, alpha=0.3, axis='y')
plt.suptitle('X and Y component errors by scenario', y=1.02)
plt.tight_layout()
plt.savefig(f'{out_dir}/06_xy_error_by_scenario.png', dpi=150, bbox_inches='tight')
plt.close()

# Summary stats
print('--- Summary ---')
print(f'Total points: {len(data)}')
print(f'Overall: Mean error = {np.mean(err_euclidean):.3f} m, RMSE = {np.sqrt(np.mean(err_euclidean**2)):.3f} m, Max = {np.max(err_euclidean):.3f} m')
for sc in scenario_labels:
    e = err_euclidean[scenarios == sc]
    print(f'  {sc}: n={len(e)}, Mean={np.mean(e):.3f} m, RMSE={np.sqrt(np.mean(e**2)):.3f} m')
print(f'Figures saved to {out_dir}/')
