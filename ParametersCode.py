"""
PLOTTING PARAMETERS CODE 

Returns: 
Empirical Stabilisation Time
Rise Time
Overshoot
SSE

Regression parameters provided with standard error

System parameters provided with 1sig standard deviation 

"""


import pandas as pd
from pathlib import Path
import re
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import linregress

plt.rcParams.update({
    'font.size': 14,
    'axes.titlesize': 16,
    'axes.labelsize': 14,
    'xtick.labelsize': 12,
    'ytick.labelsize': 12,
    'legend.fontsize': 12,
})

def parse_simulation_files(directory_path):
    compiled_data = []
    file_pattern = r"Xinit([-.\d]+)_Yinit([-.\d]+)_PX(\d+)_IX(\d+)_DX(\d+)_PY(\d+)_IY(\d+)_DY(\d+)"
    
    for file_path in Path(directory_path).glob("*.csv"):
        match = re.search(file_pattern, file_path.name)
        if match:
            df = pd.read_csv(file_path)
            compiled_data.append({
                "filename": file_path.name,
                "Xinit": float(match.group(1)),
                "Yinit": float(match.group(2)),
                "data": df
            })
    return compiled_data

def detect_stabilisation_time(t_ms, angle, window_ms=400.0, threshold_deg=0.5, min_windows=5):
    dt = np.median(np.diff(t_ms))
    if dt <= 0:
        return np.nan
    
    win_n = max(2, int(round(window_ms / dt)))
    streak = 0
    streak_start = None
    
    for i in range(len(angle) - win_n + 1):
        if np.std(angle[i:i + win_n]) < threshold_deg:
            if streak == 0:
                streak_start = i
            streak += 1
            if streak >= min_windows:
                return float(t_ms[streak_start])
        else:
            streak = 0
            streak_start = None
            
    return np.nan

def get_std(x):
    return x.std(ddof=1) if len(x) > 1 else 0.0

def calculate_metrics(t, angle_vals):
    y0 = angle_vals[0]
    sse = np.mean(angle_vals[-20:])
    
    if y0 > 0:
        cross_90 = t[angle_vals <= 0.9 * y0]
        cross_10 = t[angle_vals <= 0.1 * y0]
        overshoot_val = min(0, np.min(angle_vals) - sse)
    else:
        cross_90 = t[angle_vals >= 0.9 * y0]
        cross_10 = t[angle_vals >= 0.1 * y0]
        overshoot_val = max(0, np.max(angle_vals) - sse)

    t90 = cross_90[0] if len(cross_90) > 0 else np.nan
    t10 = cross_10[0] if len(cross_10) > 0 else np.nan
    rise_time = t10 - t90
    
    overshoot_pct = abs(overshoot_val / y0) * 100 if y0 != 0 else 0
    stab_time = detect_stabilisation_time(t, angle_vals)
    
    return stab_time, rise_time, sse, overshoot_pct

def plot_symmetry_panel(gx, gy, metric_x, std_x, metric_y, std_y, title, ylabel, fit_mode=None):
    fig, axs = plt.subplots(1, 2, figsize=(15, 6))
    fig.suptitle(title, fontsize=16)
    stats_out = {}

    for ax, data, axis_name, metric, std_col in zip(axs, [gx, gy], ['X', 'Y'], [metric_x, metric_y], [std_x, std_y]):
        ax_lower = axis_name.lower()
        pos = data[data[f'Is_Positive_{axis_name}'] == True].dropna(subset=[f'Mean_Abs_A{ax_lower}', metric])
        neg = data[data[f'Is_Positive_{axis_name}'] == False].dropna(subset=[f'Mean_Abs_A{ax_lower}', metric])

        ax.errorbar(pos[f'Mean_Abs_A{ax_lower}'], pos[metric], xerr=pos[f'Std_Abs_A{ax_lower}'], yerr=pos[std_col], fmt='o', color='crimson', ecolor='black', capsize=4, markeredgecolor='black', alpha=0.8, label='Positive Initial Angle')
        ax.errorbar(neg[f'Mean_Abs_A{ax_lower}'], neg[metric], xerr=neg[f'Std_Abs_A{ax_lower}'], yerr=neg[std_col], fmt='o', color='royalblue', ecolor='black', capsize=4, markeredgecolor='black', alpha=0.8, label='Negative Initial Angle')
        
        if fit_mode == 'linear':
            for df, color, label in [(pos, 'crimson', 'Pos'), (neg, 'royalblue', 'Neg')]:
                if len(df) > 2:
                    res = linregress(df[f'Mean_Abs_A{ax_lower}'], df[metric])
                    x_vals = np.array([df[f'Mean_Abs_A{ax_lower}'].min(), df[f'Mean_Abs_A{ax_lower}'].max()])
                    ax.plot(x_vals, res.slope * x_vals + res.intercept, color=color, linestyle='--')
                    stats_out[f'{axis_name}_{label}'] = res
        elif fit_mode == 'asymptote':
            for df, color, label in [(pos, 'crimson', 'Pos'), (neg, 'royalblue', 'Neg')]:
                high_df = df[df[f'Mean_Abs_A{ax_lower}'] > 40.0]
                if len(high_df) > 0:
                    asymp_mean = high_df[metric].mean()
                    asymp_err = high_df[metric].std(ddof=1) if len(high_df) > 1 else 0.0
                    ax.plot([40, 75], [asymp_mean, asymp_mean], color=color, linestyle=':')
                    stats_out[f'{axis_name}_{label}_Asymp'] = (asymp_mean, asymp_err)
        elif fit_mode == 'mean':
            overall_mean = data[metric].mean()
            overall_std = data[metric].std(ddof=1)
            ax.axhline(overall_mean, color='black', linestyle='-.', alpha=0.6, label='Global Mean')
            stats_out[f'{axis_name}_Overall'] = (overall_mean, overall_std)

        ax.set_title(f'{axis_name}-Axis (A{ax_lower})')
        ax.set_xlabel(f'Absolute Initial Angle $|Start\_A{ax_lower}|$ (Degrees)')
        ax.set_ylabel(ylabel)
        ax.legend()
        ax.grid(True)

    plt.tight_layout()
    plt.show()
    return stats_out

if __name__ == "__main__":
    directory_to_scan = "gimbal_data_repeats/gimbal_data_repeats"
    all_files_data = parse_simulation_files(directory_to_scan)

    if not all_files_data:
        print("No matching files found. Check your directory path and file extensions.")
        exit()

    metrics = []
    for run in all_files_data:
        df = run['data']
        t = df['time_ms'].values
        ax_vals = df['Ax'].values
        ay_vals = df['Ay'].values
        
        stab_x, rise_x, sse_x, over_x = calculate_metrics(t, ax_vals)
        stab_y, rise_y, sse_y, over_y = calculate_metrics(t, ay_vals)

        metrics.append({
            "Start_Ax": ax_vals[0],
            "Stab_Time_X": stab_x, "Rise_Time_X": rise_x, "SSE_X": sse_x, "Over_X": over_x,
            "Start_Ay": ay_vals[0],
            "Stab_Time_Y": stab_y, "Rise_Time_Y": rise_y, "SSE_Y": sse_y, "Over_Y": over_y
        })

    pm_df = pd.DataFrame(metrics)

    pm_df['Abs_Ax'] = pm_df['Start_Ax'].abs()
    pm_df['Is_Positive_X'] = pm_df['Start_Ax'] >= 0
    df_x = pm_df.sort_values(['Is_Positive_X', 'Abs_Ax']).copy()
    df_x['Group_X'] = df_x.groupby('Is_Positive_X')['Abs_Ax'].transform(lambda x: (x.diff().abs() > 5.0).cumsum())

    grouped_x = df_x.groupby(['Is_Positive_X', 'Group_X']).agg(
        Mean_Abs_Ax=('Abs_Ax', 'mean'), Std_Abs_Ax=('Abs_Ax', get_std),
        Mean_Stab_X=('Stab_Time_X', 'mean'), Std_Stab_X=('Stab_Time_X', get_std),
        Mean_Rise_X=('Rise_Time_X', 'mean'), Std_Rise_X=('Rise_Time_X', get_std),
        Mean_SSE_X=('SSE_X', 'mean'), Std_SSE_X=('SSE_X', get_std),
        Mean_Over_X=('Over_X', 'mean'), Std_Over_X=('Over_X', get_std)
    ).reset_index()

    pm_df['Abs_Ay'] = pm_df['Start_Ay'].abs()
    pm_df['Is_Positive_Y'] = pm_df['Start_Ay'] >= 0
    df_y = pm_df.sort_values(['Is_Positive_Y', 'Abs_Ay']).copy()
    df_y['Group_Y'] = df_y.groupby('Is_Positive_Y')['Abs_Ay'].transform(lambda x: (x.diff().abs() > 5.0).cumsum())

    grouped_y = df_y.groupby(['Is_Positive_Y', 'Group_Y']).agg(
        Mean_Abs_Ay=('Abs_Ay', 'mean'), Std_Abs_Ay=('Abs_Ay', get_std),
        Mean_Stab_Y=('Stab_Time_Y', 'mean'), Std_Stab_Y=('Stab_Time_Y', get_std),
        Mean_Rise_Y=('Rise_Time_Y', 'mean'), Std_Rise_Y=('Rise_Time_Y', get_std),
        Mean_SSE_Y=('SSE_Y', 'mean'), Std_SSE_Y=('SSE_Y', get_std),
        Mean_Over_Y=('Over_Y', 'mean'), Std_Over_Y=('Over_Y', get_std)
    ).reset_index()

    print("\n" + "="*50)
    print("EXPERIMENTAL ANALYSIS EXTRACTION")
    print("="*50)

    stab_stats = plot_symmetry_panel(grouped_x, grouped_y, 'Mean_Stab_X', 'Std_Stab_X', 'Mean_Stab_Y', 'Std_Stab_Y', 
                        'Stabilisation Time', 'Stabilisation Time (ms)', fit_mode='asymptote')
    print("\n1. EMPIRICAL STABILISATION TIME (Regime Analysis)")
    for key, val in stab_stats.items():
        if 'Asymp' in key:
            print(f"   {key.replace('_Asymp', '')} Saturation Plateau: {val[0]:.1f} +/- {val[1]:.1f} ms")

    rise_stats = plot_symmetry_panel(grouped_x, grouped_y, 'Mean_Rise_X', 'Std_Rise_X', 'Mean_Rise_Y', 'Std_Rise_Y', 
                        'Rise Time', 'Rise Time (ms)', fit_mode='linear')
    print("\n2. RISE TIME (Linear Regression)")
    for key, res in rise_stats.items():
        print(f"   {key}: Gradient = {res.slope:.2f} +/- {res.stderr:.2f} ms/deg | R^2 = {res.rvalue**2:.3f}")

    sse_stats = plot_symmetry_panel(grouped_x, grouped_y, 'Mean_SSE_X', 'Std_SSE_X', 'Mean_SSE_Y', 'Std_SSE_Y', 
                        'Signed Steady State Error', 'Signed SSE (Degrees)', fit_mode='linear')
    print("\n3. STEADY STATE ERROR (Linear Regression)")
    for key, res in sse_stats.items():
        print(f"   {key}: Gradient = {res.slope:.4f} +/- {res.stderr:.4f} deg/deg | Intercept = {res.intercept:.3f} +/- {res.intercept_stderr:.3f} deg | p-val = {res.pvalue:.4f}")

    over_stats = plot_symmetry_panel(grouped_x, grouped_y, 'Mean_Over_X', 'Std_Over_X', 'Mean_Over_Y', 'Std_Over_Y', 
                        'Overshoot', 'Overshoot (%)', fit_mode='mean')
    print("\n4. OVERSHOOT (Global Mean)")
    for key, val in over_stats.items():
        print(f"   {key.replace('_Overall', '')}: Mean = {val[0]:.2f} +/- {val[1]:.2f} %")