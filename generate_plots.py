import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

algorithms = {
    "OV²SLAM Accurate Stereo": "/home/amaraly/ROS2-PROJECT-SPRING2026/results/ov2slam_benchmark_statistical/20260605_204652",
    "OV²SLAM Fast Stereo": "/home/amaraly/ROS2-PROJECT-SPRING2026/results/ov2slam_benchmark_statistical/20260610_005350",
    "ORB-SLAM2 Stereo": "/home/amaraly/ORB_SLAM2/results/20260609_003027",
    "ORB-SLAM2 Mono": "/home/amaraly/ORB_SLAM2/results/20260609_092931",
    "ORB-SLAM3 Stereo": {
        "default": "/home/amaraly/ORBSLAM3_ROS2/results/orbslam_benchmark/20260608_101634",
        "exceptions": {"MH_03_medium": "/home/amaraly/ORBSLAM3_ROS2/results/orbslam_benchmark/20260608_204508", "V2_02_medium": "/home/amaraly/ORBSLAM3_ROS2/results/orbslam_benchmark/20260608_204508"}
    }
}

sequences = [
    "MH_01_easy", "MH_02_easy", "MH_03_medium", "MH_04_difficult", "MH_05_difficult",
    "V1_01_easy", "V1_02_medium", "V1_03_difficult", "V2_01_easy", "V2_02_medium"
]
runs = [f"run_{i:02d}" for i in range(1, 11)]

os.makedirs("SLAM_Images", exist_ok=True)

results = {}
for algo in algorithms:
    results[algo] = {}
    for seq in sequences:
        results[algo][seq] = {}
        for run in runs:
            results[algo][seq][run] = {'frontend': np.nan, 'cpu': np.nan, 'traj': None}

for algo, path_info in algorithms.items():
    if isinstance(path_info, dict):
        base_path = path_info['default']
        exc = path_info['exceptions']
    else:
        base_path = path_info
        exc = {}
        
    for seq in sequences:
        actual_path = exc.get(seq, base_path)
        seq_dir = os.path.join(actual_path, seq)
        if not os.path.isdir(seq_dir): continue
        
        for run in runs:
            run_dir = os.path.join(seq_dir, run)
            if not os.path.isdir(run_dir): continue
            
            # Trajectory
            traj_file = os.path.join(run_dir, "trajectory.tum")
            if os.path.isfile(traj_file):
                results[algo][seq][run]['traj'] = traj_file
                
            # Frontend
            if "OV²SLAM" in algo:
                timing_file = os.path.join(run_dir, "ov2slam_timings.csv")
                if os.path.isfile(timing_file):
                    df = pd.read_csv(timing_file)
                    row = df[df['timer'] == '0.Full-Front_End']
                    if not row.empty:
                        results[algo][seq][run]['frontend'] = float(row['mean_ms'].iloc[0])
            else:
                timing_file = os.path.join(run_dir, "timing_summary.csv")
                if os.path.isfile(timing_file):
                    df = pd.read_csv(timing_file)
                    row = df[df['category'] == 'frontend/full_tracking']
                    if not row.empty:
                        results[algo][seq][run]['frontend'] = float(row['avg_ms'].iloc[0])

# CPU pass
for algo, path_info in algorithms.items():
    if isinstance(path_info, dict):
        paths = set([path_info['default']] + list(path_info['exceptions'].values()))
    else:
        paths = [path_info]
        
    for p in paths:
        exp_file = os.path.join(p, "experiment_summary.csv")
        if os.path.isfile(exp_file):
            df = pd.read_csv(exp_file)
            for _, row in df.iterrows():
                seq = row['sequence']
                if seq not in sequences: continue
                
                if "OV²SLAM" in algo:
                    if 'run_name' in row:
                        run = row['run_name']
                        if run in runs and pd.notna(row['cpu_total_avg_pct']):
                            results[algo][seq][run]['cpu'] = float(row['cpu_total_avg_pct'])
                else:
                    if pd.notna(row['cpu_total_avg_pct']):
                        for run in runs:
                            results[algo][seq][run]['cpu'] = float(row['cpu_total_avg_pct'])

# Plot 1 & 2
algo_names = list(algorithms.keys())
frontend_avgs = []
cpu_avgs = []

for algo in algo_names:
    seq_frontend = []
    seq_cpu = []
    for seq in sequences:
        r_f = [results[algo][seq][run]['frontend'] for run in runs if pd.notna(results[algo][seq][run]['frontend'])]
        r_c = [results[algo][seq][run]['cpu'] for run in runs if pd.notna(results[algo][seq][run]['cpu'])]
        if r_f: seq_frontend.append(np.mean(r_f))
        if r_c: seq_cpu.append(np.mean(r_c))
        
    frontend_avgs.append(np.mean(seq_frontend) if seq_frontend else 0)
    cpu_avgs.append(np.mean(seq_cpu) if seq_cpu else 0)

plt.figure(figsize=(10, 6))
plt.bar(algo_names, frontend_avgs, color='skyblue')
plt.xticks(rotation=45, ha="right")
plt.ylabel("Frontend Time (ms)")
plt.title("Frontend Time Averaged Across Sequences")
plt.tight_layout()
plt.savefig("SLAM_Images/frontend_time_avg.png")
plt.close()

plt.figure(figsize=(10, 6))
plt.bar(algo_names, cpu_avgs, color='salmon')
plt.xticks(rotation=45, ha="right")
plt.ylabel("CPU Usage (%)")
plt.title("CPU Usage Averaged Across Sequences")
plt.tight_layout()
plt.savefig("SLAM_Images/cpu_usage_avg.png")
plt.close()

# Plot 3
for algo in algo_names:
    seqs = []
    avgs = []
    for seq in sequences:
        r_f = [results[algo][seq][run]['frontend'] for run in runs if pd.notna(results[algo][seq][run]['frontend'])]
        if r_f:
            seqs.append(seq)
            avgs.append(np.mean(r_f))
            
    if avgs:
        plt.figure(figsize=(12, 6))
        plt.bar(seqs, avgs, color='lightgreen')
        plt.xticks(rotation=45, ha="right")
        plt.ylabel("Frontend Time (ms)")
        plt.title(f"Frontend Time per Sequence - {algo}")
        plt.tight_layout()
        safe_algo = algo.replace(" ", "_").replace("²", "2")
        plt.savefig(f"SLAM_Images/frontend_time_seq_{safe_algo}.png")
        plt.close()

# Plot 4
try:
    colors = plt.cm.tab10.colors
except AttributeError:
    colors = plt.get_cmap('tab10').colors

latex_code = "\\begin{table*}[h]\n\\centering\n\\resizebox{\\textwidth}{!}{\n\\begin{tabular}{c|" + "c"*10 + "}\n"
latex_code += "Sequence & " + " & ".join([f"Run {i}" for i in range(1, 11)]) + " \\\\\\hline\n"

for seq in sequences:
    row_latex = [seq.replace('_', '\\_')]
    for run in runs:
        plt.figure(figsize=(4, 4))
        has_data = False
        for idx, algo in enumerate(algo_names):
            traj_file = results[algo][seq][run]['traj']
            if traj_file and os.path.isfile(traj_file):
                try:
                    df = pd.read_csv(traj_file, sep=r'\s+', header=None, comment='#', dtype=float)
                    if len(df.columns) >= 3:
                        tx = df[1].values
                        ty = df[2].values
                        plt.plot(tx, ty, label=algo, color=colors[idx % len(colors)], linewidth=1)
                        has_data = True
                except Exception as e:
                    pass
        if has_data:
            plt.legend(fontsize=6)
            plt.axis('equal')
        plt.title(f"{seq} {run}", fontsize=8)
        plt.xticks([])
        plt.yticks([])
        img_name = f"traj_xy_{seq}_{run}.png"
        plt.savefig(f"SLAM_Images/{img_name}", dpi=150, bbox_inches='tight')
        plt.close()
        
        row_latex.append(f"\\includegraphics[width=\\linewidth]{{SLAM_Images/{img_name}}}")
    
    latex_code += " & ".join(row_latex) + " \\\\\n"

latex_code += "\\end{tabular}\n}\n\\caption{Trajectory XY Plots}\n\\label{tab:traj_xy}\n\\end{table*}\n"

with open("latex_table.tex", "w") as f:
    f.write(latex_code)
    
print("Successfully generated all plots and LaTeX table.")
