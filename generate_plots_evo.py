import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

try:
    from evo.tools import file_interface
    from evo.core import sync
    has_evo = True
except ImportError:
    has_evo = False

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

# Plot 4
algo_names = list(algorithms.keys())
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
        
        plotted_gt = False
        for idx, algo in enumerate(algo_names):
            traj_file = results[algo][seq][run]['traj']
            if traj_file and os.path.isfile(traj_file):
                g = os.path.join(os.path.dirname(traj_file), "gt.tum")
                traj_ref = None
                if has_evo and os.path.isfile(g):
                    try:
                        traj_ref = file_interface.read_tum_trajectory_file(g)
                    except:
                        pass

                if has_evo:
                    try:
                        traj_est = file_interface.read_tum_trajectory_file(traj_file)
                        if traj_ref is not None:
                            traj_ref_sync, traj_est_sync = sync.associate_trajectories(traj_ref, traj_est, max_diff=0.01)
                            if traj_est_sync.num_poses > 2:
                                try:
                                    traj_est_sync.align(traj_ref_sync, correct_scale=True, correct_only_scale=False)
                                except Exception as e:
                                    pass
                                tx = traj_est_sync.positions_xyz[:, 0]
                                ty = traj_est_sync.positions_xyz[:, 1]
                                plt.plot(tx, ty, label=algo, color=colors[idx % len(colors)], linewidth=1)
                                has_data = True
                                
                                if not plotted_gt:
                                    gt_x = traj_ref_sync.positions_xyz[:, 0]
                                    gt_y = traj_ref_sync.positions_xyz[:, 1]
                                    plt.plot(gt_x, gt_y, label='Ground Truth', color='black', linestyle='--', linewidth=1.5)
                                    plotted_gt = True
                            else:
                                tx = traj_est.positions_xyz[:, 0]
                                ty = traj_est.positions_xyz[:, 1]
                                plt.plot(tx, ty, label=algo, color=colors[idx % len(colors)], linewidth=1)
                                has_data = True
                        else:
                            tx = traj_est.positions_xyz[:, 0]
                            ty = traj_est.positions_xyz[:, 1]
                            plt.plot(tx, ty, label=algo, color=colors[idx % len(colors)], linewidth=1)
                            has_data = True
                    except Exception as e:
                        print("Error with evo for", algo, seq, run, ":", e)
                else:
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

latex_code += "\\end{tabular}\n}\n\\caption{Trajectory XY Plots aligned to Ground Truth}\n\\label{tab:traj_xy}\n\\end{table*}\n"

with open("latex_table.tex", "w") as f:
    f.write(latex_code)
    
print("Successfully generated all plots and LaTeX table.")
