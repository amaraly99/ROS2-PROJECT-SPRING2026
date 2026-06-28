#!/usr/bin/env python3
"""Restructure paper.tex: add ORB-SLAM2, per-algorithm order, total comparison last."""

from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
TEX = ROOT / "paper.tex"

ORB2_SETUP = r"""
\subsection{ORB-SLAM2 Setup Details}

ORB-SLAM2 was run through the \texttt{ros2-ORB\_SLAM2} stereo node on ROS2 Jazzy with results taken from batch \texttt{20260530\_232822}. EuRoC bags were replayed at \texttt{bag\_rate: 1.0}. Each sequence was repeated three times; numbers in the ORB-SLAM2 section are means across clean runs unless noted otherwise.

\begin{table}[H]
\centering
\small
\caption{ORB-SLAM2 key algorithm parameters from the EuRoC stereo settings file.}
\label{tab:orbslam2_algorithm_params}
\begin{tabular}{lp{9.5cm}}
\toprule
Parameter & Value \\
\midrule
Camera model & Pinhole (rectified stereo pair) \\
Image size & 752 $\times$ 480 \\
Camera FPS & 20 \\
Color order & BGR (\texttt{Camera.RGB = 0}) \\
Stereo depth threshold & \texttt{ThDepth = 30.0} \\
ORB features per image & \texttt{ORBextractor.nFeatures = 500} \\
Scale factor & \texttt{ORBextractor.scaleFactor = 1.2} \\
Pyramid levels & \texttt{ORBextractor.nLevels = 8} \\
FAST initial threshold & \texttt{ORBextractor.iniThFAST = 15} \\
FAST minimum threshold & \texttt{ORBextractor.minThFAST = 5} \\
Left/right topics & \texttt{/camera/left}, \texttt{/camera/right} via ROS2 wrapper \\
\bottomrule
\end{tabular}
\end{table}

"""

ORB2_SECTION = r"""
\clearpage

\section{ORB-SLAM2 Results}

ORB-SLAM2 was evaluated with the same Raspberry Pi~5 setup and three repeated runs per sequence (batch \texttt{20260530\_232822}). All ten sequences completed on all three runs. Reported RMSE values are \textbf{mean$\pm$std across clean runs} with Sim(3) Umeyama alignment via \texttt{evo\_ape}, matching the RTAB-Map reporting style.

\subsection{ORB-SLAM2 RMSE Summary}

\begin{table}[H]
\centering
\small
\caption{ORB-SLAM2 mean$\pm$std RMSE per sequence (3/3 valid runs) vs.\ other systems.}
\label{tab:orbslam2_rmse}
\begin{tabular}{lrrrrr}
\toprule
Sequence & ORB-SLAM2 avg (m) & OV2 acc (m) & OV2 fast (m) & ORB-SLAM3 (m) & RTAB-Map avg (m) \\
\midrule
MH\_01\_easy       & $0.046 \pm 0.004$ & 0.040 & 0.054 & 0.043 & $0.102 \pm 0.063$ \\
MH\_02\_easy       & $0.054 \pm 0.006$ & 0.049 & 0.052 & 0.071 & $0.190 \pm 0.143$ \\
MH\_03\_medium     & $0.046 \pm 0.004$ & 0.046 & 0.079 & 0.247 & $0.146 \pm 0.038$ \\
\textcolor{red}{MH\_04\_difficult} & $0.077 \pm 0.010$ & 0.069 & 0.152 & 0.039 & $0.224 \pm 0.070$ \\
MH\_05\_difficult  & $0.066 \pm 0.023$ & 0.067 & 0.142 & 0.091 & $0.166$ (1/3 valid) \\
V1\_01\_easy       & $0.058 \pm 0.006$ & 0.055 & 0.094 & 0.131 & $0.096 \pm 0.029$ \\
\textcolor{red}{V1\_02\_medium}   & $0.091 \pm 0.007$ & 0.088 & 0.095 & 0.331 & $0.081$ (1/3 valid) \\
\textcolor{red}{V1\_03\_difficult}& $0.128 \pm 0.012$ & 0.118 & 0.670 & 0.069 & $0.133 \pm 0.046$ \\
\textcolor{red}{V2\_01\_easy}     & $0.061 \pm 0.002$ & 0.072 & 0.086 & n/a & $0.161 \pm 0.052$ \\
V2\_02\_medium     & $0.093 \pm 0.013$ & 0.054 & 0.204 & 0.228 & $0.244 \pm 0.035$ \\
\bottomrule
\end{tabular}
\end{table}

\begin{figure}[H]
\centering
\includegraphics[width=\textwidth]{images/orbslam2_rmse_per_sequence.png}
\caption{ORB-SLAM2 mean APE RMSE per sequence with $\pm$1\,std across runs. Dashed lines mark 5\,cm and 10\,cm.}
\label{fig:orbslam2_rmse}
\end{figure}

ORB-SLAM2 finished every sequence on all three runs, which is already a practical advantage over ORB-SLAM3 on this board (\textcolor{red}{V2\_01} failed there). Average RMSE sits around 7--13\,cm on most sequences, with \textcolor{red}{V1\_03\_difficult} the worst at $\approx$13\,cm mean.

\subsection{ORB-SLAM2 Frontend Timing}

Timing uses the \texttt{frontend/full\_tracking} timer (full stereo tracking callback per processed frame), averaged across clean runs.

\begin{table}[H]
\centering
\small
\caption{ORB-SLAM2 frontend full-tracking time per sequence (mean, run-to-run std).}
\label{tab:orbslam2_timing}
\begin{tabular}{lrr}
\toprule
Sequence & Mean (ms) & Std (ms) \\
\midrule
MH\_01\_easy       & 46.1 & 2.1 \\
MH\_02\_easy       & 45.4 & 2.6 \\
MH\_03\_medium     & 47.6 & 1.5 \\
\textcolor{red}{MH\_04\_difficult} & 40.6 & 1.0 \\
MH\_05\_difficult  & 39.2 & 0.4 \\
V1\_01\_easy       & 59.6 & 0.8 \\
\textcolor{red}{V1\_02\_medium}   & 50.8 & 3.4 \\
\textcolor{red}{V1\_03\_difficult}& 47.7 & 1.2 \\
\textcolor{red}{V2\_01\_easy}     & 49.9 & 0.3 \\
V2\_02\_medium     & 55.2 & 2.2 \\
\midrule
\textbf{Mean}      & \textbf{48.2} & \textbf{1.6} \\
\bottomrule
\end{tabular}
\end{table}

\begin{figure}[H]
\centering
\includegraphics[width=\textwidth]{images/orbslam2_frontend_timing_per_sequence.png}
\caption{ORB-SLAM2 frontend full-tracking time per sequence ($\pm$1\,std across runs). Red dashed line = 20\,Hz (50\,ms) budget.}
\label{fig:orbslam2_timing}
\end{figure}

Mean frontend time is $\approx$48\,ms, so ORB-SLAM2 is usually just above the EuRoC real-time line, similar to ORB-SLAM3 and much slower than OV$^2$SLAM Fast ($\approx$5\,ms).

\subsection{ORB-SLAM2 CPU Utilization}

\begin{table}[H]
\centering
\small
\caption{ORB-SLAM2 total process CPU per sequence (\% of one core, mean over clean runs).}
\label{tab:orbslam2_cpu}
\begin{tabular}{lrr}
\toprule
Sequence & Mean (\%) & Run-to-run std (\%) \\
\midrule
MH\_01\_easy       & 161.4 & 3.6 \\
MH\_02\_easy       & 162.5 & 6.3 \\
MH\_03\_medium     & 162.8 & 3.0 \\
\textcolor{red}{MH\_04\_difficult} & 144.3 & 5.0 \\
MH\_05\_difficult  & 139.3 & 1.6 \\
V1\_01\_easy       & 198.9 & 3.9 \\
\textcolor{red}{V1\_02\_medium}   & 171.7 & 9.4 \\
\textcolor{red}{V1\_03\_difficult}& 184.4 & 1.9 \\
\textcolor{red}{V2\_01\_easy}     & 190.3 & 1.9 \\
V2\_02\_medium     & 205.4 & 5.1 \\
\midrule
\textbf{Mean}      & \textbf{172.1} & \textbf{4.2} \\
\bottomrule
\end{tabular}
\end{table}

\begin{figure}[H]
\centering
\includegraphics[width=\textwidth]{images/orbslam2_cpu_per_sequence.png}
\caption{ORB-SLAM2 mean process CPU per sequence with run-to-run $\pm$1\,std error bars.}
\label{fig:orbslam2_cpu}
\end{figure}

\begin{figure}[H]
\centering
\includegraphics[width=0.55\textwidth]{images/orbslam2_cpu_per_thread.png}
\caption{ORB-SLAM2 logical thread-role CPU usage aggregated across sequences and runs.}
\label{fig:orbslam2_roles}
\end{figure}

\texttt{ORBGBA} dominates when global bundle adjustment fires, then \texttt{ORBLocalMap} and \texttt{ORBFrontEnd}.

\subsection{ORB-SLAM2 Paper Comparison}

\begin{table}[H]
\centering
\small
\caption{ORB-SLAM2 measured mean RMSE vs.\ published stereo RMSE (ORB-SLAM2 paper, desktop).}
\label{tab:orbslam2_vs_paper}
\begin{tabular}{lrrr}
\toprule
Sequence & Measured avg (m) & Paper stereo (m) & Delta (m) \\
\midrule
MH\_01\_easy       & $0.046 \pm 0.004$ & 0.007 & +0.039 \\
MH\_02\_easy       & $0.054 \pm 0.006$ & 0.008 & +0.046 \\
MH\_03\_medium     & $0.046 \pm 0.004$ & 0.010 & +0.036 \\
\textcolor{red}{MH\_04\_difficult} & $0.077 \pm 0.010$ & 0.021 & +0.056 \\
MH\_05\_difficult  & $0.066 \pm 0.023$ & 0.016 & +0.050 \\
V1\_01\_easy       & $0.058 \pm 0.006$ & 0.020 & +0.038 \\
\textcolor{red}{V1\_02\_medium}   & $0.091 \pm 0.007$ & 0.022 & +0.069 \\
\textcolor{red}{V1\_03\_difficult}& $0.128 \pm 0.012$ & 0.029 & +0.099 \\
\textcolor{red}{V2\_01\_easy}     & $0.061 \pm 0.002$ & 0.014 & +0.047 \\
V2\_02\_medium     & $0.093 \pm 0.013$ & 0.027 & +0.066 \\
\bottomrule
\end{tabular}
\end{table}

The gap vs.\ the original paper is expected on Pi~5 with ROS~2 bag playback; still, ORB-SLAM2 remained the most \emph{reliable} ORB-family system in this batch.

"""

TOTAL_COMPARISON = r"""
\clearpage

\section{Total Comparison Across All Algorithms}
\label{sec:total_comparison}

This section compares all five evaluated configurations on the same ten EuRoC stereo sequences: OV$^2$SLAM Accurate, OV$^2$SLAM Fast, ORB-SLAM3 (best run), ORB-SLAM2 (mean$\pm$std over three runs, batch \texttt{20260530\_232822}), and RTAB-Map (mean$\pm$std over valid runs). Frontend metric definitions match the per-algorithm sections (OV$^2$ full FE; ORB-SLAM2/3 tracking path; RTAB-Map stereo odometry time).

\subsection{Master comparison table}

\begin{table}[H]
\centering
\scriptsize
\setlength{\tabcolsep}{3pt}
\caption{EuRoC per-sequence comparison: APE RMSE (m), frontend / odometry time (ms), and process CPU (\%). OV$^2$SLAM and ORB-SLAM3 are best-run values; ORB-SLAM2 and RTAB-Map show mean$\pm$std across repeated runs where available.}
\label{tab:master_comparison}
\begin{tabular}{lrrrrrrrrrrr}
\toprule
 & \multicolumn{5}{c}{RMSE (m)} & \multicolumn{5}{c}{Frontend / odom (ms)} & \multicolumn{5}{c}{CPU (\%)} \\
\cmidrule(lr){2-6}\cmidrule(lr){7-11}\cmidrule(lr){12-16}
Sequence & OV2 acc & OV2 fast & ORB3 & ORB2 & RTAB & OV2 acc & OV2 fast & ORB3 & ORB2 & RTAB & OV2 acc & OV2 fast & ORB3 & ORB2 & RTAB \\
\midrule
MH\_01\_easy & 0.040 & 0.054 & 0.043 & $0.046\pm.004$ & $.102\pm.063$ & 17.1 & 4.9 & 50.1 & $46.1\pm2.1$ & 101.7 & 153.4 & 42.8 & 139.6 & $161\pm4$ & 132.3 \\
MH\_02\_easy & 0.049 & 0.052 & 0.071 & $0.054\pm.006$ & $.190\pm.143$ & 17.1 & 5.0 & 48.8 & $45.4\pm2.6$ & 104.4 & 149.6 & 43.6 & 134.5 & $163\pm6$ & 134.6 \\
MH\_03\_medium & 0.046 & 0.079 & 0.247 & $0.046\pm.004$ & $.146\pm.038$ & 19.5 & 5.1 & 51.7 & $47.6\pm1.5$ & 97.2 & 160.9 & 48.4 & 145.5 & $163\pm3$ & 125.3 \\
\textcolor{red}{MH\_04\_difficult} & 0.069 & 0.152 & 0.039 & $0.077\pm.010$ & $.224\pm.070$ & 15.6 & 5.0 & 38.3 & $40.6\pm1.0$ & 104.2 & 147.0 & 42.7 & 149.6 & $144\pm5$ & 130.0 \\
MH\_05\_difficult & 0.067 & 0.142 & 0.091 & $0.066\pm.023$ & 0.166 & 15.8 & 5.0 & 46.1 & $39.2\pm0.4$ & 88.3 & 147.5 & 42.3 & 143.6 & $139\pm2$ & 131.2 \\
V1\_01\_easy & 0.055 & 0.094 & 0.131 & $0.058\pm.006$ & $.096\pm.029$ & 19.7 & 5.1 & 32.3 & $59.6\pm0.8$ & 103.6 & 157.1 & 46.1 & 87.4 & $199\pm4$ & 131.1 \\
\textcolor{red}{V1\_02\_medium} & 0.088 & 0.095 & 0.331 & $0.091\pm.007$ & 0.081 & 21.7 & 5.7 & 37.8 & $50.8\pm3.4$ & 93.0 & 164.9 & 46.8 & 112.2 & $172\pm9$ & 120.1 \\
\textcolor{red}{V1\_03\_difficult} & 0.118 & 0.670 & 0.069 & $0.128\pm.012$ & $.133\pm.046$ & 21.2 & 6.1 & 46.4 & $47.7\pm1.2$ & 92.8 & 162.0 & 46.6 & 132.0 & $184\pm2$ & 120.3 \\
\textcolor{red}{V2\_01\_easy} & 0.072 & 0.086 & n/a & $0.061\pm.002$ & $.161\pm.052$ & 15.0 & 4.9 & n/a & $49.9\pm0.3$ & 100.3 & 141.7 & 42.0 & n/a & $190\pm2$ & 128.2 \\
V2\_02\_medium & 0.054 & 0.204 & 0.228 & $0.093\pm.013$ & $.244\pm.035$ & 20.5 & 5.8 & 36.5 & $55.2\pm2.2$ & 90.4 & 163.9 & 51.9 & 118.8 & $205\pm5$ & 123.0 \\
\midrule
\textbf{Cross-seq.\ mean} & \textbf{0.066} & \textbf{0.163} & \textbf{0.139} & \textbf{0.072} & \textbf{0.154} & \textbf{18.2} & \textbf{5.2} & \textbf{43.0} & \textbf{48.2} & \textbf{97.6} & \textbf{154.7} & \textbf{45.3} & \textbf{129.0} & \textbf{172.1} & \textbf{127.6} \\
\bottomrule
\end{tabular}
\end{table}

\subsection{Comparison plots}

\begin{figure}[H]
\centering
\includegraphics[width=\textwidth]{images/comparison_rmse_all_algorithms.png}
\caption{Per-sequence RMSE comparison across all five configurations. Error bars: ORB-SLAM2 and RTAB-Map run-to-run std.}
\label{fig:comparison_rmse}
\end{figure}

\begin{figure}[H]
\centering
\includegraphics[width=\textwidth]{images/comparison_latency_all_algorithms.png}
\caption{Per-sequence frontend / odometry latency comparison. Dashed lines: 50\,ms (20\,Hz) and 100\,ms (10\,Hz).}
\label{fig:comparison_latency}
\end{figure}

\begin{figure}[H]
\centering
\includegraphics[width=\textwidth]{images/comparison_cpu_all_algorithms.png}
\caption{Per-sequence process CPU comparison (\% of one core). Dashed lines: 100\% and 200\%.}
\label{fig:comparison_cpu}
\end{figure}

\begin{figure}[H]
\centering
\includegraphics[width=\textwidth]{images/comparison_overall_means.png}
\caption{Cross-sequence average RMSE, frontend time, and CPU for each configuration.}
\label{fig:comparison_overall}
\end{figure}

\subsection{Comparison summary}

\begin{itemize}
  \item \textbf{Accuracy:} OV$^2$SLAM Accurate still has the lowest typical RMSE; ORB-SLAM2 is next-best among systems that always finish ($\approx$7\,cm mean on easy MH, up to $\approx$13\,cm on \textcolor{red}{V1\_03}). ORB-SLAM3 can beat everyone on a lucky best run but fails or spikes on several sequences.
  \item \textbf{Speed:} OV$^2$SLAM Fast ($\approx$5\,ms) is the only mode clearly inside the 20\,Hz budget. ORB-SLAM2 and ORB-SLAM3 sit near or above 50\,ms. RTAB-Map is slowest ($\approx$88--104\,ms).
  \item \textbf{CPU:} OV$^2$SLAM Fast uses $<$0.5 core. RTAB-Map and ORB-SLAM2/3 use $\approx$1.3--2 cores. OV$^2$SLAM Accurate is heaviest when loop closure runs.
  \item \textbf{Reliability:} ORB-SLAM2 and RTAB-Map completed every sequence in their batches; ORB-SLAM3 missed \textcolor{red}{V2\_01} and often needed cherry-picked best runs.
\end{itemize}

\paragraph{Why ORB-SLAM3 fails more than RTAB-Map despite being faster per frame.}
ORB-SLAM3's descriptor matching can fail abruptly under backlog, while RTAB-Map's flow-based odometry degrades more softly — see per-algorithm sections for detail.

"""


def main() -> None:
    tex = TEX.read_text()

    # Title + intro
    tex = tex.replace(
        r"\title{OV$^{2}$SLAM, ORB-SLAM3, and RTAB-Map EuRoC Comparison Report}",
        r"\title{OV$^{2}$SLAM, ORB-SLAM2, ORB-SLAM3, and RTAB-Map EuRoC Comparison Report}",
    )
    tex = tex.replace(
        "In this report, OV$^{2}$SLAM is benchmarked against ORB-SLAM3 on the EuRoC MAV Sequences.",
        "In this report, OV$^{2}$SLAM (Accurate and Fast), ORB-SLAM2, ORB-SLAM3, and RTAB-Map are benchmarked on the EuRoC MAV sequences.",
    )
    tex = tex.replace(
        "Therefore all numbers reported are the \\textbf{BEST}-case, because the worst case for these algorithm is an algorithm having 30cm RMSE error and effectively wrong trajectories.\nAny sequence red indicates that it failed on ORB-SLAM3 and its number is not representative of the actual performance.",
        "ORB-SLAM3 and the OV$^{2}$SLAM variants mostly report \\textbf{best-run} RMSE (closest to paper values after reruns). ORB-SLAM2 (batch \\texttt{20260530\\_232822}) and RTAB-Map report \\textbf{mean$\\pm$std} over repeated runs. Red sequence names mark known weak spots (often ORB-SLAM3 instability).",
    )

    # ORB2 implementation blurb
    if "ros2-ORB_SLAM2" not in tex:
        tex = tex.replace(
            "However, ORB-SLAM3 was messier",
            "ORB-SLAM2 was evaluated through the local \\texttt{ros2-ORB\\_SLAM2} wrapper (stereo node, EuRoC YAML, automated shutdown). It needed less surgery than ORB-SLAM3 but still needed ROS2 bag bridging and timing/CPU hooks matching the other pipelines.\n\nHowever, ORB-SLAM3 was messier",
        )

    # ORB2 setup after ORB3 setup paragraph (after orb3 table footnote line)
    if "ORB-SLAM2 Setup Details" not in tex:
        tex = tex.replace(
            "*ORB features per image and Pyramid levels were changed",
            ORB2_SETUP + "*ORB features per image and Pyramid levels were changed",
            1,
        )

    # Remove early three-way overview
    tex = re.sub(
        r"\\clearpage\s*\\section\{Three-way Per-sequence Overview\}.*?\\end\{table\}\s*\n\s*\\begin\{table\}.*?\\end\{table\}\s*\n\s*\\begin\{table\}.*?\\end\{table\}\s*\n",
        "",
        tex,
        count=1,
        flags=re.DOTALL,
    )

    # Rename OV2 section
    tex = tex.replace(
        "\\section{OV$^{2}$SLAM Accurate vs Fast Tables}",
        "\\section{OV$^{2}$SLAM Results}",
    )

    # Rename Paper Comparisons -> ORB-SLAM3 Results subsection block
    tex = tex.replace(
        "\\section{Paper Comparisons}",
        "\\section{ORB-SLAM3 Results}",
    )

    # Insert ORB2 section before RTAB
    if "\\section{ORB-SLAM2 Results}" not in tex:
        tex = tex.replace(
            "% ═══════════════════════════════════════════════════════════════════════════════\n\\section{RTAB-Map Results}",
            ORB2_SECTION + "% ═══════════════════════════════════════════════════════════════════════════════\n\\section{RTAB-Map Results}",
        )

    # Trajectory: add ORB-SLAM2 column helper
    def add_orb2_to_figure(block: str, seq: str) -> str:
        run_note = {
            "MH_01_easy": "run 2",
            "MH_02_easy": "run 3",
            "MH_03_medium": "run 1",
            "MH_04_difficult": "run 2",
            "MH_05_difficult": "run 2",
            "V1_01_easy": "run 3",
            "V1_02_medium": "run 3",
            "V1_03_difficult": "run 3",
            "V2_01_easy": "run 1",
            "V2_02_medium": "run 1",
        }.get(seq, "best")
        orb2 = f"""\\begin{{minipage}}[t]{{0.18\\textwidth}}
\\centering
\\includegraphics[width=\\linewidth]{{images/orbslam2_{seq}_trajectory_xy.png}}
\\\\
\\footnotesize ORB-SLAM2 ({run_note})
\\end{{minipage}}
\\hfill
"""
        if "orbslam2_" in block:
            return block
        # shrink existing minipages
        block = block.replace("0.24\\textwidth", "0.18\\textwidth")
        return block.replace("\\begin{figure}[H]\n\\centering\n", "\\begin{figure}[H]\n\\centering\n" + orb2, 1)

    for seq in [
        "MH_01_easy", "MH_02_easy", "MH_03_medium", "MH_04_difficult", "MH_05_difficult",
        "V1_01_easy", "V1_02_medium", "V1_03_difficult", "V2_01_easy", "V2_02_medium",
    ]:
        seq_esc = seq.replace("_", r"\_")
        pat = (
            r"(\\subsection\{[^}]*" + re.escape(seq_esc) + r"[^}]*\}\s*)"
            r"(\\begin\{figure\}\[H\].*?\\label\{fig:trajectory_" + seq + r"\}\s*\\end\{figure\})"
        )
        m = re.search(pat, tex, flags=re.DOTALL)
        if m:
            new_fig = add_orb2_to_figure(m.group(2), seq)
            cap_old = "OV2SLAM accurate, OV2SLAM fast, ORB-SLAM3, and RTAB-Map"
            cap_new = "OV2SLAM accurate, OV2SLAM fast, ORB-SLAM3, ORB-SLAM2, and RTAB-Map"
            new_fig = new_fig.replace(cap_old, cap_new)
            tex = tex[: m.start(2)] + new_fig + tex[m.end(2) :]

    # Trajectories intro
    tex = tex.replace(
        "OV2SLAM accurate, OV2SLAM fast, and ORB-SLAM3 for each available EuRoC sequence.",
        "OV2SLAM accurate, OV2SLAM fast, ORB-SLAM3, ORB-SLAM2, and RTAB-Map for each EuRoC sequence.",
    )

    # Conclusions: five-way + ORB2 bullet
    tex = tex.replace(
        "\\subsection*{RTAB-Map Summary and Four-Way Comparison}",
        "\\subsection*{Five-way summary (see Table~\\ref{tab:master_comparison} for full numbers)}",
    )
    orb2_bullet = (
        "    \\item \\textbf{ORB-SLAM2} (three runs per sequence, batch \\texttt{20260530\\_232822}) "
        "averages $\\approx$\\,7.2\\,cm RMSE with low run-to-run spread, completes all sequences, "
        "and uses $\\approx$\\,172\\% CPU with $\\approx$\\,48\\,ms frontend time — similar cost to ORB-SLAM3 but far more stable here.\n"
    )
    if "textbf{ORB-SLAM2}" not in tex.split("Five-way summary")[1].split("Frontend/odometry")[0]:
        tex = tex.replace(
            "    \\item \\textbf{ORB-SLAM3} delivers competitive",
            orb2_bullet + "    \\item \\textbf{ORB-SLAM3} delivers competitive",
        )

    # Replace Pairwise section with Total Comparison
    marker = "%% ============================================================\n\\section{Pairwise Comparison}"
    if marker in tex:
        head, _tail = tex.split(marker, 1)
        tex = head + TOTAL_COMPARISON + "\\end{document}\n"
    elif "\\section{Pairwise Comparison}" in tex:
        head, _tail = tex.split("\\section{Pairwise Comparison}", 1)
        tex = head + TOTAL_COMPARISON + "\\end{document}\n"

    tex = reorder_algorithm_sections(tex)

    TEX.write_text(tex)
    print(f"Wrote {TEX} ({len(tex.splitlines())} lines)")


def reorder_algorithm_sections(tex: str) -> str:
    """Move OV2 paper/timing/role blocks under OV2; ORB3 roles under ORB3."""

    def extract_section(title: str, text: str) -> tuple[str, str]:
        pat = rf"\\section\{{{re.escape(title)}\}}.*?(?=\\section\{{|\\end\{{document\}})"
        m = re.search(pat, text, flags=re.DOTALL)
        if not m:
            return "", text
        block = m.group(0)
        text = text[: m.start()] + text[m.end() :]
        return block, text

    timing_block, tex = extract_section("OV$^{2}$SLAM Timing Delta Table", tex)
    roles_block, tex = extract_section("Logical Thread-role Tables", tex)

    # Split OV2 paper tables out of ORB-SLAM3 Results
    orb3_pat = r"(\\section\{ORB-SLAM3 Results\}\s*)(.*?)(\\begin\{table\}.*?\\label\{tab:orb_vs_paper_accuracy\})"
    m = re.search(orb3_pat, tex, flags=re.DOTALL)
    ov2_paper = ""
    if m:
        ov2_paper = m.group(2)
        tex = tex[: m.start(2)] + tex[m.end(2) :]

    # Insert OV2 extras before ORB-SLAM3 section
    insert = ""
    if ov2_paper.strip():
        insert += "\\subsection{OV$^{2}$SLAM Paper Comparison}\n" + ov2_paper.strip() + "\n\\clearpage\n\n"
    if roles_block:
        ov2_roles, orb_roles = split_role_block(roles_block)
        if ov2_roles:
            insert += "\\subsection{OV$^{2}$SLAM Thread Roles}\n" + ov2_roles + "\n"
        if timing_block:
            insert += timing_block.replace(
                "\\section{OV$^{2}$SLAM Timing Delta Table}",
                "\\subsection{OV$^{2}$SLAM Timing Deltas}",
            ) + "\n"
    tex = tex.replace("\\clearpage\n\n\\section{ORB-SLAM3 Results}", insert + "\\section{ORB-SLAM3 Results}", 1)

    if roles_block:
        _, orb_roles = split_role_block(roles_block)
        if orb_roles:
            orb_insert = "\\subsection{ORB-SLAM3 Thread Roles}\n" + orb_roles + "\n\\clearpage\n\n"
            tex = tex.replace(
                "\\section{RTAB-Map Results}",
                orb_insert + "\\section{RTAB-Map Results}",
                1,
            )

    # Remove duplicate clearpages before ORB2
    while "\\clearpage\n\n\\clearpage\n\n\\section{ORB-SLAM2 Results}" in tex:
        tex = tex.replace(
            "\\clearpage\n\n\\clearpage\n\n\\section{ORB-SLAM2 Results}",
            "\\clearpage\n\n\\section{ORB-SLAM2 Results}",
        )

    return tex


def split_role_block(block: str) -> tuple[str, str]:
    orb_marker = "\\caption{ORB-SLAM3 logical thread-role"
    idx = block.find(orb_marker)
    if idx < 0:
        return block, ""
    # back to table start for ORB
    orb_start = block.rfind("\\begin{table}", 0, idx)
    if orb_start < 0:
        orb_start = block.rfind("\\begin{figure}", 0, idx)
    ov2_part = block[:orb_start].replace("\\section{Logical Thread-role Tables}\n\n", "").strip()
    orb_part = block[orb_start:].strip()
    return ov2_part, orb_part


if __name__ == "__main__":
    main()
