# MATLAB-side HIL automation

## What you (the operator) do per experiment session

1. On the Windows laptop, open MATLAB and `cd` to the folder with the HIL
   Simulink model.
2. Open `hil_run_supervisor.m` (copy it over from this repo), set `MODEL_NAME`
   at the top to your model's name (without `.slx`).
3. Run the script. It listens on TCP port **55556** and prints every command.
4. Leave it running. Start sweeps from the Pi:
   ```bash
   ./benchmarks/sweep_detectors.sh                 # full 20-config sweep
   ./benchmarks/run_hil_experiments.sh --label smoke --runs 1 --duration 30
   ```
5. When the sweep finishes, Ctrl+C the supervisor.

The Pi sends `start` before each run — the supervisor stops and restarts the
model, which re-applies all initial conditions (drone back at its starting
pose) — and `stop` after each run.

**Firewall note:** the first time MATLAB opens the port, Windows will ask to
allow it. Allow on the VirtualBox host-only / private network. The Pi connects
from 192.168.56.2 to 192.168.56.1:55556.

## If you don't run the supervisor

Everything still works: the Pi falls back to prompting
`(Re)start the Simulink simulation manually, then press Enter...` between
runs. With `NONINTERACTIVE=1` it skips the prompt and assumes the sim is
already streaming (no per-run drone reset — fine for quick smoke tests,
wrong for comparable closed-loop trajectories).

## Quick test of the supervisor from the Pi

```bash
echo start | nc -w 3 192.168.56.1 55556   # should print "ok" and start the sim
echo stop  | nc -w 3 192.168.56.1 55556   # should print "ok" and stop it
```
