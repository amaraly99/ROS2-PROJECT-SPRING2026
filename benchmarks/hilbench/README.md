# hilbench — the shared HIL benchmark harness

One package, many experiments. The core (every module outside `kinds/`) never names an
experiment. Each experiment is a `RunKind` plug-in in `kinds/`. Design: `benchmarks/HILBENCH_DESIGN.html`.
Exact names and signatures: `CONTRACT.md`.

## Run

Use `~/.venvs/hilrun/bin/python` (has rich and PyYAML). In MATLAB on Windows first: `cd <repo>\matlab ; hil_session_start`.

    ./run_controller_benchmark2.sh                    # controller kind, settings from config/hil/controller_bench.yaml
    ./run_controller_benchmark2.sh --runs 10 --camera-pub off     # a command-line option wins over the file
    ./run_controller_benchmark2.sh --dry-run          # preflight only (Wi-Fi, MATLAB)
    ~/.venvs/hilrun/bin/python benchmarks/hilbench_run.py controller --help     # every option

A session writes `reports/<kind>_<tag>_<stamp>/` (session.log, deviations.log, runs.csv, settings/, summary.md).
Keys during a session: `p` pause after this run, `a` abort, `l` show/hide the bench log.
The old runner `ctrl_runner.py` stays until the controller campaign ends.

## Layout

    util  bench_geometry                      L0  helpers, shared numbers
    config                                    L1  YAML + command line -> frozen Settings
    supervisor net pi_state container gate
    model_check overhead                      L2  one job each; none imports another
    trial ui summary                          L3  one trial (the ladder), the screen, the summary
    session                                   L4  the loop over trials
    kinds/*                                   L5  one plug-in per experiment
    factory                                   L6  registry + main()

A module imports only modules below it. `scripts/check_hilbench_layers.py` checks it and also checks that no
core file names a kind.

## Settings

`config/hil/<kind>_bench.yaml`: sections `session`, `matlab`, `network`, `pi` (common) and one section named
like the kind. Quote word values (`"off"`, `"none"`): YAML reads bare off/on as booleans. Order of precedence:
built-in default, then the YAML file, then the command line.
`config/hil/models.yaml` lists what each Simulink model must contain; `model_check` compares it with MATLAB's `model_info`.

## Add a kind

1. Write `kinds/foo.py` with `@register("foo")` on a class that fills the slots of `kinds/base.py::RunKind`
   (`fields`, `conditions`, `matlab_setup`, `boot`, `go`, `fly`, `stop`, `score`, `read_results`, `table_row`, plus `columns`,
   `summary_script`, `bench_kill_names`).
2. Add `config/hil/foo_bench.yaml` (common sections + a `foo:` section).
3. Add `benchmarks/foo_runner.py` that calls `main("foo", sys.argv[1:])`.

No other file changes. Optional attributes the core reads from a kind: `n_topics` (recorder topics, default 7),
`step_labels` (screen names of the ladder steps), `run_title(cond)`, `title` (header), `cond_key` (name of the condition
column in runs.csv).
