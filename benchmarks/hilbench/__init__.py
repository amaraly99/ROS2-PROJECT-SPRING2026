"""hilbench — the shared HIL benchmark harness (approved design: benchmarks/HILBENCH_DESIGN.html).

One package, many experiments. The core (every module outside kinds/) never names an
experiment; each experiment is a RunKind plug-in in kinds/. Entry points call
factory.main(kind_name, argv). The contract all modules follow is in CONTRACT.md.
"""
