"""Experiment plug-ins. Each module registers one RunKind with factory.register(name).

Importing this package imports every kind, so the registry is complete.
A kind never imports another kind (scripts/check_hilbench_layers.py checks it).
"""
import importlib
import pkgutil


def load_all():
    """Import every module in this package except base and helpers whose name starts with '_'."""
    for m in pkgutil.iter_modules(__path__):
        if m.name != "base" and not m.name.startswith("_"):
            importlib.import_module(f"{__name__}.{m.name}")
