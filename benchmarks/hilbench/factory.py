"""factory.py — the registry and the entry point. The only module that knows kind names exist.

    from hilbench.factory import main
    sys.exit(main("controller", sys.argv[1:]))
"""
import inspect

from .kinds.base import RunKind

_REGISTRY = {}


def register(name):
    """Class decorator: @register("slam"). Refuses duplicates and incomplete classes."""
    def deco(cls):
        if name in _REGISTRY and _REGISTRY[name] is not cls:
            raise ValueError(f"kind {name!r} is already registered by {_REGISTRY[name].__name__}")
        if not issubclass(cls, RunKind):
            raise TypeError(f"{cls.__name__} is not a RunKind")
        if inspect.isabstract(cls):
            missing = ", ".join(sorted(cls.__abstractmethods__))
            raise TypeError(f"{cls.__name__} does not fill these RunKind slots: {missing}")
        cls.name = name
        _REGISTRY[name] = cls
        return cls
    return deco


def kinds_available():
    _load()
    return sorted(_REGISTRY)


def kind_class(name):
    _load()
    if name not in _REGISTRY:
        raise KeyError(f"unknown kind {name!r}; known kinds: {', '.join(sorted(_REGISTRY)) or 'none'}")
    return _REGISTRY[name]


def make_kind(name, settings, services):
    return kind_class(name)(settings, services)


def _load():
    from . import kinds
    kinds.load_all()


def main(kind_name, argv=None):
    """Parse settings, build the session, run it. Returns the exit code."""
    from . import config
    from .session import Session
    cls = kind_class(kind_name)
    settings = config.load(argv, cls)
    return Session(settings, cls).run()
