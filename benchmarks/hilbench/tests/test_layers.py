"""scripts/check_hilbench_layers.py: the guard itself, then the guard on the finished package."""
import importlib.util
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from ._fakes import ROOT

SCRIPT = ROOT / "scripts" / "check_hilbench_layers.py"


def load_guard():
    spec = importlib.util.spec_from_file_location("check_hilbench_layers", SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class Tree(unittest.TestCase):
    """A temporary hilbench/ tree: write(path, text) then call the guard with root=."""

    def setUp(self):
        self.guard = load_guard()
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.root = Path(self._tmp.name) / "hilbench"
        self.root.mkdir()

    def write(self, rel, text="x = 1\n"):
        p = self.root / rel
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(text)


class NamesGuard(Tree):
    def test_word_in_core_file_is_found(self):
        """in: net.py holds the comment '# the Controller node' (capital C)
        exp: one problem that names net.py
        bug: the check is case sensitive and misses 'Controller' or 'SLAM'."""
        self.write("net.py", "# the Controller node\n")
        out = self.guard.check_names(self.root)
        self.assertEqual(len(out), 1)
        self.assertIn("net.py", out[0])

    def test_slam_in_a_string_is_found(self):
        """in: ui.py holds the string 'reports/slam_x'
        exp: one problem (string data counts)
        bug: only code is scanned and a default path with the kind name hides in a string."""
        self.write("ui.py", "PATH = 'reports/slam_x'\n")
        self.assertEqual(len(self.guard.check_names(self.root)), 1)

    def test_allowed_identifier(self):
        """in: config.py holds 'controller_cores' only
        exp: []
        bug: the approved setting name is flagged and nobody can run the guard."""
        self.write("config.py", "FIELD = 'controller_cores'\ns.controller_cores = 1\n")
        self.assertEqual(self.guard.check_names(self.root), [])

    def test_allowed_identifier_does_not_hide_other_hits(self):
        """in: one line holds 'controller_cores' and also the word 'controller' alone
        exp: one problem
        bug: the allow-list switches the whole line off."""
        self.write("config.py", "x = 'controller_cores controller'\n")
        self.assertEqual(len(self.guard.check_names(self.root)), 1)

    def test_kinds_and_factory_are_exempt(self):
        """in: kinds/controller.py, kinds/slam.py and factory.py hold both words
        exp: []
        bug: the guard flags the places where the kind names belong."""
        for rel in ("kinds/controller.py", "kinds/slam.py", "factory.py"):
            self.write(rel, "# controller slam\n")
        self.assertEqual(self.guard.check_names(self.root), [])

    def test_tests_folder_is_not_scanned(self):
        """in: tests/test_x.py holds both words
        exp: []
        bug: the tests of the package break the guard."""
        self.write("tests/test_x.py", "# controller slam\n")
        self.assertEqual(self.guard.check_names(self.root), [])

    def test_report_has_file_and_line(self):
        """in: the word on line 3 of session.py
        exp: the problem text starts with 'session.py:3'
        bug: the operator cannot find the hit."""
        self.write("session.py", "a = 1\nb = 2\nc = 'slam'\n")
        self.assertTrue(self.guard.check_names(self.root)[0].startswith("session.py:3"))


class ImportsGuard(Tree):
    def test_downward_imports_are_fine(self):
        """in: session (L4) imports trial (L3), config (L1), util (L0); trial imports net (L2)
        exp: []
        bug: a legal downward import is flagged."""
        for m in ("util", "config", "net", "trial", "session"):
            self.write(f"{m}.py")
        self.write("session.py", "from .trial import Trial\nfrom . import config\nfrom .util import run\n")
        self.write("trial.py", "from . import net\nimport bench_geometry\n")
        self.assertEqual(self.guard.check_imports(self.root), [])

    def test_upward_import_is_found(self):
        """in: net.py (L2) imports session (L4)
        exp: one problem that names net.py and session
        bug: a low module reaches up and the layer order is a fiction."""
        self.write("net.py", "from .session import Session\n")
        self.write("session.py")
        out = self.guard.check_imports(self.root)
        self.assertEqual(len(out), 1)
        self.assertIn("net.py", out[0])
        self.assertIn("session", out[0])

    def test_same_layer_import_is_found(self):
        """in: supervisor.py (L2) imports net (L2)
        exp: one problem
        bug: modules of one layer depend on each other."""
        self.write("supervisor.py", "from .net import pct_from_dbm\n")
        self.write("net.py")
        self.assertEqual(len(self.guard.check_imports(self.root)), 1)

    def test_absolute_package_import_is_checked(self):
        """in: gate.py (L2) does 'from hilbench.session import Session'
        exp: one problem
        bug: only relative imports are checked."""
        self.write("gate.py", "from hilbench.session import Session\n")
        self.write("session.py")
        self.assertEqual(len(self.guard.check_imports(self.root)), 1)

    def test_import_inside_a_function_is_checked(self):
        """in: net.py has 'def f(): from .session import Session'
        exp: one problem
        bug: a lazy import hides an upward dependency."""
        self.write("net.py", "def f():\n    from .session import Session\n")
        self.write("session.py")
        self.assertEqual(len(self.guard.check_imports(self.root)), 1)

    def test_session_imports_factory_is_found(self):
        """in: session.py (L4) imports factory (L6)
        exp: one problem
        bug: session knows how kinds are registered."""
        self.write("session.py", "from .factory import kind_class\n")
        self.write("factory.py")
        self.assertEqual(len(self.guard.check_imports(self.root)), 1)

    def test_session_imports_a_kind_is_found(self):
        """in: session.py imports kinds.controller
        exp: one problem
        bug: the session knows a concrete kind."""
        self.write("session.py", "from .kinds.controller import ControllerKind\n")
        self.write("kinds/controller.py")
        self.write("kinds/__init__.py")
        self.assertEqual(len(self.guard.check_imports(self.root)), 1)

    def test_kind_imports_lower_layers_and_base(self):
        """in: kinds/controller.py imports config, trial, session, util, kinds.base and factory.register
        exp: []
        bug: a kind may not use the toolbox or the base contract."""
        for m in ("config", "trial", "session", "util", "factory"):
            self.write(f"{m}.py")
        self.write("kinds/base.py")
        self.write("kinds/__init__.py")
        self.write("kinds/controller.py", "from ..config import FieldSpec\nfrom ..trial import Outcome\n"
                   "from ..session import Session\nfrom ..util import run\nfrom .base import RunKind\n"
                   "from ..factory import register\n")
        self.assertEqual(self.guard.check_imports(self.root), [])

    def test_kind_imports_other_kind_is_found(self):
        """in: kinds/controller.py imports kinds/slam.py
        exp: one problem that says 'another kind'
        bug: kinds depend on each other and cannot be removed one by one."""
        self.write("kinds/__init__.py")
        self.write("kinds/slam.py")
        self.write("kinds/controller.py", "from .slam import SlamKind\n")
        out = self.guard.check_imports(self.root)
        self.assertEqual(len(out), 1)
        self.assertIn("another kind", out[0])

    def test_slam_may_import_slam_probe(self):
        """in: kinds/slam.py imports kinds/slam_probe.py
        exp: []
        bug: the one allowed kind-to-kind edge is rejected."""
        self.write("kinds/__init__.py")
        self.write("kinds/slam_probe.py")
        self.write("kinds/slam.py", "from .slam_probe import Probe\n")
        self.assertEqual(self.guard.check_imports(self.root), [])

    def test_probe_may_not_import_slam(self):
        """in: kinds/slam_probe.py imports kinds/slam.py (the edge in the wrong direction)
        exp: one problem
        bug: the exception is symmetric and a cycle is allowed."""
        self.write("kinds/__init__.py")
        self.write("kinds/slam.py")
        self.write("kinds/slam_probe.py", "from .slam import SlamKind\n")
        self.assertEqual(len(self.guard.check_imports(self.root)), 1)

    def test_controller_may_not_import_slam_probe(self):
        """in: kinds/controller.py imports kinds/slam_probe.py
        exp: one problem
        bug: the exception for slam is open to every kind."""
        self.write("kinds/__init__.py")
        self.write("kinds/slam_probe.py")
        self.write("kinds/controller.py", "from .slam_probe import Probe\n")
        self.assertEqual(len(self.guard.check_imports(self.root)), 1)

    def test_factory_imports_everything_below(self):
        """in: factory.py imports kinds.base, config, session
        exp: []
        bug: the top layer cannot wire the program."""
        self.write("kinds/__init__.py")
        self.write("kinds/base.py")
        self.write("config.py")
        self.write("session.py")
        self.write("factory.py", "from .kinds.base import RunKind\nfrom . import config\nfrom .session import Session\n")
        self.assertEqual(self.guard.check_imports(self.root), [])

    def test_third_party_and_stdlib_are_ignored(self):
        """in: net.py imports os, rich and yaml
        exp: []
        bug: a non-package import is judged as a layer violation."""
        self.write("net.py", "import os\nimport yaml\nfrom rich.table import Table\n")
        self.assertEqual(self.guard.check_imports(self.root), [])


class MainGuard(Tree):
    def test_main_exit_codes(self):
        """in: main() on a clean tree is not possible without a root argument, so run the script as a program
              on the real package: it must return 0 or 1 and print either 'clean' or the problems
        exp: return code is 0 when it prints 'clean', 1 otherwise
        bug: the script exits 0 although it printed problems (the build never fails)."""
        r = subprocess.run([sys.executable, str(SCRIPT)], capture_output=True, text=True)
        self.assertIn(r.returncode, (0, 1))
        self.assertEqual(r.returncode == 0, "clean" in r.stdout)


class FinishedPackage(unittest.TestCase):
    """These fail until the package is finished. They are the proof of rule 1 and rule 2."""

    def test_names_clean(self):
        """in: check_names() on benchmarks/hilbench/
        exp: [] (no controller or slam outside kinds/ and factory.py, except controller_cores)
        bug: the core names a kind (rule 1 of CONTRACT section 0)."""
        self.assertEqual(load_guard().check_names(), [])

    def test_imports_clean(self):
        """in: check_imports() on benchmarks/hilbench/
        exp: [] (every import goes down a layer; no kind imports another kind)
        bug: an upward import (rule 2 of CONTRACT section 0)."""
        self.assertEqual(load_guard().check_imports(), [])

    def test_script_exit_code_zero(self):
        """in: python scripts/check_hilbench_layers.py
        exp: exit code 0 and the text 'clean'
        bug: the script fails the build on a finished package."""
        r = subprocess.run([sys.executable, str(SCRIPT)], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)

    def test_every_core_module_has_a_layer(self):
        """in: every .py file of hilbench/ (except tests/)
        exp: its first module name is in the LAYERS table of the guard
        bug: a new module escapes the layer rules because the guard does not know it."""
        guard = load_guard()
        for p in guard.python_files(guard.HILBENCH):
            parts = guard._module_parts(p, guard.HILBENCH)
            if parts:
                self.assertIn(parts[0], guard.LAYERS, str(p))


if __name__ == "__main__":
    unittest.main()
