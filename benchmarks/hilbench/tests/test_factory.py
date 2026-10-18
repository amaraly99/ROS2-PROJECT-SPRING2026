"""factory.py: the registry and main()."""
import tempfile
import unittest
from unittest import mock

from hilbench import factory
from hilbench.kinds.base import RunKind

from ._fakes import FakeKind, make_services, make_settings


class RegistryTests(unittest.TestCase):
    def setUp(self):
        try:
            factory._load()                            # import the real kinds first, so patch.dict cannot undo them
        except ImportError:
            pass
        p = mock.patch.dict(factory._REGISTRY)     # every test leaves the registry as it was
        p.start()
        self.addCleanup(p.stop)

    def test_register_sets_name_and_returns_class(self):
        """in: @register('fake_a') on a complete RunKind subclass
        exp: the class is returned unchanged, cls.name == 'fake_a', kind_class('fake_a') is the class
        bug: the decorator wraps or loses the class."""
        class K(FakeKind):
            pass
        self.assertIs(factory.register("fake_a")(K), K)
        self.assertEqual(K.name, "fake_a")
        self.assertIs(factory.kind_class("fake_a"), K)

    def test_duplicate_name_raises_value_error(self):
        """in: register('fake_d') for class A, then register('fake_d') for class B
        exp: ValueError whose text names 'fake_d'
        bug: a second kind silently replaces the first."""
        class A(FakeKind):
            pass

        class B(FakeKind):
            pass
        factory.register("fake_d")(A)
        with self.assertRaises(ValueError) as cm:
            factory.register("fake_d")(B)
        self.assertIn("fake_d", str(cm.exception))

    def test_same_class_twice_is_allowed(self):
        """in: register('fake_s') twice for the same class (a module imported twice)
        exp: no error
        bug: re-import of a kind module breaks the registry."""
        class K(FakeKind):
            pass
        factory.register("fake_s")(K)
        factory.register("fake_s")(K)

    def test_incomplete_class_raises_type_error_with_slots(self):
        """in: register('fake_i') on a RunKind subclass that fills no slot
        exp: TypeError whose text lists 'matlab_setup', 'boot', 'go', 'fly', 'stop', 'score', 'conditions', 'fields'
        bug: a half-written kind registers and fails in the middle of a session."""
        class Bad(RunKind):
            pass
        with self.assertRaises(TypeError) as cm:
            factory.register("fake_i")(Bad)
        for slot in ("matlab_setup", "boot", "go", "fly", "stop", "score", "conditions", "fields"):
            self.assertIn(slot, str(cm.exception))
        self.assertNotIn("fake_i", factory._REGISTRY)

    def test_partly_complete_class_lists_only_missing_slots(self):
        """in: a subclass of FakeKind that is abstract again because 'score' is declared abstract
        exp: TypeError that names 'score' and not 'boot'
        bug: the message lists every slot, not the missing ones."""
        import abc

        class Part(FakeKind):
            @abc.abstractmethod
            def score(self, trial):
                ...
        with self.assertRaises(TypeError) as cm:
            factory.register("fake_p")(Part)
        self.assertIn("score", str(cm.exception))
        self.assertNotIn("boot", str(cm.exception))

    def test_non_runkind_raises_type_error(self):
        """in: register('fake_n') on a plain class
        exp: TypeError
        bug: any class can enter the registry."""
        class Plain:
            pass
        with self.assertRaises(TypeError):
            factory.register("fake_n")(Plain)

    def test_unknown_name_raises_key_error_listing_names(self):
        """in: kind_class('zzz') after register('fake_k')
        exp: KeyError whose text contains 'zzz' and 'fake_k'
        bug: the operator sees 'KeyError: zzz' and does not know the valid names."""
        class K(FakeKind):
            pass
        factory.register("fake_k")(K)
        with self.assertRaises(KeyError) as cm:
            factory.kind_class("zzz")
        self.assertIn("zzz", str(cm.exception))
        self.assertIn("fake_k", str(cm.exception))

    def test_kinds_available_has_controller_and_slam_once(self):
        """in: kinds_available() with the real kinds present
        exp: a sorted list holding 'controller' once and 'slam' once
        bug: a kind module is not imported by load_all, or it registers twice."""
        names = factory.kinds_available()
        self.assertEqual(names, sorted(names))
        self.assertEqual(names.count("controller"), 1)
        self.assertEqual(names.count("slam"), 1)

    def test_make_kind_builds_instance(self):
        """in: make_kind('fake_m', settings, services)
        exp: an instance of the registered class, with .settings and .services set and .cfg == kind_cfg
        bug: make_kind does not pass settings and services to the constructor."""
        class K(FakeKind):
            pass
        factory.register("fake_m")(K)
        with tempfile.TemporaryDirectory() as tmp:
            st, sv = make_settings(kind_cfg={"count": 2}), make_services(tmp)
            k = factory.make_kind("fake_m", st, sv)
        self.assertIsInstance(k, K)
        self.assertIs(k.settings, st)
        self.assertIs(k.services, sv)
        self.assertEqual(k.cfg, {"count": 2})

    def test_main_wires_load_session_run(self):
        """in: main('fake_w', ['--tag','t']) with config.load and Session replaced by mocks (Session.run returns 7)
        exp: returns 7; load got (argv, class); Session got (settings, class)
        bug: main drops the exit code, or builds the Session before the settings are loaded."""
        class K(FakeKind):
            pass
        factory.register("fake_w")(K)
        st = make_settings()
        with mock.patch("hilbench.config.load", return_value=st) as load, \
                mock.patch("hilbench.session.Session") as sess:
            sess.return_value.run.return_value = 7
            code = factory.main("fake_w", ["--tag", "t"])
        self.assertEqual(code, 7)
        load.assert_called_once()
        self.assertEqual(load.call_args.args[0], ["--tag", "t"])
        self.assertIs(load.call_args.args[1], K)
        sess.assert_called_once_with(st, K)


if __name__ == "__main__":
    unittest.main()
