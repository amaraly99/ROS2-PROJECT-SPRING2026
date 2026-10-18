"""session.py: the schedule (round robin over the conditions of a kind)."""
import tempfile
import types
import unittest

from hilbench.session import Session

from ._fakes import FakeKind, make_services, make_settings


def schedule_for(conds, runs):
    """Session.schedule on a minimal stand-in object: only .settings and .kind are used."""
    with tempfile.TemporaryDirectory() as tmp:
        kind = FakeKind(make_settings(runs=runs), make_services(tmp), conds=conds)
    return Session.schedule(types.SimpleNamespace(settings=make_settings(runs=runs), kind=kind))


class ScheduleTests(unittest.TestCase):
    def test_round_robin_three_conditions(self):
        """in: conditions [A,B,C], runs 2
        exp: [(1,'A'),(2,'B'),(3,'C'),(4,'A'),(5,'B'),(6,'C')]
        bug: the schedule runs all of A first (AABBCC) and a slow drift hits only one condition."""
        self.assertEqual(schedule_for(["A", "B", "C"], 2),
                         [(1, "A"), (2, "B"), (3, "C"), (4, "A"), (5, "B"), (6, "C")])

    def test_one_condition_is_a_plain_loop(self):
        """in: conditions ['A'], runs 3
        exp: [(1,'A'),(2,'A'),(3,'A')]
        bug: a single condition produces one run, or numbering starts at 0."""
        self.assertEqual(schedule_for(["A"], 3), [(1, "A"), (2, "A"), (3, "A")])

    def test_two_conditions_alternate(self):
        """in: conditions ['off','on'], runs 2
        exp: [(1,'off'),(2,'on'),(3,'off'),(4,'on')]
        bug: camera ab no longer alternates OFF, ON, OFF, ON."""
        self.assertEqual(schedule_for(["off", "on"], 2), [(1, "off"), (2, "on"), (3, "off"), (4, "on")])

    def test_length_is_runs_times_conditions(self):
        """in: 4 conditions, runs 5
        exp: 20 entries, numbered 1..20
        bug: runs is counted for the whole session instead of per condition."""
        sch = schedule_for(["a", "b", "c", "d"], 5)
        self.assertEqual([k for k, _ in sch], list(range(1, 21)))

    def test_zero_runs_is_empty(self):
        """in: runs 0
        exp: []
        bug: a zero-run session still flies one trial."""
        self.assertEqual(schedule_for(["A", "B"], 0), [])


if __name__ == "__main__":
    unittest.main()
