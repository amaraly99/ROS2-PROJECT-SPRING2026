"""ui.spark and summary.append_runs_csv (the small pure functions)."""
import tempfile
import unittest
from pathlib import Path

from hilbench import summary, ui


class Spark(unittest.TestCase):
    def test_empty(self):
        """in: spark([])
        exp: ''
        bug: an empty history raises ValueError from min()."""
        self.assertEqual(ui.spark([]), "")

    def test_one_char_per_value_and_extremes(self):
        """in: spark([0, 5, 10])
        exp: 3 characters; the first differs from the last; they are the lowest and the highest block
        bug: the scale is inverted or the highest value is out of range."""
        s = ui.spark([0, 5, 10])
        self.assertEqual(len(s), 3)
        self.assertEqual(s[0], "▁")
        self.assertEqual(s[-1], "█")

    def test_constant_values(self):
        """in: spark([3, 3, 3])
        exp: 3 characters, no ZeroDivisionError
        bug: a flat line divides by a zero span."""
        self.assertEqual(len(ui.spark([3, 3, 3])), 3)

    def test_fixed_scale(self):
        """in: spark([5], lo=0, hi=10)
        exp: a middle block (not the lowest, not the highest)
        bug: lo and hi are ignored."""
        c = ui.spark([5], lo=0, hi=10)
        self.assertNotIn(c, ("▁", "█"))


class RunsCsv(unittest.TestCase):
    def test_append_rows(self):
        """in: append_runs_csv(path, ['1','off','True']) then (path, ['2','on','False'])
        exp: the file has the two lines '1,off,True' and '2,on,False'
        bug: the second call overwrites the first, so a crash loses the finished runs."""
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / "runs.csv"
            summary.append_runs_csv(p, ["1", "off", "True"])
            summary.append_runs_csv(p, ["2", "on", "False"])
            self.assertEqual(p.read_text().splitlines(), ["1,off,True", "2,on,False"])

    def test_comma_in_cell_is_quoted(self):
        """in: a row with the cell 'a, b'
        exp: the line is '1,"a, b"'
        bug: a comma in a reason text shifts the columns."""
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / "runs.csv"
            summary.append_runs_csv(p, ["1", "a, b"])
            self.assertEqual(p.read_text().strip(), '1,"a, b"')


if __name__ == "__main__":
    unittest.main()
