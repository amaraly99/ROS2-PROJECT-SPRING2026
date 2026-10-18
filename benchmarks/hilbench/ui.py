"""ui.py — the rich terminal: what is shown, and the three keys p / a / l.

It knows nothing about an experiment: a plug-in gives it the table columns and the flight rows.
"""
import select
import sys
import termios
import threading
import tty

from rich.console import Group
from rich.markup import escape
from rich.panel import Panel
from rich.progress_bar import ProgressBar
from rich.prompt import Prompt
from rich.table import Table
from rich.text import Text

SPARK = "▁▂▃▄▅▆▇█"


def spark(values, lo=None, hi=None):
    if not values:
        return ""
    lo = min(values) if lo is None else lo
    hi = max(values) if hi is None else hi
    span = (hi - lo) or 1.0
    return "".join(SPARK[min(7, max(0, int((v - lo) / span * 7.999)))] for v in values)


class Keys(threading.Thread):
    """p = pause after this run, a = abort, l = show / hide the bench log."""

    def __init__(self):
        super().__init__(daemon=True)
        self.pause = False
        self.abort = False
        self.show_log = False
        self._fd = sys.stdin.fileno() if sys.stdin.isatty() else None
        self._old = termios.tcgetattr(self._fd) if self._fd is not None else None
        self._stop_evt = threading.Event()

    def run(self):
        if self._fd is None:
            return
        tty.setcbreak(self._fd)
        while not self._stop_evt.is_set():
            r, _, _ = select.select([sys.stdin], [], [], 0.2)
            if r:
                ch = sys.stdin.read(1).lower()
                if ch == "p":
                    self.pause = not self.pause
                elif ch == "a":
                    self.abort = True
                elif ch == "l":
                    self.show_log = not self.show_log

    def restore(self):
        self._stop_evt.set()
        if self._old is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)

    def suspend(self):
        if self._old is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)

    def resume_cbreak(self):
        if self._fd is not None:
            tty.setcbreak(self._fd)


class Screen:
    """Everything on the screen. Use it as the renderable of a rich Live: Live(get_renderable=screen.render)."""

    def __init__(self, header_text):
        self.header_text = header_text      # rich markup
        self.links = {}                     # name -> (ok, text)
        self.banner = ""
        self.steps = []                     # (name, status, detail)
        self.run_title = ""
        self.flight = None                  # dict, set by the plug-in
        self.flight_rows = lambda flight: []   # flight -> list of (left, right) text pairs
        self.columns = ()
        self.rows = []                      # finished runs: one list of cells each
        self.log_lines = lambda: None       # -> list of lines | None
        self.keys = None
        self.on_step = None                 # called as on_step(name, status, detail)

    def set_link(self, name, ok, text):
        self.links[name] = (ok, text)

    def step(self, name, status, detail=""):
        for i, (n, _, _) in enumerate(self.steps):
            if n == name:
                self.steps[i] = (name, status, detail)
                break
        else:
            self.steps.append((name, status, detail))
        if self.on_step:
            self.on_step(name, status, detail)

    def render(self):
        lk = Table.grid(padding=(0, 2))
        for name, (ok, txt) in self.links.items():
            mark = {True: "[green]✔[/]", False: "[red]✘[/]", None: "[yellow]…[/]"}[ok]
            lk.add_row(mark, f"[bold]{escape(name)}[/]", escape(str(txt)))
        st = Table.grid(padding=(0, 1))
        for name, status, detail in self.steps:
            mark = {"done": "[green]✔[/]", "run": "[cyan]▶[/]", "fail": "[red]✘[/]", "wait": "[dim]·[/]"}[status]
            st.add_row(mark, escape(name), f"[dim]{escape(str(detail))}[/]")
        parts = [Text.from_markup(self.header_text), Panel(lk, title="links", border_style="dim")]
        if self.banner:
            parts.append(Panel(Text(self.banner, style="bold yellow"), border_style="yellow"))
        if self.steps:
            parts.append(Panel(st, title=self.run_title, border_style="cyan"))
        if self.flight:
            f = self.flight
            bar = ProgressBar(total=f["total"], completed=min(f["elapsed"], f["total"]), width=48)
            ft = Table.grid(padding=(0, 2))
            ft.add_row(bar, f"{f['elapsed']:5.0f} / {f['total']:.0f} s")
            for left, right in self.flight_rows(f):
                ft.add_row(left, right)
            parts.append(Panel(ft, title="flight", border_style="green"))
        rt = Table(show_edge=False, header_style="bold")
        for c in self.columns:
            rt.add_column(c)
        for r in self.rows:
            rt.add_row(*r)
        parts.append(Panel(rt, title="runs", border_style="dim"))
        keys = self.keys
        if keys is not None and keys.show_log:
            lines = self.log_lines()
            if lines:
                parts.append(Panel(Text("\n".join(list(lines)[-12:]), style="dim"), title="bench log (l to hide)", border_style="dim"))
        flags = []
        if keys is not None and keys.pause:
            flags.append("[yellow]PAUSE after this run[/]")
        if keys is not None and keys.abort:
            flags.append("[red]ABORT requested[/]")
        parts.append(Text.from_markup("  ".join(flags) + "   [dim]p pause after this run · a abort · l bench log[/]"))
        return Group(*parts)


def ask_next_or_abort(live, keys, k, reason):
    """The run is not valid: stop the live view and ask. Returns 'n' (next run) or 'a' (abort)."""
    live.stop()
    keys.suspend()
    ans = Prompt.ask(f"[red]Run {k} is not valid[/] ({escape(str(reason))}). n = next run, a = abort",
                     choices=["n", "a"], default="n")
    keys.resume_cbreak()
    live.start()
    return ans


def wait_for_enter(live, keys):
    """The operator pressed p: wait for Enter before the next run."""
    live.stop()
    keys.suspend()
    Prompt.ask("[yellow]Paused.[/] Press Enter for the next run", default="")
    keys.pause = False
    keys.resume_cbreak()
    live.start()
