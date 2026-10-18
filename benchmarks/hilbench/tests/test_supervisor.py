"""supervisor.py against a fake TCP server on 127.0.0.1."""
import unittest

from hilbench.supervisor import Supervisor, SupervisorError, parse_status
from hilbench.trial import RigFailed

from ._fakes import FakeSupervisorServer, closed_port

STATUS = "status sim=running t=1.25 state=held pub=on timers=3 hz=16.1 yaw=357.36 x=15.12 y=3.84 z=7.5"
MODEL_INFO = ("model_info model=hil_closed_loop_unified x_ic=15.12 y_ic=3.84 z_ic=7.5 yaw_ic=6.2372 "
              "fx_left=554 fx_right=none right_cam=off right_y=none depth_port=off left_image=rgb right_image=rgb")


def make_sup(server, timeout_s=2.0):
    return Supervisor("127.0.0.1", server.port, timeout_s=timeout_s)


class ParseStatus(unittest.TestCase):
    def test_numbers_become_floats_words_stay_text(self):
        """in: parse_status of 'status sim=running t=1.25 state=held yaw=357.36 x=15.12 hz=16.1'
        exp: t 1.25 (float), yaw 357.36, x 15.12, state 'held', sim 'running'
        bug: numbers stay as text and the start check compares text with a number."""
        d = parse_status(STATUS)
        self.assertEqual((d["t"], d["yaw"], d["x"], d["hz"]), (1.25, 357.36, 15.12, 16.1))
        self.assertIsInstance(d["t"], float)
        self.assertEqual((d["state"], d["sim"]), ("held", "running"))

    def test_question_mark_value_stays_text(self):
        """in: parse_status of 'status sim=? t=0 state=idle'
        exp: sim == '?' (text), t == 0.0
        bug: '?' raises ValueError."""
        d = parse_status("status sim=? t=0 state=idle")
        self.assertEqual((d["sim"], d["t"]), ("?", 0.0))


class SendAndErrors(unittest.TestCase):
    def test_send_returns_reply_line(self):
        """in: send('resume') to a server that answers 'ok'
        exp: 'ok'; the server received exactly 'resume'
        bug: the command is sent without a newline, or the reply keeps the newline."""
        with FakeSupervisorServer(lambda c: "ok") as srv:
            self.assertEqual(make_sup(srv).send("resume"), "ok")
            self.assertEqual(srv.received, ["resume"])

    def test_error_reply_raises(self):
        """in: reply 'error busy'
        exp: SupervisorError whose text holds 'busy'
        bug: an error reply is returned as normal text and the run goes on."""
        with FakeSupervisorServer(lambda c: "error busy") as srv:
            with self.assertRaises(SupervisorError) as cm:
                make_sup(srv).send("start_hold")
        self.assertIn("busy", str(cm.exception))

    def test_empty_reply_raises(self):
        """in: the server closes the connection without a reply
        exp: SupervisorError
        bug: an empty answer is treated as ok."""
        with FakeSupervisorServer(lambda c: None) as srv:
            with self.assertRaises(SupervisorError):
                make_sup(srv).send("status")

    def test_no_server_raises(self):
        """in: nothing listens on the port
        exp: SupervisorError (not OSError)
        bug: a refused connection crashes the runner with a raw socket error."""
        with self.assertRaises(SupervisorError):
            Supervisor("127.0.0.1", closed_port(), timeout_s=0.5).send("status")


class StatusAndCommands(unittest.TestCase):
    def test_status_parses_and_counts_calls(self):
        """in: status() twice
        exp: dict with state 'held', x 15.12; status_calls 0 -> 2
        bug: the overhead meter reads a counter that never grows."""
        with FakeSupervisorServer(lambda c: STATUS) as srv:
            sup = make_sup(srv)
            self.assertEqual(sup.status_calls, 0)
            d = sup.status()
            sup.status()
            self.assertEqual(sup.status_calls, 2)
        self.assertEqual((d["state"], d["x"]), ("held", 15.12))

    def test_send_does_not_count_as_status_call(self):
        """in: send('stop') then send('resume')
        exp: status_calls stays 0
        bug: every command inflates the runner_status_calls figure in meta.txt."""
        with FakeSupervisorServer(lambda c: "ok") as srv:
            sup = make_sup(srv)
            sup.send("stop")
            sup.send("resume")
            self.assertEqual(sup.status_calls, 0)

    def test_model_info_parse(self):
        """in: reply to 'model_info' (fx_right=none, right_cam=off, fx_left=554)
        exp: model 'hil_closed_loop_unified', x_ic 15.12, y_ic 3.84, z_ic 7.5, fx_left 554.0, fx_right 'none',
             right_cam 'off', depth_port 'off', left_image 'rgb'; server received 'model_info'
        bug: model_info is parsed with the 'status' prefix rule and fails, or fx_right 'none' is turned into a number."""
        with FakeSupervisorServer(lambda c: MODEL_INFO) as srv:
            d = make_sup(srv).model_info()
            self.assertEqual(srv.received, ["model_info"])
        self.assertEqual(d["model"], "hil_closed_loop_unified")
        self.assertEqual((d["x_ic"], d["y_ic"], d["z_ic"], d["fx_left"]), (15.12, 3.84, 7.5, 554.0))
        self.assertEqual((d["fx_right"], d["right_cam"], d["depth_port"], d["left_image"]),
                         ("none", "off", "off", "rgb"))

    def test_pub_stop_stop_resume_commands(self):
        """in: pub_off(), stop(), resume()
        exp: the server receives 'pub_stop', 'stop', 'resume'; resume returns two ints (sent_ns <= ok_ns)
        bug: wrong command text, or resume loses the two time stamps that cross-check t = 0."""
        with FakeSupervisorServer(lambda c: "ok") as srv:
            sup = make_sup(srv)
            sup.pub_off()
            sup.stop()
            sent, ok = sup.resume()
            self.assertEqual(srv.received, ["pub_stop", "stop", "resume"])
        self.assertIsInstance(sent, int)
        self.assertLessEqual(sent, ok)

    def test_pub_on_command_has_rate(self):
        """in: pub_on(20.0)
        exp: server receives 'pub_start' followed by the number 20
        bug: the camera rate is dropped and MATLAB starts at its own default."""
        with FakeSupervisorServer(lambda c: "ok") as srv:
            make_sup(srv).pub_on(20.0)
            parts = srv.received[0].split()
        self.assertEqual(parts[0], "pub_start")
        self.assertEqual(float(parts[1]), 20.0)


class Hold(unittest.TestCase):
    @staticmethod
    def handler(states):
        """start_hold -> ok; every status call returns the next state of the list (the last one repeats)."""
        seq = list(states)

        def h(cmd):
            if cmd == "start_hold":
                return "ok"
            st = seq.pop(0) if len(seq) > 1 else seq[0]
            return f"status sim=running t=0.25 state={st} x=15.12 y=3.84 z=7.5 yaw=357.36"
        return h

    def test_hold_returns_status_when_held(self):
        """in: states running, running, held
        exp: returns the status dict with state 'held' and x 15.12; the server saw 'start_hold' first
        bug: hold returns before MATLAB is held and the drone is flown from a moving start."""
        with FakeSupervisorServer(self.handler(["running", "running", "held"])) as srv:
            d = make_sup(srv).hold(5.0, poll_s=0.05)
            self.assertEqual(srv.received[0], "start_hold")
        self.assertEqual((d["state"], d["x"]), ("held", 15.12))

    def test_hold_frozen_raises_rig_failed(self):
        """in: states running, frozen
        exp: RigFailed with step 'start_hold'
        bug: a frozen simulation is waited on until the timeout."""
        with FakeSupervisorServer(self.handler(["running", "frozen"])) as srv:
            with self.assertRaises(RigFailed) as cm:
                make_sup(srv).hold(5.0, poll_s=0.05)
        self.assertEqual(cm.exception.step, "start_hold")

    def test_hold_timeout_raises_rig_failed(self):
        """in: state stays 'running'; timeout 0.4 s
        exp: RigFailed with step 'start_hold' after about 0.4 s
        bug: hold waits forever."""
        import time
        with FakeSupervisorServer(self.handler(["running"])) as srv:
            t0 = time.monotonic()
            with self.assertRaises(RigFailed) as cm:
                make_sup(srv).hold(0.4, poll_s=0.05)
            self.assertLess(time.monotonic() - t0, 3.0)
        self.assertEqual(cm.exception.step, "start_hold")

    def test_hold_start_hold_error_raises(self):
        """in: MATLAB answers 'error busy' to start_hold
        exp: RigFailed or SupervisorError (not a normal return)
        bug: the error is swallowed and hold polls a simulation that never starts."""
        with FakeSupervisorServer(lambda c: "error busy") as srv:
            with self.assertRaises((RigFailed, SupervisorError)):
                make_sup(srv).hold(0.5, poll_s=0.05)


class WaitMatlab(unittest.TestCase):
    def test_returns_true_on_first_answer(self):
        """in: a server that answers status
        exp: True; on_tick not needed
        bug: wait_matlab waits poll_s before the first try."""
        with FakeSupervisorServer(lambda c: STATUS) as srv:
            self.assertTrue(make_sup(srv).wait_matlab(2.0, poll_s=0.05))

    def test_returns_false_and_ticks_when_no_answer(self):
        """in: no server; wait 0.4 s, poll 0.1 s, on_tick collects values
        exp: False; on_tick called at least once with a number <= 0.4
        bug: the screen never shows the 'paste hil_session_start' countdown."""
        ticks = []
        sup = Supervisor("127.0.0.1", closed_port(), timeout_s=0.2)
        self.assertFalse(sup.wait_matlab(0.4, on_tick=ticks.append, poll_s=0.1))
        self.assertTrue(ticks)
        self.assertLessEqual(max(ticks), 0.4 + 1e-6)


if __name__ == "__main__":
    unittest.main()
