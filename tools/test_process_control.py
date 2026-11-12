#!/usr/bin/env python3
"""Exercise the actual shell supervisor with owned local child processes."""
import subprocess
import tempfile
from pathlib import Path
import unittest

TOOLS = Path(__file__).resolve().parent


class ProcessControlTests(unittest.TestCase):
    def test_exited_orphan_is_not_treated_as_a_live_process_group(self):
        self.check_orphan(False, 0)

    def test_live_orphan_still_requires_termination(self):
        self.check_orphan(True, 124)

    def check_orphan(self, live, expected):
        # A private subreaper simulates container PID 1; it always reaps the
        # fixture, including if the supervisor accidentally misses a live child.
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            pidfile = folder/'pid'
            child = folder/'orphan.py'
            child.write_text("import os,time,signal,sys\nfrom pathlib import Path\n"
                "signal.signal(signal.SIGINT,signal.SIG_IGN)\n"
                "signal.signal(signal.SIGTERM,signal.SIG_IGN)\n"
                "pid=os.fork()\n"
                + ("if pid==0: time.sleep(60); os._exit(0)\n" if live else "if pid==0: os._exit(0)\n")
                + "Path(sys.argv[1]).write_text(str(pid))\n"
                + ("time.sleep(.05)\n" if live else
                   "while Path('/proc/%d/stat'%pid).read_text().rpartition(') ')[2].split()[0]!='Z': time.sleep(.01)\n"))
            supervisor = folder/'supervisor.py'
            supervisor.write_text("import ctypes,os,signal,subprocess,sys\nfrom pathlib import Path\n"
                "assert ctypes.CDLL(None).prctl(36,1,0,0,0)==0\n"
                "try:\n p=subprocess.run(sys.argv[2:],timeout=8)\n"
                "finally:\n"
                " if Path(sys.argv[1]).exists():\n"
                "  try: os.kill(int(Path(sys.argv[1]).read_text()),signal.SIGKILL)\n"
                "  except ProcessLookupError: pass\n"
                " while True:\n"
                "  try: os.waitpid(-1,0)\n"
                "  except ChildProcessError: break\n"
                "sys.exit(p.returncode)\n")
            script = '''source "$1"
python3 "$2" python3 "$3" "$4" & pid=$!
wait "$pid"
stop_process "$pid" 0 0
exit $?
'''
            result = subprocess.run(['python3', str(supervisor), str(pidfile), 'bash', '-c', script,
                'test', str(TOOLS/'process_control.sh'), str(TOOLS/'launch_process.py'), str(child), str(pidfile)],
                capture_output=True, text=True, timeout=12)
            self.assertEqual(result.returncode, expected, result.stderr)

    def test_shutdown_reaps_graceful_child(self):
        self.check_shutdown(False, 0)

    def test_shutdown_escalates_without_waiting_forever(self):
        self.check_shutdown(True, 124)

    def check_shutdown(self, ignore, expected):
        with tempfile.TemporaryDirectory() as folder:
            child = Path(folder, 'child.py')
            child.write_text('import signal,time,sys\nfrom pathlib import Path\n'
                + ('signal.signal(signal.SIGINT,signal.SIG_IGN)\nsignal.signal(signal.SIGTERM,signal.SIG_IGN)\n'
                   if ignore else 'signal.signal(signal.SIGINT,lambda *_: sys.exit(0))\n')
                + 'Path(sys.argv[1]).touch()\ntime.sleep(60)\n')
            script = '''source "$1"
python3 "$2" python3 "$3" "$4" &
pid=$!
trap 'stop_process "$pid" 0 0 >/dev/null 2>&1 || true' EXIT
for ((i=0;i<200;i++)); do [[ ! -e "$4" ]] || break; sleep .01; done
stop_process "$pid" 1 1
exit $?
'''
            result = subprocess.run(['bash', '-c', script, 'test', str(TOOLS/'process_control.sh'),
                str(TOOLS/'launch_process.py'), str(child), str(Path(folder,'ready'))],
                text=True, capture_output=True, timeout=8)
            self.assertEqual(result.returncode, expected, result.stderr)

    def test_live_driver_failure_is_propagated(self):
        self.check_live(42, 42)

    def test_live_normal_driver_exit_is_unexpected(self):
        self.check_live(0, 1)

    def check_live(self, driver_code, expected):
        script = '''source "$1"
python3 "$2" python3 -c 'import time;time.sleep(60)' & mapper=$!
python3 "$2" python3 -c 'import time,sys;time.sleep(.1);sys.exit(int(sys.argv[1]))' "$3" & driver=$!
trap 'stop_process "$mapper" 0 0 >/dev/null 2>&1 || true; stop_process "$driver" 0 0 >/dev/null 2>&1 || true' EXIT
wait_live "$mapper" "$driver"
exit $?
'''
        result = subprocess.run(['bash','-c',script,'test',str(TOOLS/'process_control.sh'),
            str(TOOLS/'launch_process.py'),str(driver_code)],capture_output=True,text=True,timeout=8)
        self.assertEqual(result.returncode, expected, result.stderr)
        self.assertIn('Live driver stopped', result.stderr)


if __name__ == '__main__':
    unittest.main()
