#!/usr/bin/env python3
"""Exercise the actual shell supervisor with owned local child processes."""
import subprocess
import tempfile
from pathlib import Path
import unittest

TOOLS = Path(__file__).resolve().parent


class ProcessControlTests(unittest.TestCase):
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
