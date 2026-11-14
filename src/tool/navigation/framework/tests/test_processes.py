import os
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path
from uav_planner_framework.processes import terminate_session


class RuntimeCleanupTest(unittest.TestCase):
    def test_wrapper_exit_does_not_leave_old_planner_running(self):
        with tempfile.TemporaryDirectory() as directory:
            child_file=Path(directory)/'child'
            child_code="import os,signal,time,pathlib; signal.signal(signal.SIGTERM,signal.SIG_IGN); pathlib.Path("+repr(str(child_file))+").write_text(str(os.getpid())); time.sleep(30)"
            wrapper_code="import subprocess,sys,time; subprocess.Popen([sys.executable,'-c',"+repr(child_code)+"]); time.sleep(30)"
            wrapper=subprocess.Popen([sys.executable,'-c',wrapper_code],start_new_session=True)
            child_pid=None
            try:
                deadline=time.monotonic()+5
                while time.monotonic()<deadline:
                    if child_file.exists() and child_file.read_text().isdigit():break
                    time.sleep(.01)
                self.assertTrue(child_file.exists() and child_file.read_text().isdigit())
                child_pid=int(child_file.read_text())
                # The wrapper exits first; its child still ignores TERM in that session.
                wrapper.terminate();wrapper.wait(timeout=2)
                terminate_session(wrapper,.2)
                deadline=time.monotonic()+3
                while time.monotonic()<deadline:
                    stat=Path('/proc')/str(child_pid)/'stat'
                    try:
                        if stat.read_text().rsplit(')',1)[1].split()[0]=='Z':break
                    except (FileNotFoundError,ProcessLookupError):break
                    time.sleep(.01)
                else:self.fail('old planner child survived wrapper cleanup')
            finally:
                try:os.killpg(wrapper.pid,signal.SIGKILL)
                except ProcessLookupError:pass
                wrapper.wait(timeout=2)
