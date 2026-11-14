"""Terminate a runtime session, including children left by a ros2 run wrapper."""
import os
import signal
import subprocess


def terminate_session(process, timeout=2.0):
    if process is None:
        return
    if process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait(timeout=timeout)
    # Wrapper exit does not imply its entire session exited. Do this immediately
    # after reaping it, before a resumed task can receive an old adapter's output.
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
