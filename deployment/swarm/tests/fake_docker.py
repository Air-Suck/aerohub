#!/usr/bin/env python3
"""Controllable stand-in for the Docker CLI; containers are local processes tracked in FAKE_DOCKER_STATE."""
import json
import os
from pathlib import Path
import signal
import sys
import time


STATE = Path(os.environ["FAKE_DOCKER_STATE"])


def pid_file(name):
    return STATE / f"{name}.pid"


def alive(name):
    try:
        pid = int(pid_file(name).read_text())
        os.kill(pid, 0)
        return pid
    except (OSError, ValueError):
        return None


def append(event, **fields):
    with (STATE / "events.jsonl").open("a") as stream:
        stream.write(json.dumps(dict(event=event, **fields)) + "\n")


def behavior():
    path = STATE / "behavior.json"
    return json.loads(path.read_text()) if path.exists() else {}


def run(arguments):
    name = arguments[arguments.index("--name") + 1]
    exit_after, exit_code = behavior().get("exit", {}).get(name, [None, 0])
    if alive(name):
        print(f"Conflict. The container name {name} is already in use", file=sys.stderr)
        return 125
    stopping = []
    signal.signal(signal.SIGTERM, lambda *_: stopping.append(True))
    pid_file(name).write_text(str(os.getpid()))
    append("run", name=name, pid=os.getpid())
    print(f"started {name}", flush=True)
    started = time.monotonic()
    code = 0
    while not stopping:
        if exit_after is not None and time.monotonic() - started >= exit_after:
            code = exit_code
            break
        time.sleep(0.02)
    print(f"stopped {name} code={code}", flush=True)
    pid_file(name).unlink(missing_ok=True)
    append("exit", name=name, code=code)
    return code


def stop(name, timeout):
    pid = alive(name)
    if pid is None:
        print(f"No such container: {name}", file=sys.stderr)
        return 1
    if name in behavior().get("ignore_stop", []):
        append("stop_ignored", name=name)
        return 0
    os.kill(pid, signal.SIGTERM)
    deadline = time.monotonic() + timeout
    while alive(name) and time.monotonic() < deadline:
        time.sleep(0.02)
    if alive(name):
        os.kill(pid, signal.SIGKILL)
        pid_file(name).unlink(missing_ok=True)
    append("stop", name=name)
    print(name)
    return 0


def remove(name):
    pid = alive(name)
    if pid is not None:
        os.kill(pid, signal.SIGKILL)
    pid_file(name).unlink(missing_ok=True)
    append("rm", name=name)
    return 0


def main(arguments):
    command = arguments[0]
    if command == "run":
        return run(arguments)
    if command == "inspect":
        name = arguments[-1]
        if not alive(name):
            print(f"Error: No such object: {name}", file=sys.stderr)
            return 1
        print("true" if "--format" in arguments else json.dumps([{"Name": name}]))
        return 0
    if command == "stop":
        return stop(arguments[-1], float(arguments[arguments.index("--timeout") + 1]))
    if command == "rm":
        return remove(arguments[-1])
    print(f"fake docker does not implement {command}", file=sys.stderr)
    return 2


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
