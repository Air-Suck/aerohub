#!/usr/bin/env bash
set -euo pipefail
root=$(cd "$(dirname "$0")/../../../.." && pwd)
exec bash "$root/tools/robopi/build_lib.sh" catora
