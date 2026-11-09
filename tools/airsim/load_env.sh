# Source from repository scripts. .env contains trusted local shell assignments.
aerohub_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
if [[ -f "$aerohub_root/.env" ]]; then
  set -a
  source "$aerohub_root/.env"
  set +a
fi
