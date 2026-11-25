#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -x "$project_dir/build/g1_deploy" ]]; then
  printf '%s\n' 'Run ./build_cpp.sh first.' >&2
  exit 1
fi
exec "$project_dir/build/g1_deploy" --root "$project_dir" "$@"
