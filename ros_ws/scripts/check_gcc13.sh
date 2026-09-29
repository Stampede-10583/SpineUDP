#!/usr/bin/env bash
# Compile-checks every spine_link source with GCC 13, the compiler on the Orin (Ubuntu 24.04),
# using the exact flags from the normal build. Syntax/semantics only (-fsyntax-only): on Ubuntu
# 26.04 the ROS libraries are built with GCC 15, so GCC 13 can't link against them.
# Run scripts/build.sh first.
set -euo pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CXX13="${CXX13:-g++-13}"
DB="${WS}/build/spine_link/compile_commands.json"
command -v "${CXX13}" >/dev/null || { echo "error: ${CXX13} not installed" >&2; exit 1; }
[ -f "${DB}" ] || { echo "error: ${DB} missing; run scripts/build.sh first" >&2; exit 1; }

python3 - "${DB}" "${CXX13}" <<'PY'
import json, shlex, subprocess, sys
db, cxx = sys.argv[1], sys.argv[2]
failed = 0
entries = [e for e in json.load(open(db)) if '/rclcpp_components/' not in e['file']]
for e in entries:
    args = shlex.split(e['command']) if 'command' in e else list(e['arguments'])
    # Drop the compiler, output and -c; keep defines, includes and flags.
    out, skip = [], False
    for a in args[1:]:
        if skip:
            skip = False
            continue
        if a == '-o':
            skip = True
            continue
        if a in ('-c',) or a == e['file']:
            continue
        out.append(a)
    r = subprocess.run([cxx, '-fsyntax-only', *out, e['file']], cwd=e['directory'])
    print(('ok   ' if r.returncode == 0 else 'FAIL ') + e['file'].split('/spine_link/')[-1])
    failed += r.returncode != 0
print(f'{len(entries) - failed}/{len(entries)} sources compile with {cxx}')
sys.exit(1 if failed else 0)
PY
