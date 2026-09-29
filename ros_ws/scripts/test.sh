#!/usr/bin/env bash
# Build, then run every test (gtests, pytest, launch_testing) and print the results.
#   scripts/test.sh [extra colcon test args...]   e.g. --packages-select spine_link
# Exits non-zero if the build fails or any test fails.
set -euo pipefail
HERE="$(dirname "${BASH_SOURCE[0]}")"
"${HERE}/build.sh"
source "${HERE}/env.sh"
cd "${WS}"

colcon test --return-code-on-test-failure --event-handlers console_cohesion+ "$@" || status=$?
colcon test-result --verbose
exit "${status:-0}"
