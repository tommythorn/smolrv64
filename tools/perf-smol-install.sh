#!/usr/bin/env bash
# Install the board's perf tools from this tree into the boards' shared home, so on either board
# `perf-smol top CMD` runs and reports with the event table that matches the scripts:
#   ~/bin/perf-smol (tools/perf-smol.sh), ~/bin/perf-cpi-stack.py, ~/bin/perf-smol-rec.py,
#   ~/bin/smolrv64-perf-events.json
# The four travel together; perf-cpi-stack.py refuses a table older than itself.
#   tools/perf-smol-install.sh [dir]     # default /srv/ubuntu-home/bin: coffee's export of the boards' /home/tommy
# The boards' ~/.profile puts ~/bin on PATH when it exists (a new login, or `. ~/.profile`).
set -eu
cd "$(dirname "$0")/.."
D=${1:-/srv/ubuntu-home/bin}
mkdir -p "$D"
install -m 755 tools/perf-smol.sh "$D/perf-smol"
install -m 755 tools/perf-cpi-stack.py "$D/perf-cpi-stack.py"
install -m 755 tools/perf-smol-rec.py "$D/perf-smol-rec.py"
install -m 644 docs/smolrv64-perf-events.json "$D/smolrv64-perf-events.json"
echo "perf-smol: installed in $D from $(git rev-parse --short HEAD)$(git diff --quiet HEAD -- tools docs || echo '-dirty')"
