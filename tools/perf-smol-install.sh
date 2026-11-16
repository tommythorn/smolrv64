#!/usr/bin/env bash
# Install the board's perf tools from this tree into a directory on the board's PATH, so
# `perf-smol top CMD` runs and reports with the event table that matches the scripts:
#   perf-smol (tools/perf-smol.sh), perf-cpi-stack.py, perf-smol-rec.py, smolrv64-perf-events.json
# The four travel together; perf-cpi-stack.py refuses a table older than itself.
#   tools/perf-smol-install.sh [dir]     # default ~/bin: run on the board, or name the NFS
#                                        # server's copy of the board's ~/bin
# Ubuntu's ~/.profile puts ~/bin on PATH when it exists (a new login, or `. ~/.profile`).
set -eu
cd "$(dirname "$0")/.."
D=${1:-$HOME/bin}
mkdir -p "$D"
install -m 755 tools/perf-smol.sh "$D/perf-smol"
install -m 755 tools/perf-cpi-stack.py "$D/perf-cpi-stack.py"
install -m 755 tools/perf-smol-rec.py "$D/perf-smol-rec.py"
install -m 644 docs/smolrv64-perf-events.json "$D/smolrv64-perf-events.json"
echo "perf-smol: installed in $D from $(git rev-parse --short HEAD)$(git diff --quiet HEAD -- tools docs || echo '-dirty')"
