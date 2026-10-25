#!/usr/bin/env bash
set -euo pipefail

PIO="$(command -v pio || true)"
for candidate in "$HOME/.platformio/penv/bin/pio" "$HOME/.local/bin/pio"; do
  [[ -n "$PIO" ]] && break
  [[ -x "$candidate" ]] && PIO="$candidate"
done
if [[ -z "$PIO" ]]; then
  echo "error: pio not found on PATH, in ~/.platformio/penv/bin or ~/.local/bin" >&2
  exit 1
fi

cd "$(dirname "$0")"

# Serial devices are root:uucp 0660. A shell started before the user joined
# uucp does not have the group yet.
if [[ ! -r /dev/ttyACM0 ]] && ! id -nG | tr ' ' '\n' | grep -qx uucp; then
  if command -v sg >/dev/null; then
    echo "note: no uucp group in this shell, re-running via sg" >&2
    exec sg uucp -c "$0 $*"
  fi
  echo "error: /dev/ttyACM0 is not readable; log out and back in after joining uucp" >&2
  exit 1
fi

cat <<'EOF'
------------------------------------------------------------------
 MKR1000 ELM327 adapter
   ATL1          readable line endings in this terminal
   AT#SURVEY     listen-only wiring check (transmits nothing)
   AT#SELFTEST   MCP2515 presence and loopback test
   AT#GUARD      show the write guard tier (READ by default)
   Ctrl-C        quit
 Everything is logged to ./logs/device-monitor-*.log
------------------------------------------------------------------
EOF
exec "$PIO" device monitor
