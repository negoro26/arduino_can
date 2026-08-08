#!/usr/bin/env bash
# Open the serial monitor for the MKR1000 bring-up firmware.
#
# PlatformIO installs its own venv and does not add itself to PATH, so resolve
# it here rather than relying on the shell environment.
set -euo pipefail

PIO="$HOME/.platformio/penv/bin/pio"
if [[ ! -x "$PIO" ]]; then
  echo "error: pio not found at $PIO" >&2
  exit 1
fi

cd "$(dirname "$0")"

# Serial devices are root:uucp 0660. If this shell's credentials predate being
# added to the uucp group, re-exec through sg to pick the membership up.
if [[ ! -r /dev/ttyACM0 ]] && ! id -nG | tr ' ' '\n' | grep -qx uucp; then
  echo "note: no uucp group in this shell, re-running via sg" >&2
  exec sg uucp -c "$0 $*"
fi

cat <<'EOF'
------------------------------------------------------------------
 MKR1000 CAN bring-up monitor
   stages 1-3 run automatically and are PASSIVE (cannot disturb
   the vehicle bus)
   press  s   to arm stage 4 (ACTIVE - transmits on the bus)
   press  Ctrl-C  to quit
 Everything is logged to ./logs/device-monitor-*.log
------------------------------------------------------------------
EOF
exec "$PIO" device monitor
