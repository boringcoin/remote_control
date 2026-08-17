#!/bin/bash
# ============================================================
# Sichiray Glove → AuraOS Teleoperation Launcher
#
# Usage:
#   ./start_glove_teleop.sh             # LIVE mode (drives robot)
#   ./start_glove_teleop.sh --dry-run   # Monitor only, no movement
#   ./start_glove_teleop.sh --calibrate # Recalibrate then drive
# ============================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
VENV_PYTHON="/home/sunrise/auraos/.venv-linux/bin/python"
CONTROLLER="$SCRIPT_DIR/rdk_sichiray_glove_http_controller.py"
CALIB_FILE="$SCRIPT_DIR/glove_calibration_roll_pitch.json"
LOG_FILE="$SCRIPT_DIR/glove_teleop_$(date +%Y%m%d_%H%M%S).log"
API_BASE="http://127.0.0.1:8765/api/motion"

# ---- Safety defaults ----
DEADZONE=8.0
SPEED=150          # mm/s max wheel speed
TURN_SPEED=80      # mm/s max turn differential
LIVE_FLAG="--live"

# ---- Parse args ----
MODE="live"
while [[ $# -gt 0 ]]; do
    case "$1" in
        --dry-run)
            LIVE_FLAG=""
            MODE="dry-run"
            shift ;;
        --calibrate)
            echo ">>> Removing old calibration to force recalibration..."
            rm -f "$CALIB_FILE"
            shift ;;
        --speed)
            SPEED="$2"; shift 2 ;;
        --turn-speed)
            TURN_SPEED="$2"; shift 2 ;;
        --deadzone)
            DEADZONE="$2"; shift 2 ;;
        *)
            echo "Unknown arg: $1"
            echo "Usage: $0 [--dry-run] [--calibrate] [--speed N] [--turn-speed N] [--deadzone N]"
            exit 1 ;;
    esac
done

# ---- Pre-flight checks ----
if [ ! -f "$CONTROLLER" ]; then
    echo "ERROR: Controller not found at $CONTROLLER"
    exit 1
fi

if [ ! -x "$VENV_PYTHON" ]; then
    echo "ERROR: venv python not found at $VENV_PYTHON"
    exit 1
fi

# Check daemon motion API
echo ">>> Checking AuraOS motion API..."
STATUS=$(curl -sf http://127.0.0.1:8765/api/motion/status 2>&1 || true)
if echo "$STATUS" | grep -q '"connected":true'; then
    echo "    Motion backend OK: $STATUS"
else
    echo "    WARNING: Motion backend may not be ready: $STATUS"
fi

if [ -f "$CALIB_FILE" ]; then
    echo ">>> Calibration: $(cat "$CALIB_FILE")"
    CALIB_MSG="pre-loaded"
else
    echo ">>> Calibration: will collect $(date +%H:%M:%S) — keep glove NEUTRAL for ~1s"
    CALIB_MSG="live calibration"
fi

echo ">>> Mode: $MODE | speed=$SPEED turn=$TURN_SPEED deadzone=$DEADZONE | $CALIB_MSG"
echo ">>> Log: $LOG_FILE"
echo ">>> Press Ctrl+C to stop."
echo ""

# ---- Launch ----
exec "$VENV_PYTHON" "$CONTROLLER" \
    $LIVE_FLAG \
    --calibration-file "$CALIB_FILE" \
    --speed "$SPEED" \
    --turn-speed "$TURN_SPEED" \
    --deadzone "$DEADZONE" \
    --api-base "$API_BASE" \
    --log-file "$LOG_FILE"
