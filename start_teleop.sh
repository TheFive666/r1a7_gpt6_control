#!/usr/bin/env bash
set -euo pipefail

# Start the R1 Dex1 gripper server and the upgraded four-camera image bridge
# on robot PC2 (normally 192.168.123.164).

SESSION="${SESSION:-r1_dex1_services_new}"
ROS_SELECTION="${ROS_SELECTION:-1}"
REPO_DIR="${REPO_DIR:-/home/unitree}"
DEX1_BIN_DIR="${DEX1_BIN_DIR:-/home/unitree/dex1_1_service/bin}"
DEX1_SERVER="${DEX1_SERVER:-./dex1_1_gripper_server}"
DEX1_NETWORK="${DEX1_NETWORK:-eth10}"

# The upgraded robot publishes its official head streams as RTP/JPEG at 30 Hz.
HEAD_CODEC="${HEAD_CODEC:-jpeg}"
HEAD_LEFT_PORT="${HEAD_LEFT_PORT:-5004}"
HEAD_RIGHT_PORT="${HEAD_RIGHT_PORT:-5005}"
HEAD_PAYLOAD="${HEAD_PAYLOAD:-26}"
HEAD_LATENCY="${HEAD_LATENCY:-0}"

WRIST_LEFT="${WRIST_LEFT:-/dev/cam_wrist_left}"
WRIST_RIGHT="${WRIST_RIGHT:-/dev/cam_wrist_right}"
WRIST_FOURCC="${WRIST_FOURCC:-YUYV}"
WRIST_EXPOSURE="${WRIST_EXPOSURE:-9000}"
WRIST_GAIN="${WRIST_GAIN:-16}"

SWAP_WRIST="${SWAP_WRIST:-false}"
LEFT_WRIST_ROTATION="${LEFT_WRIST_ROTATION:-none}"
RIGHT_WRIST_ROTATION="${RIGHT_WRIST_ROTATION:-180}"

IMAGE_SERVER="${IMAGE_SERVER:-/home/unitree/image_server_dex1_new.py}"
SYSTEM_PYTHON="${SYSTEM_PYTHON:-/usr/bin/python3}"

if [ ! -x "$SYSTEM_PYTHON" ]; then
    echo "System Python is not executable: $SYSTEM_PYTHON" >&2
    exit 1
fi

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "tmux session '$SESSION' already exists. Attaching..."
    exec tmux attach-session -t "$SESSION"
fi

GRIPPER_CMD="cd '$DEX1_BIN_DIR' && echo '[gripper] starting Dex1 gripper server on $DEX1_NETWORK' && sudo $DEX1_SERVER --network '$DEX1_NETWORK'"

IMAGE_CMD="cd '$REPO_DIR' && \
echo '[image] starting upgraded R1 four-camera bridge with $SYSTEM_PYTHON' && \
'$SYSTEM_PYTHON' '$IMAGE_SERVER' bridge \
  --head-codec '$HEAD_CODEC' \
  --left-port '$HEAD_LEFT_PORT' \
  --right-port '$HEAD_RIGHT_PORT' \
  --payload '$HEAD_PAYLOAD' \
  --latency '$HEAD_LATENCY' \
  --wrist-left '$WRIST_LEFT' \
  --wrist-right '$WRIST_RIGHT' \
  --wrist-fourcc '$WRIST_FOURCC' \
  --left-wrist-rotation '$LEFT_WRIST_ROTATION' \
  --right-wrist-rotation '$RIGHT_WRIST_ROTATION'"

if [ "$SWAP_WRIST" = "true" ]; then
    IMAGE_CMD="$IMAGE_CMD --swap-wrist"
else
    IMAGE_CMD="$IMAGE_CMD --no-swap-wrist"
fi
if [ -n "$WRIST_EXPOSURE" ]; then
    IMAGE_CMD="$IMAGE_CMD --wrist-exposure '$WRIST_EXPOSURE'"
fi
if [ -n "$WRIST_GAIN" ]; then
    IMAGE_CMD="$IMAGE_CMD --wrist-gain '$WRIST_GAIN'"
fi

if ! command -v tmux >/dev/null 2>&1; then
    LOG_DIR="${LOG_DIR:-$HOME/r1_dex1_logs_new}"
    mkdir -p "$LOG_DIR"
    echo "tmux is not installed; starting both services in the background."
    echo "Logs: $LOG_DIR/gripper.log and $LOG_DIR/image.log"
    sudo -v
    nohup bash -lc "$GRIPPER_CMD" >"$LOG_DIR/gripper.log" 2>&1 &
    GRIPPER_PID=$!
    nohup bash -lc "$IMAGE_CMD" >"$LOG_DIR/image.log" 2>&1 &
    IMAGE_PID=$!
    echo "Gripper PID: $GRIPPER_PID"
    echo "Image server PID: $IMAGE_PID"
    echo "Stop: kill $GRIPPER_PID $IMAGE_PID"
    exit 0
fi

tmux new-session -d -s "$SESSION" -n services
tmux split-window -v -t "$SESSION":0
tmux select-layout -t "$SESSION":0 tiled >/dev/null

# The robot shell asks for the ROS distribution when a tmux pane starts.
# Select Foxy before sending the service commands.
sleep 0.5
tmux send-keys -t "$SESSION":0.0 "$ROS_SELECTION" C-m
tmux send-keys -t "$SESSION":0.1 "$ROS_SELECTION" C-m
sleep 0.5

tmux send-keys -t "$SESSION":0.0 "$GRIPPER_CMD" C-m
tmux send-keys -t "$SESSION":0.1 "$IMAGE_CMD" C-m

tmux select-pane -t "$SESSION":0.0
echo "Started tmux session '$SESSION'."
echo "Pane 0: Dex1 gripper server"
echo "Pane 1: upgraded R1 four-camera image bridge"
exec tmux attach-session -t "$SESSION"
