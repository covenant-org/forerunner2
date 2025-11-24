#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname "${BASH_SOURCE[0]}")" && pwd)

SERVICE_TEMPLATE="${SCRIPT_DIR}/rerun-comprobation.service"
SERVICE_TIMER_TEMPLATE="${SCRIPT_DIR}/rerun-comprobation.timer"
SERVICE_TARGET="${HOME}/.config/systemd/user/"
MC_ALIAS="covenant_server"
IP="[IP]" # TODO: Reemplazar con la IP del servidor real de covenant
PORT="[PORT]" # TODO: Reemplazar con el puerto del servidor real de covenant
ENDPOINT="https://${IP}:${PORT}"
RELATIVE_BINARY_PATH="build/rerun/rerun_comprobation"

# Check for MINIO credentials
if [ -z "${MINIO_ACCESS_KEY:-}" ] || [ -z "${MINIO_SECRET_KEY:-}" ]; then
  echo "Error: MINIO_ACCESS_KEY and/or MINIO_SECRET_KEY environment variables are not set."
  exit 1
fi

# Locate project root directory
ROOT_DIR=""
current="$SCRIPT_DIR"

while [[ "$current" != "/" ]]; do
  if [[ -e "$current/.root" ]]; then
    ROOT_DIR="$current"
    break
  fi
  current=$(dirname "$current")
done

BINARY_PATH="${ROOT_DIR}/${RELATIVE_BINARY_PATH}"

## Installation and configuration of mc (minio client)
curl -fsSLo /tmp/mc https://dl.min.io/client/mc/release/linux-amd64/mc \
	&& sudo install -m 0755 /tmp/mc /usr/local/bin/mc

mc alias set ${MC_ALIAS} ${ENDPOINT} ${MINIO_ACCESS_KEY} ${MINIO_SECRET_KEY}

sed "s|rerun_comprobation|${BINARY_PATH}|g" "$SERVICE_TEMPLATE" > "/tmp/${SERVICE_TEMPLATE}"
# Configure service and timer
TMP_SERVICE="/tmp/${SERVICE_TEMPLATE}"
TMP_TIMER="/tmp/${SERVICE_TIMER_TEMPLATE}"

# Replace binary path in .service and copy
sed "s|rerun_comprobation|${BINARY_PATH}|g" "$SERVICE_TEMPLATE" > "$TMP_SERVICE"}"
cp "$TMP_SERVICE" "$SERVICE_TARGET"

cp "$SERVICE_TIMER_TEMPLATE" "$TMP_TIMER"
cp "$TMP_TIMER" "$SERVICE_TARGET"

systemctl daemon-reload
systemctl --user enable rerun-comprobation.timer
systemctl --user start rerun-comprobation.timer