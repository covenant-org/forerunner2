#!/usr/bin/env bash
set -euo pipefail

## TODO: cambiar direcciones de ssh y carpetas a direcciones finales
RELATIVE_DIR="rerun/test-records-send"
REMOTE_USER="manuelo247"
REMOTE_HOST="192.168.100.11"
REMOTE_PATH="~/test"
PROBE_BIN_OVERRIDE="${COMPROBATION_BIN:-}"

SCRIPT_DIR=$(cd -- "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROOT_DIR=""
current="$SCRIPT_DIR"

while [[ "$current" != "/" ]]; do
  if [[ -e "$current/.root" ]]; then
    ROOT_DIR="$current"
    break
  fi
  current=$(dirname "$current")
done

if [[ -z "$ROOT_DIR" ]]; then
  echo "Unable to locate .root marker file." >&2
  exit 1
fi

SOURCE_DIR="${ROOT_DIR}/${RELATIVE_DIR}"
PROBE_BIN="${PROBE_BIN_OVERRIDE:-${ROOT_DIR}/build/rerun/rerun_comprobation}"

if [[ ! -d "$SOURCE_DIR" ]]; then
  echo "Source directory not found: $SOURCE_DIR" >&2
  exit 1
fi

if [[ ! -x "$PROBE_BIN" ]]; then
  echo "Connectivity probe not executable: $PROBE_BIN" >&2
  exit 1
fi

check_conditions() {
  local mode="${1:-verbose}"
  local output
  if ! output="$($PROBE_BIN)"; then
    [[ "$mode" == "verbose" ]] && echo "Probe execution failed." >&2
    [[ "$mode" == "verbose" ]] && echo "$output" >&2
    return 1
  fi

  local internet="" registry=""
  while IFS='=' read -r key value; do
    case "$key" in
      internet) internet="$value" ;;
      registry) registry="$value" ;;
    esac
  done < <(tr ' ' '\n' <<<"$output")

  if [[ "$mode" == "verbose" ]]; then
    echo "$output"
  fi

  if [[ "$internet" != "true" || "$registry" != "true" ]]; then
    return 1
  fi
  return 0
}

if ! check_conditions verbose; then
  echo "Initial conditions not met. Aborting transfers." >&2
  exit 0
fi

mapfile -t files < <(find "$SOURCE_DIR" -type f -print)

if ((${#files[@]} == 0)); then
  echo "No files to transfer from ${SOURCE_DIR}."
  exit 0
fi

trap 'echo "Interrupt received, stopping."; exit 1' INT TERM

remote_home=$(ssh "${REMOTE_USER}@${REMOTE_HOST}" 'printf %s "$HOME"') || {
  echo "Failed to determine remote home directory." >&2
  exit 1
}

resolve_remote_path() {
  local path="$1"
  if [[ "$path" == "~" ]]; then
    printf '%s' "$remote_home"
  elif [[ "$path" == ~/* ]]; then
    printf '%s/%s' "$remote_home" "${path:2}"
  else
    printf '%s' "$path"
  fi
}

escape_remote_path() {
  printf "%s" "$1" | sed "s/'/'\\''/g"
}

remote_base=$(resolve_remote_path "$REMOTE_PATH")
remote_base=${remote_base%/}
[[ -z "$remote_base" ]] && remote_base="."

abort_transfer=false

for file in "${files[@]}"; do
  [[ -f "$file" ]] || continue

  if ! check_conditions quiet; then
    echo "Conditions changed before transferring ${file}. Stopping." >&2
    abort_transfer=true
    break
  fi

  rel_path=${file#"${SOURCE_DIR}/"}
  remote_target="$remote_base/$rel_path"
  remote_dir=$(dirname "$remote_target")

  remote_dir_escaped=$(escape_remote_path "$remote_dir")
  remote_target_escaped=$(escape_remote_path "$remote_target")

  echo "Transferring ${file} -> ${REMOTE_USER}@${REMOTE_HOST}:${remote_target}"
  ssh "${REMOTE_USER}@${REMOTE_HOST}" "mkdir -p '$remote_dir_escaped'" >/dev/null

  scp "$file" "${REMOTE_USER}@${REMOTE_HOST}:'$remote_target_escaped'" &
  scp_pid=$!

  while kill -0 "$scp_pid" >/dev/null 2>&1; do
    if ! check_conditions quiet; then
      echo "Conditions changed during transfer of ${file}. Cancelling." >&2
  kill "$scp_pid" >/dev/null 2>&1 || true
  wait "$scp_pid" >/dev/null 2>&1 || true
  ssh "${REMOTE_USER}@${REMOTE_HOST}" "rm -f '$remote_target_escaped'" >/dev/null 2>&1 || true
      abort_transfer=true
      break 2
    fi
    sleep 1
  done

  if ! wait "$scp_pid"; then
    echo "scp failed for ${file}. Stopping." >&2
    abort_transfer=true
    break
  fi

  rm -f "$file"
done

if [[ "$abort_transfer" == "true" ]]; then
  echo "Transfer session ended early due to failing conditions." >&2
fi

find "$SOURCE_DIR" -type d -empty -delete
