#!/usr/bin/env bash
set -euo pipefail

## TODO: Asegurar de establecer alias
## TODO: Bash de instalacion de mc, ajuste de alias y creacion de servicio
RELATIVE_DIR="rerun/recordings"
MC_ALIAS="covenant_server"
MC_BUCKET="rerun"

MC_DESTINATION="${MC_ALIAS}/${MC_BUCKET}"

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

PROBE_BIN_OVERRIDE="${COMPROBATION_BIN:-}"
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

if [[ -z "$MC_DESTINATION" ]]; then
  echo "MC_DESTINATION must be set." >&2
  exit 1
fi

if ! command -v mc >/dev/null 2>&1; then
  echo "Unable to locate mc binary: mc" >&2
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

remote_base=${MC_DESTINATION%/}
[[ -z "$remote_base" ]] && remote_base="$MC_DESTINATION"

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

  echo "Transferring ${file} -> ${remote_target}"
  mc cp "$file" "$remote_target" &
  mc_pid=$!

  while kill -0 "$mc_pid" >/dev/null 2>&1; do
    if ! check_conditions quiet; then
      echo "Conditions changed during transfer of ${file}. Cancelling." >&2
      kill "$mc_pid" >/dev/null 2>&1 || true
      wait "$mc_pid" >/dev/null 2>&1 || true
      mc rm --force "$remote_target" >/dev/null 2>&1 || true
      abort_transfer=true
      break 2
    fi
    sleep 1
  done

  if ! wait "$mc_pid"; then
    echo "mc cp failed for ${file}. Stopping." >&2
    abort_transfer=true
    break
  fi

  rm -f "$file"
done

if [[ "$abort_transfer" == "true" ]]; then
  echo "Transfer session ended early due to failing conditions." >&2
fi

find "$SOURCE_DIR" -type d -empty -delete
