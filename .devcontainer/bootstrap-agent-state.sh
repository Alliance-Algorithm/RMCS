#!/usr/bin/env bash

set -euo pipefail

is_uninitialized_dir() {
  local dir="$1"
  local entries

  shopt -s dotglob nullglob
  entries=("${dir}"/*)
  shopt -u dotglob nullglob

  if [ "${#entries[@]}" -eq 0 ]; then
    return 0
  fi

  if [ "${#entries[@]}" -eq 1 ] && [ "$(basename "${entries[0]}")" = ".gitignore" ]; then
    return 0
  fi

  return 1
}

copy_dir_contents() {
  local src="$1"
  local dst="$2"

  mkdir -p "${dst}"
  cp -a "${src}"/. "${dst}"/
}

bootstrap_dir() {
  local src="$1"
  local dst="$2"
  local label="$3"

  mkdir -p "${dst}"

  if ! is_uninitialized_dir "${dst}"; then
    echo "[bootstrap-agent-state] Skip ${label}: state already initialized"
    return 0
  fi

  if [ ! -d "${src}" ]; then
    echo "[bootstrap-agent-state] Skip ${label}: source ${src} not found"
    return 0
  fi

  echo "[bootstrap-agent-state] Importing ${label} from ${src}"
  copy_dir_contents "${src}" "${dst}"
}

bootstrap_dir "/mnt/host-agent-source/codex" "${HOME}/.codex" "Codex state"
bootstrap_dir "/mnt/host-agent-source/opencode-config" "${HOME}/.config/opencode" "OpenCode config"
bootstrap_dir "/mnt/host-agent-source/opencode-data" "${HOME}/.local/share/opencode" "OpenCode data"
bootstrap_dir "/mnt/host-agent-source/opencode-skills" "${HOME}/.opencode/skills" "OpenCode skills"
