#!/usr/bin/env bash

set -eu

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

history_path="$REPO_ROOT/.devcontainer/.zsh_history"
history_link="$HOME/.zsh_history"

mkdir -p "$(dirname "$history_path")"
touch "$history_path"
rm -f "$history_link"
ln -s "$history_path" "$history_link"

printf 'Shell history symlinked: %s -> %s\n' "$history_link" "$history_path"
