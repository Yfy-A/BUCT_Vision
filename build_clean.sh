#!/usr/bin/env bash
set -euo pipefail

# Build in a clean overlay context to avoid overriding packages from this same workspace.
WS_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_INSTALL="$WS_DIR/install"

sanitize_colon_path_var() {
    local var_name="$1"
    local remove_prefix="$2"
    local current="${!var_name-}"

    if [[ -z "$current" ]]; then
        return 0
    fi

    local filtered=""
    local entry
    IFS=':' read -ra entries <<< "$current"
    for entry in "${entries[@]}"; do
        [[ -z "$entry" ]] && continue
        # Remove this workspace install prefixes from underlay-like path variables.
        if [[ "$entry" == "$remove_prefix" || "$entry" == "$remove_prefix"/* ]]; then
            continue
        fi
        if [[ -z "$filtered" ]]; then
            filtered="$entry"
        else
            filtered="$filtered:$entry"
        fi
    done

    export "$var_name=$filtered"
}

sanitize_colon_path_var COLCON_PREFIX_PATH "$WS_INSTALL"
sanitize_colon_path_var AMENT_PREFIX_PATH "$WS_INSTALL"
sanitize_colon_path_var CMAKE_PREFIX_PATH "$WS_INSTALL"

cd "$WS_DIR"
exec colcon build "$@"
