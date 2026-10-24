#!/usr/bin/env bash
## =========================================================================== ##
# MIT License
# Copyright (c) 2026 JIC Smart Factory
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
## =========================================================================== ##
# Author   : Roman Parak
# File Name: ../install.sh
## =========================================================================== ##

# Description:
#   Install the KINALI_Test_it_off_Simulation environment: a .venv with the pinned requirements,
#   checked by importing them, and the run-time folders logger/ and Data/Generated/.
#
#   Steps, cheapest check first:
#     1. Detect the platform: macOS on Apple silicon (verified) or Ubuntu Linux (written and tested
#        only against stubs: unverified).
#     2. Find Python 3.11 by an explicit name: on macOS the Homebrew paths, then python3.11 on PATH;
#        on Linux python3.11 on PATH, then /usr/bin/python3.11. A bare python3 is never called (on
#        the development Mac it is miniforge's).
#     3. Create .venv if it is absent and reuse a working one. A broken .venv is reported with the
#        command to remove it; it is deleted only with --recreate.
#     4. Upgrade pip and pip install -r requirements.txt (every version pinned).
#     5. Verify: import mujoco, numpy, scipy and yaml, print their versions, require MuJoCo 3.16.0.
#     6. Create logger/ (logs) and Data/Generated/ (generated artefacts), both git-ignored.
#     7. Print the next commands.
#   No sudo, no global installs; every step is re-runnable.
#
#   Usage:
#     ./install.sh                  run all steps
#     ./install.sh --recreate       delete .venv first, then run all steps
#     ./install.sh --detect-only    report the platform, the interpreter and the plan, then stop
#
#   Output: every line is 'YYYY-MM-DD HH:MM:SS [LEVEL  ] message', the level padded to 7.

set -euo pipefail

# Description:
#   Initialization of constants.

# The repository root: the directory holding this script, so it works from any directory.
#   Unit [string]
CONST_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# The Python version every pinned requirement was verified with (requirements.txt header).
#   Unit [string]
CONST_PYTHON_VERSION="3.11"
# The MuJoCo version the physics was verified on (design-decisions: MuJoCo is pinned to 3.16.0).
#   Unit [string]
CONST_MUJOCO_VERSION="3.16.0"
# The interpreters tried on macOS, in order: Homebrew on Apple silicon, Homebrew on Intel, then the
# versioned name on PATH. Explicit names only.
#   Unit [string] (space-separated)
CONST_PYTHON_MACOS="/opt/homebrew/bin/python3.11 /usr/local/bin/python3.11 python3.11"
# The prerequisite's install command on macOS (shown, never run).
#   Unit [string]
CONST_INSTALL_MACOS="brew install python@3.11"
# The interpreters tried on Linux, in order: the versioned name on PATH, then the distribution path.
#   Unit [string] (space-separated)
CONST_PYTHON_LINUX="python3.11 /usr/bin/python3.11"
# The prerequisite's install command on Linux (shown, never run; on Ubuntu releases without 3.11 the
# deadsnakes PPA provides it).
#   Unit [string]
CONST_INSTALL_LINUX="sudo apt install python3.11 python3.11-venv (Ubuntu; older releases: sudo add-apt-repository ppa:deadsnakes/ppa first)"
# The supported platforms, as the failure message names them.
#   Unit [string]
CONST_PLATFORMS_TEXT="macOS (Apple silicon), Ubuntu Linux"
# The folders created for logs and generated artefacts, relative to the root (git-ignored; design-decisions).
#   Unit [string] (space-separated)
CONST_RUN_FOLDERS="logger Data/Generated"
# Width the level is padded to, so the messages line up.
#   Unit [int]
CONST_LEVEL_WIDTH=7

# Description:
#   Print one log line in the project's format.
# Args:
#   (1) level [string]: INFO, WARNING or ERROR.
#   (2) message [string]: The message.
# Returns:
#   (1) parameter [string]: The line, on stdout.
Log() {
    local level="$1"; shift
    printf '%s [%-*s] %s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$CONST_LEVEL_WIDTH" "$level" "$*"
}

# Description:
#   Report one or more ERROR lines and exit non-zero. The last line says what to do.
# Args:
#   (1 - n) message [string]: One ERROR line each.
# Returns:
#   (1) parameter [int]: Exit status 1.
Die() {
    local line
    for line in "$@"; do Log ERROR "$line"; done
    exit 1
}

# Description:
#   Name the line and command of anything unanticipated, instead of a bare trace.
# Args:
#   (1) line [int]: The line number.
#   (2) command [string]: The failing command.
# Returns:
#   None
On_Unexpected_Error() {
    Log ERROR "Unexpected failure at install.sh line $1: $2"
    Log ERROR "Nothing was deleted. Fix the cause above and run ./install.sh again."
    exit 1
}

# Description:
#   Interrupted: say that re-running is safe.
# Args:
#   None
# Returns:
#   None
On_Interrupt() {
    Log WARNING "Interrupted. Every step is re-runnable; run ./install.sh again."
    exit 130
}

# Description:
#   Detect the operating system and architecture.
# Args:
#   None
# Returns:
#   (1) parameter [string]: Sets PLATFORM to 'macos' or 'linux'; fails on anything else.
Detect_Platform() {
    local os arch
    os="$(uname -s)"; arch="$(uname -m)"
    case "$os" in
        Darwin)
            PLATFORM="macos"
            if [ "$arch" = "arm64" ]; then
                Log INFO "Detected macOS $(sw_vers -productVersion 2>/dev/null || true) on Apple silicon ($arch)."
            else
                Log WARNING "Detected macOS on $arch; only Apple silicon has been verified."
            fi ;;
        Linux)
            PLATFORM="linux"
            Log INFO "Detected Linux ($arch). The Ubuntu path is written but not verified on hardware." ;;
        *)
            Die "Unsupported operating system '$os'." \
                "This project installs on $CONST_PLATFORMS_TEXT only." ;;
    esac
}

# Description:
#   Find Python 3.11 among the platform's explicit candidates and check its version.
# Args:
#   None
# Returns:
#   (1) parameter [string]: Sets PYTHON to the interpreter path; fails with the install command.
Find_Python() {
    local candidates install name path version tried=""
    case "$PLATFORM" in
        macos) candidates="$CONST_PYTHON_MACOS"; install="$CONST_INSTALL_MACOS" ;;
        linux) candidates="$CONST_PYTHON_LINUX"; install="$CONST_INSTALL_LINUX" ;;
    esac
    for name in $candidates; do
        tried="$tried $name"
        # An absolute candidate is used as it is; a name is looked up on PATH. The presence check of
        # the prerequisite: its install command is shown, never run.
        case "$name" in
            /*) path="$name" ;;
            *) path="$(command -v "$name" || true)" ;;
        esac
        if [ -z "$path" ] || [ ! -x "$path" ]; then continue; fi
        version="$("$path" -c 'import sys; print("%d.%d" % sys.version_info[:2])' 2>/dev/null || true)"
        if [ "$version" = "$CONST_PYTHON_VERSION" ]; then
            PYTHON="$path"
            Log INFO "Using Python $version at $PYTHON."
            return 0
        fi
        Log WARNING "$path reports Python '${version:-unknown}', not $CONST_PYTHON_VERSION; skipped."
    done
    Die "Python $CONST_PYTHON_VERSION was not found (tried:$tried)." \
        "Install Python $CONST_PYTHON_VERSION with: $install"
}

# Description:
#   Create .venv if absent and reuse a working one; with --recreate delete it first. A broken .venv
#   is reported with the command to remove it, never deleted without the flag.
# Args:
#   (1) recreate [string]: 'yes' to delete an existing .venv first.
# Returns:
#   None
Ensure_Venv() {
    local venv="$CONST_ROOT/.venv" version
    if [ "$1" = "yes" ] && [ -d "$venv" ]; then
        rm -rf "$venv"
        Log WARNING "--recreate: deleted $venv."
    fi
    if [ -d "$venv" ]; then
        version="$("$venv/bin/python" -c 'import sys; print("%d.%d" % sys.version_info[:2])' 2>/dev/null || true)"
        if [ "$version" = "$CONST_PYTHON_VERSION" ]; then
            Log INFO ".venv already present and working (Python $CONST_PYTHON_VERSION); reused."
            return 0
        fi
        Die ".venv exists but its interpreter is missing or Python '${version:-none}', not $CONST_PYTHON_VERSION." \
            "Run ./install.sh --recreate, or remove it with: rm -rf \"$venv\""
    fi
    "$PYTHON" -m venv "$venv" || Die "Creating .venv failed." "Check that the venv module is installed for $PYTHON."
    Log INFO "Created .venv with $PYTHON."
}

# Description:
#   Upgrade pip and install the pinned requirements into .venv; pip's output is kept and only its
#   tail is shown on failure.
# Args:
#   None
# Returns:
#   None
Install_Requirements() {
    local out line
    out="$(mktemp)"
    if "$CONST_ROOT/.venv/bin/python" -m pip install --disable-pip-version-check --upgrade pip > "$out" 2>&1 \
        && "$CONST_ROOT/.venv/bin/python" -m pip install --disable-pip-version-check -r "$CONST_ROOT/requirements.txt" >> "$out" 2>&1; then
        Log INFO "pip: $("$CONST_ROOT/.venv/bin/python" -m pip --version | cut -d' ' -f1-2)."
        if grep -q '^Successfully installed' "$out"; then
            grep '^Successfully installed' "$out" | sed 's/^Successfully installed //' | while IFS= read -r line; do
                Log INFO "Installed: $line"
            done
        else
            Log INFO "pip changed nothing; every pinned requirement is already installed."
        fi
        rm -f "$out"
    else
        tail -n 15 "$out" | while IFS= read -r line; do Log ERROR "pip: $line"; done
        rm -f "$out"
        Die "pip install -r requirements.txt failed (output above)." "Fix the cause and run ./install.sh again."
    fi
}

# Description:
#   Import the main requirements in .venv, print their versions and require MuJoCo 3.16.0.
# Args:
#   None
# Returns:
#   None
Verify_Install() {
    local versions mujoco
    versions="$("$CONST_ROOT/.venv/bin/python" -c 'import sys, mujoco, numpy, scipy, yaml; print("Python %d.%d.%d, mujoco %s, numpy %s, scipy %s, PyYAML %s" % (sys.version_info[:3] + (mujoco.__version__, numpy.__version__, scipy.__version__, yaml.__version__)))' 2>&1)" \
        || Die "Importing the requirements in .venv failed: $versions" "Run ./install.sh --recreate."
    Log INFO "Verified: $versions."
    mujoco="$("$CONST_ROOT/.venv/bin/python" -c 'import mujoco; print(mujoco.__version__)')"
    if [ "$mujoco" != "$CONST_MUJOCO_VERSION" ]; then
        Die "MuJoCo in .venv is $mujoco, not $CONST_MUJOCO_VERSION (the physics is verified on it only)." \
            "Fix requirements.txt and run ./install.sh --recreate."
    fi
}

# Description:
#   Create the run-time folders (git-ignored).
# Args:
#   None
# Returns:
#   None
Make_Folders() {
    local folder
    for folder in $CONST_RUN_FOLDERS; do
        mkdir -p "$CONST_ROOT/$folder"
    done
    Log INFO "Run-time folders present: $CONST_RUN_FOLDERS."
}

# Description:
#   Print the commands that come next.
# Args:
#   None
# Returns:
#   None
Print_Next_Steps() {
    Log INFO "Done. Next:"
    Log INFO "  run the cycle:  .venv/bin/python Example/Simulation/run_cycle.py"
    Log INFO "  run the tests:  .venv/bin/python -m pytest -m \"not slow\"   (full: .venv/bin/python -m pytest)"
    Log INFO "  open the viewer: .venv/bin/python -m mujoco.viewer --mjcf=models/cell_scene.xml"
}

# Description:
#   Run the steps in order.
# Args:
#   (1) option [string]: Optional --detect-only or --recreate.
# Returns:
#   None
Main() {
    local recreate="no"
    trap 'On_Unexpected_Error $LINENO "$BASH_COMMAND"' ERR
    trap On_Interrupt INT TERM
    case "${1:-}" in
        "") ;;
        --detect-only) ;;
        --recreate) recreate="yes" ;;
        *) Die "Unknown option '$1'." "Usage: ./install.sh [--detect-only | --recreate]" ;;
    esac
    Detect_Platform
    Find_Python
    Log INFO "Plan: .venv with Python $CONST_PYTHON_VERSION$([ "$recreate" = yes ] && echo ' (recreated)' || true), pip upgraded, requirements.txt, verified imports, $CONST_RUN_FOLDERS."
    if [ "${1:-}" = "--detect-only" ]; then Log INFO "Detect only: stopping before any change."; return 0; fi
    Ensure_Venv "$recreate"
    Install_Requirements
    Verify_Install
    Make_Folders
    Print_Next_Steps
}

if [ "${BASH_SOURCE[0]}" = "$0" ]; then Main "$@"; fi
