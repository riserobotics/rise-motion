#!/bin/bash
#
# Author: Furkan M. Lafci
# Email: f.lafci@campus.tu-berlin.de
#
# This updates the git submodules for the rise-os-core repository. It has been moved to the rise-motiuon
# repositorie for the same reason.
# The script requires a clean working directory (no uncommitted changes) to prevent merge conflicts.
#
# It:
#   - Verifies the Git working tree is clean before proceeding
#   - Reads each submodule entry from .gitmodules
#   - For each submodule, pulls the configured branch or checks out the configured tag
#   - Optionally commits all submodule pointer updates

set -e
clear

# ============== Helper Functions

RED='\033[0;31m'
YELLOW='\033[1;33m'
GREEN='\033[0;32m'
CYAN='\033[0;36m'
BOLD='\033[1m'
RESET='\033[0m'


ask_yes_no() {
    local prompt="$1"
    local response
    while true; do
        read -p "$prompt" -r response
        echo
        case "$response" in
            [Yy]) return 0 ;;
            [Nn]) return 1 ;;
            *) echo "Invalid input. Please enter 'y' or 'n'." ;;
        esac
    done
}

# ============= MAIN SCRIPT

# Go to project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$SCRIPT_DIR/.."
cd "$PROJECT_ROOT"


# Check for clean working tree
if ! git diff --quiet || ! git diff --cached --quiet; then
    echo -e "${RED}Error: Working directory has uncommitted changes.${RESET}"
    exit 1
fi

# Parse .gitmodules, returns [submodule_path, url, branch, tag] for each submodule
mapfile -t SUBMODULE_PATHS < <(git config --file .gitmodules --get-regexp path | awk '{print $2}')

# Initialize Submodules if not already done
echo -e "${CYAN}Initializing submodules...${RESET}"
git submodule update --init --recursive
echo

# Check for updates in each submodule
# If branch is configured, pull latest changes
# If tag is configured, checkout the tag
for SUBMODULE_PATH in "${SUBMODULE_PATHS[@]}"; do
    echo -e "${CYAN}Updating submodule: ${BOLD}$SUBMODULE_PATH${RESET}"
    BRANCH=$(git config --file .gitmodules --get "submodule.$SUBMODULE_PATH.branch" || true)
    TAG=$(git config --file .gitmodules --get "submodule.$SUBMODULE_PATH.tag" || true)

    if [[ -n "$BRANCH" && -n "$TAG" ]]; then
        echo -e "${RED}Error: Both branch and tag set for $SUBMODULE_PATH${RESET}"
        exit 1
    fi

    if [[ -n "$BRANCH" ]]; then
        echo -e "Tracking branch ${YELLOW}$BRANCH${RESET}"
        git -C "$SUBMODULE_PATH" fetch origin "$BRANCH"
        git -C "$SUBMODULE_PATH" checkout "$BRANCH"
        git -C "$SUBMODULE_PATH" reset --hard "origin/$BRANCH"

    elif [[ -n "$TAG" ]]; then
        echo -e "Checking out tag ${YELLOW}$TAG${RESET}"
        git -C "$SUBMODULE_PATH" fetch --tags
        git -C "$SUBMODULE_PATH" checkout "tags/$TAG"

    else
        echo -e "${YELLOW}No branch/tag → skipping${RESET}"
        echo
        continue
    fi
    echo
done

# Commit submodule pointer updates if there are any changes
if ! git diff --quiet; then
    echo
    echo -e "${GREEN}Submodules were updated.${RESET}"

    if ask_yes_no "Commit updates? (y/n): "; then
        git add "${SUBMODULE_PATHS[@]}"
        git commit -m "Update submodule pointers"
        echo -e "${GREEN}Committed.${RESET}"
    else
        echo -e "${YELLOW}Skipped commit.${RESET}"
    fi
else
    echo -e "${GREEN}All submodules up to date.${RESET}"
fi
