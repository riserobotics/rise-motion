#!/bin/bash
#
# Author: Furkan M. Lafci
# Email: f.lafci@campus.tu-berlin.de
#
# This script sets up the development environment for Linux and macOS hosts, including WSL on Windows.
# It is based on the script that is used in the rise-os-core repository, but adapted for use within rise-motion.
#
# It:
#   - Checks for Docker and VS Code (+ extensions), and offers to install them if missing
#   - Ensures Docker daemon is running
#   - Checks Git configuration and prompts to set up a personal access token for GitHub
#   - Clones the rise-motion repository if not already cloned
#   - Pulls the RISE-OS Docker image from GitHub Container Registry

set -e
clear

# ============== Helper Functions

RED='\033[0;31m'
YELLOW='\033[1;33m'
GREEN='\033[0;32m'
CYAN='\033[0;36m'
WHITE='\033[1;37m'
BOLD='\033[1m'
RESET='\033[0m'

detect_host_env() {
    case "$(uname -s)" in
        Darwin) echo "macos" ;;
        Linux) grep -qi microsoft /proc/version 2>/dev/null && echo "wsl" || echo "linux" ;;
        *) echo "unknown" ;;
    esac
}

get_shell_rc() {
    case "$(basename "$(getent passwd "$USER" | cut -d: -f7)")" in
        bash) echo "$HOME/.bashrc" ;;
        zsh) echo "$HOME/.zshrc" ;;
        fish) echo "$HOME/.config/fish/config.fish" ;;
        ksh) echo "$HOME/.kshrc" ;;
        csh|tcsh) echo "$HOME/.cshrc" ;;
        *) echo "$HOME/.profile" ;;
    esac
}

get_export_cmd() {
    case "$(basename "$(getent passwd "$USER" | cut -d: -f7)")" in
        fish) echo "set -gx $1 \"$2\"" ;;
        csh|tcsh) echo "setenv $1 \"$2\"" ;;
        *) echo "export $1=\"$2\"" ;;
    esac
}

dnf_add_repo_file() {
    local url="$1"

    # DNF version 5 changed the way repositories are added
    if dnf --version 2>/dev/null | head -1 | grep -qE '^5\.|^dnf5'; then
        echo
        echo -e "${CYAN}Detected dnf version 5${RESET}"
        sudo dnf config-manager addrepo --overwrite --from-repofile="$url"
    else
        echo
        echo -e "${CYAN}Detected dnf version 4${RESET}"
        sudo dnf config-manager --overwrite --add-repo "$url"
    fi
}

ask_yes_no() {
    local prompt="$1"
    local response
    while true; do
        read -p "$prompt" -r response
        echo
        case "$response" in
            [Yy]) return 0 ;;
            [Nn]) return 1 ;;
            *) echo -e "${YELLOW}Invalid input. Please enter 'y' or 'n'.${RESET}" ;;
        esac
    done
}

setup_token() {
    local rc_file="$1"
    local token_var="$2"

    if [ -n "${!token_var}" ]; then
        echo -e "${GREEN}GitHub access token found in environment${RESET}"
        return 0
    fi

    if [ -f "$rc_file" ]; then
        token_value=$(
            grep -E "^(export |setenv |set -gx )?$token_var=" "$rc_file" \
            | tail -n1 \
            | sed -E 's/^[^=]+=//; s/^["'\'']//; s/["'\'']$//'
        )


        if [ -n "$token_value" ]; then
            echo -e "${GREEN}GitHub access token found in $rc_file${RESET}"
            export "$token_var=$token_value"
            return 0
        fi
    fi

    echo
    echo -e "${BOLD}${WHITE}===================== GitHub Access Token Required =======================${RESET}"
    echo
    echo -e "Create a new personal access token at: ${CYAN}https://github.com/settings/tokens/new${RESET}"
    echo
    echo "Token settings:"
    echo "  Note: Access to rise-motion project"
    echo "  Expiration: at least 6 months from now"
    echo
    echo "Required scopes:"
    echo "  - repo (full control of repositories - for pushing/pulling code)"
    echo "  - workflow (update GitHub Actions workflows)"
    echo "  - read:packages (for pulling Docker images from GHCR)"
    echo
    echo "After creating the token, save it somewhere safe, as you won't be able to see it again."
    echo
    read -s -p "Enter your personal access token: " token_value
    echo

    export "$token_var=$token_value"

    echo
    if ask_yes_no "Save access token to $rc_file for future sessions? (y/n): "; then
        SAVE_TOKEN_TO_RC=1
    else
        SAVE_TOKEN_TO_RC=0
        echo -e "${YELLOW}Access token will not be persisted. You will need to enter it again next time.${RESET}"
    fi
}

print_header() {
    cat <<'EOF'
    
████  ███  ████ █████      █   █  ███  █████ ███  ███  █   █
█   █  █  █     █          ██ ██ █   █   █    █  █   █ ██  █
████   █   ███  ████  ████ █ █ █ █   █   █    █  █   █ █ █ █
█  █   █      █ █          █   █ █   █   █    █  █   █ █  ██
█   █ ███ ████  █████      █   █  ███    █   ███  ███  █   █

EOF
}

save_token_to_rc() {
    local rc_file="$1"
    local token_var="$2"
    local token_value="${!token_var}"

    if ! grep -q "$token_var" "$rc_file" 2>/dev/null; then
        if [ "${SAVE_TOKEN_TO_RC:-0}" = "1" ]; then
            echo
            echo -e "${CYAN}Adding $token_var to $rc_file for future sessions...${RESET}"
            echo "" >> "$rc_file"
            echo "# $token_var" >> "$rc_file"
            echo "$(get_export_cmd "$token_var" "$token_value")" >> "$rc_file"
            echo -e "${GREEN}Access token saved to $rc_file${RESET}"
        fi
    else
        echo -e "${GREEN}Access token already configured in $rc_file${RESET}"
    fi
}

wait_for_confirmation() {
    local waiting_msg="$1"
    local validation_cmd="$2"
    local success_msg="$3"

    echo -e "${CYAN}${waiting_msg}${RESET}"
    while true; do
        if eval "$validation_cmd" >/dev/null 2>&1; then
            echo -e "${GREEN}${success_msg}${RESET}"
            break
        fi
        sleep 5
    done
}

install_yay() {
    echo
    echo -e "${CYAN}Installing yay (AUR helper)...${RESET}"
    sudo pacman -S --needed git base-devel
    local tmp_dir
    tmp_dir=$(mktemp -d)
    git clone https://aur.archlinux.org/yay.git "$tmp_dir/yay"
    (cd "$tmp_dir/yay" && makepkg -si)
    rm -rf "$tmp_dir"
    echo -e "${GREEN}yay installed successfully.${RESET}"
}

install_docker() {
    case "$HOST_ENV" in

        macos)
            if command -v brew >/dev/null 2>&1; then
                echo -e "${CYAN}Installing Docker Desktop via Homebrew...${RESET}"
                brew install --cask docker
                echo -e "${GREEN}Docker Desktop installed.${RESET}"
                echo
                echo -e "${CYAN}Starting Docker Desktop...${RESET}"
                osascript -e 'tell application "Terminal" to do script "open -a Docker; exit"'
                wait_for_confirmation \
                    "Waiting for Docker daemon to be ready... (CTRL+C to exit script)" \
                    "docker info" \
                    "Docker daemon is now running."
            else
                echo -e "${YELLOW}Homebrew not found.${RESET}"
                if ask_yes_no "Install Homebrew now? (Required for full automation) (y/n): "; then
                    echo "Installing Homebrew..."
                    /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
                    echo -e "${GREEN}Homebrew installed successfully.${RESET}"
                    echo
                    echo -e "${CYAN}Installing Docker Desktop via Homebrew...${RESET}"
                    brew install --cask docker
                    echo -e "${GREEN}Docker Desktop installed.${RESET}"
                    echo
                    echo -e "${CYAN}Starting Docker Desktop...${RESET}"
                    osascript -e 'tell application "Terminal" to do script "open -a Docker; exit"'
                    wait_for_confirmation \
                        "Waiting for Docker daemon to be ready... (CTRL+C to exit script)" \
                        "docker info" \
                        "Docker daemon is now running."
                    return 0
                fi

                echo -e "Please install Docker Desktop for macOS: ${CYAN}https://www.docker.com/products/docker-desktop/${RESET}"
                echo "Once installed, open Docker Desktop to start the daemon, then return here."
                echo
                wait_for_confirmation \
                    "Waiting for Docker daemon to be ready... (CTRL+C to exit script)" \
                    "command -v docker" \
                    "Docker daemon is now running."
            fi
            ;;

        wsl)
            echo
            echo "Start Docker Desktop in Windows and ensure WSL integration is enabledn."
            wait_for_confirmation \
                "Waiting for Docker to become available via WSL integration... (CTRL+C to exit script)" \
                "docker info" \
                "Docker daemon is now running."
            ;;

        linux)

            # Debian based distros
            if command -v apt-get >/dev/null 2>&1; then
                echo -e "${CYAN}Detected apt-based distro. Installing Docker via official repository...${RESET}"
                sudo apt-get update -y
                sudo apt-get install -y ca-certificates curl gnupg lsb-release
                sudo install -m 0755 -d /etc/apt/keyrings
                local distro_id
                distro_id=$(. /etc/os-release && echo "$ID")

                # Some distros are based on debian but have their own ID
                case "$distro_id" in
                    linuxmint|pop|elementary|zorin|neon) distro_id="ubuntu" ;;
                    raspbian) distro_id="debian" ;;
                esac

                curl -fsSL "https://download.docker.com/linux/${distro_id}/gpg" \
                    | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
                sudo chmod a+r /etc/apt/keyrings/docker.gpg
                echo \
                    "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
https://download.docker.com/linux/${distro_id} \
$(. /etc/os-release && echo "${VERSION_CODENAME:-$(lsb_release -cs)}") stable" \
                    | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
                sudo apt-get update -y
                sudo apt-get install -y docker-ce docker-ce-cli containerd.io \
                    docker-buildx-plugin docker-compose-plugin
                sudo systemctl enable --now docker

            # Fedora/CentOS based distro
            elif command -v dnf >/dev/null 2>&1; then
                echo -e "${CYAN}Detected dnf-based distro. Installing Docker via official repository...${RESET}"
                sudo dnf -y install dnf-plugins-core
                local distro_id
                distro_id=$(. /etc/os-release && echo "$ID")
                case "$distro_id" in
                    fedora) dnf_add_repo_file https://download.docker.com/linux/fedora/docker-ce.repo ;;
                    *) dnf_add_repo_file https://download.docker.com/linux/centos/docker-ce.repo ;;
                esac
                sudo dnf -y install docker-ce docker-ce-cli containerd.io \
                    docker-buildx-plugin docker-compose-plugin
                sudo systemctl enable --now docker

            # Arch based distro
            elif command -v pacman >/dev/null 2>&1; then
                echo -e "${CYAN}Detected pacman-based distro. Installing Docker...${RESET}"
                sudo pacman -Sy --noconfirm docker docker-compose
                sudo systemctl enable --now docker

            # SUSE based distro
            elif command -v zypper >/dev/null 2>&1; then
                echo -e "${CYAN}Detected zypper-based distro. Installing Docker...${RESET}"
                sudo zypper install -y docker docker-compose
                sudo systemctl enable --now docker

            # Alpine based distro
            elif command -v apk >/dev/null 2>&1; then
                echo -e "${CYAN}Detected apk-based distro. Installing Docker...${RESET}"
                sudo apk add --no-cache docker docker-compose
                sudo rc-update add docker boot
                sudo service docker start

            # Nix/NixOS
            elif command -v nix-env >/dev/null 2>&1; then
                echo -e "${YELLOW}Nix/NixOS detected.${RESET}"
                echo "On NixOS, add 'virtualisation.docker.enable = true;' to your configuration.nix"
                echo "and run 'sudo nixos-rebuild switch'."
                echo
                echo "On non-NixOS with Nix, install via: nix-env -iA nixpkgs.docker"
                echo
                wait_for_confirmation \
                    "Waiting for Docker CLI to be available... (CTRL+C to exit script)" \
                    "command -v docker" \
                    "Docker CLI found."
                return 0

            else
                echo -e "${YELLOW}Unknown Linux distribution — no supported package manager detected.${RESET}"
                echo "Please install Docker manually in a separate shell:"
                echo -e "  ${CYAN}https://docs.docker.com/engine/install/${RESET}"
                echo
                wait_for_confirmation \
                    "Waiting for Docker CLI to be available... (CTRL+C to exit script)" \
                    "command -v docker" \
                    "Docker CLI found."
                return 0
            fi

            if getent group docker >/dev/null 2>&1; then
                echo -e "${CYAN}Adding $USER to the docker group...${RESET}"
                sudo usermod -aG docker "$USER"
                echo -e "${CYAN}Restarting script with docker group active...${RESET}"
                sleep 2
                exec sg docker "$0" "$@"
            fi
            ;;

        *)
            echo -e "${YELLOW}Unknown OS detected. Cannot auto-install Docker.${RESET}"
            echo -e "Please install Docker manually: ${CYAN}https://docs.docker.com/get-docker/${RESET}"
            echo
            wait_for_confirmation \
                "Waiting for Docker CLI to be available... (CTRL+C to exit script)" \
                "command -v docker" \
                "Docker CLI found."
            return 0
            ;;
    esac

    echo -e "${GREEN}Docker installed successfully.${RESET}"
}

install_vscode() {
    case "$HOST_ENV" in

        macos)
            if command -v brew >/dev/null 2>&1; then
                echo -e "${CYAN}Installing VS Code via Homebrew...${RESET}"
                brew install --cask visual-studio-code
                echo -e "${GREEN}VS Code installed.${RESET}"

             # Homebrew should be installed before if user wanted to, dont offer agian
            else
                echo -e "Please install VS Code for macOS: ${CYAN}https://code.visualstudio.com/${RESET}"
                echo "Once installed, ensure 'code' is in your PATH (Help > Shell Command: Install 'code' command), then return here."
                echo
                wait_for_confirmation \
                    "Waiting for VS Code to be available... (CTRL+C to exit script)" \
                    "command -v code" \
                    "VS Code found."
            fi
            ;;

        wsl)
            echo
                echo -e "For WSL, install VS Code on Windows: ${CYAN}https://code.visualstudio.com/${RESET}"
            echo "The 'code' command is then automatically available inside WSL."
            echo
            wait_for_confirmation \
                "Waiting for VS Code to be available... (CTRL+C to exit script)" \
                "command -v code" \
                "VS Code found."
            ;;

        linux)

            # Debian based distros
            if command -v apt-get >/dev/null 2>&1; then

                # Try snap first only on Ubuntu
                if command -v snap >/dev/null 2>&1; then
                    distro_id=$(. /etc/os-release && echo "$ID")
                    if [ "$distro_id" = "ubuntu" ]; then
                        echo -e "${CYAN}Installing VS Code via snap...${RESET}"
                        sudo snap install --classic code
                        echo -e "${GREEN}VS Code installed successfully via snap.${RESET}"
                        return 0
                    fi
                fi

                echo -e "${CYAN}Detected apt-based distro. Installing VS Code via Microsoft repository...${RESET}"
                sudo apt-get update -y
                sudo apt-get install -y wget gpg apt-transport-https
                sudo install -m 0755 -d /etc/apt/keyrings
                wget -qO- https://packages.microsoft.com/keys/microsoft.asc \
                    | gpg --dearmor \
                    | sudo tee /etc/apt/keyrings/packages.microsoft.gpg > /dev/null
                sudo chmod a+r /etc/apt/keyrings/packages.microsoft.gpg
                echo \
                    "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/packages.microsoft.gpg] \
https://packages.microsoft.com/repos/code stable main" \
                    | sudo tee /etc/apt/sources.list.d/vscode.list > /dev/null
                sudo apt-get update -y
                sudo apt-get install -y code

            # Fedora/CentOS based distros
            elif command -v dnf >/dev/null 2>&1; then
                echo -e "${CYAN}Detected dnf-based distro. Installing VS Code via Microsoft repository...${RESET}"
                sudo rpm --import https://packages.microsoft.com/keys/microsoft.asc
                echo -e "[code]\nname=Visual Studio Code\nbaseurl=https://packages.microsoft.com/yumrepos/vscode\nenabled=1\nautorefresh=1\ntype=rpm-md\ngpgcheck=1\ngpgkey=https://packages.microsoft.com/keys/microsoft.asc" | sudo tee /etc/yum.repos.d/vscode.repo > /dev/null
                sudo dnf clean all
                sudo dnf makecache
                sudo dnf -y install code

            # Arch based distros
            elif command -v pacman >/dev/null 2>&1; then
                echo -e "${CYAN}Detected pacman-based distro. Installing VS Code...${RESET}"
                if command -v yay >/dev/null 2>&1; then
                    yay -Sy --noconfirm visual-studio-code-bin
                elif command -v paru >/dev/null 2>&1; then
                    paru -Sy --noconfirm visual-studio-code-bin
                else
                    echo -e "${YELLOW}No AUR helper (yay/paru) found. The official Microsoft VS Code binary is only available via the AUR.${RESET}"
                    echo
                    echo "Note that code forks like code-oss in the main Arch repos do NOT work properly with the Microsoft Dev Containers extension."
                    echo
                    if ask_yes_no "Install yay (AUR helper) now? (y/n): "; then
                        echo
                        echo -e "${CYAN}Installing yay (AUR helper)...${RESET}"
                        install_yay
                        yay -Sy --noconfirm visual-studio-code-bin
                    else
                        echo
                        echo -e "${YELLOW}Skipping yay installation.${RESET}"
                        echo "Please install an AUR helper (e.g. yay) and then run: yay -S visual-studio-code-bin"
                        echo -e "Or download VS Code directly from: ${CYAN}https://code.visualstudio.com/${RESET}"
                        echo
                        wait_for_confirmation \
                            "Waiting for VS Code to be available... (CTRL+C to exit script)" \
                            "command -v code" \
                            "VS Code found."
                        return 0
                    fi
                fi

            # SUSE based distros
            elif command -v zypper >/dev/null 2>&1; then
                echo -e "${CYAN}Detected zypper-based distro. Installing VS Code via Microsoft repository...${RESET}"
                sudo rpm --import https://packages.microsoft.com/keys/microsoft.asc
                sudo zypper addrepo https://packages.microsoft.com/yumrepos/vscode vscode
                sudo zypper refresh
                sudo zypper install -y code

            # Nix/NixOS
            elif command -v nix-env >/dev/null 2>&1; then
                echo -e "${CYAN}Detected Nix. Installing VS Code...${RESET}"
                nix-env -iA nixpkgs.vscode

            else
                echo -e "${YELLOW}Unknown Linux distribution — no supported package manager detected.${RESET}"
                echo -e "Please install VS Code manually: ${CYAN}https://code.visualstudio.com/${RESET}"
                echo
                wait_for_confirmation \
                    "Waiting for VS Code to be available... (CTRL+C to exit script)" \
                    "command -v code" \
                    "VS Code found."
                return 0
            fi
            ;;

        *)
            echo -e "${YELLOW}Unknown OS detected. Cannot auto-install VS Code.${RESET}"
            echo "Please install VS Code manually: https://code.visualstudio.com/"
            echo
            wait_for_confirmation \
                "Waiting for VS Code to be available... (CTRL+C to exit script)" \
                "command -v code" \
                "VS Code found."
            return 0
            ;;
    esac

    echo -e "${GREEN}VS Code installed successfully.${RESET}"
}


# ============= MAIN SCRIPT


print_header
HOST_ENV=$(detect_host_env)
[ "$HOST_ENV" = "unknown" ] && echo -e "${YELLOW}Warning: Unknown OS detected${RESET}" && sleep 2

if [ "$(id -u)" -eq 0 ]; then
    echo
    echo -e "${RED}Don't run this script as root or with sudo.${RESET}"
    exit 1
fi

echo -e "${BOLD}${WHITE}========================= Checking Dependencies ==========================${RESET}"

echo
echo -e "${WHITE}---------------------------- Checking Docker -----------------------------${RESET}"
echo

if ! command -v docker >/dev/null 2>&1; then
    echo -e "${YELLOW}Docker CLI not found.${RESET}"
    echo "Docker is required for this project..."
    echo
    case "$HOST_ENV" in
        wsl)
            echo -e "${YELLOW}WSL environment detected.${RESET}"
            echo "We strongly recommend using the Windows setup script (setup.bat) for Windows users instead."
            echo
            if ask_yes_no "Do you want to continue with Docker setup in WSL? (y/n): "; then
                install_docker
            else
                echo "Please run the Windows setup script from PowerShell."
                exit 1
            fi
            ;;
        linux|macos)
            if ask_yes_no "Do you want to install Docker now? (y/n): "; then
                install_docker
            else
                echo "Please install Docker manually: https://docs.docker.com/get-docker/"
                echo "The script will wait until Docker CLI is available to continue."
                echo
                wait_for_confirmation \
                    "Waiting for Docker CLI to be available... (CTRL+C to exit script)" \
                    "command -v docker" \
                    "Docker CLI found."
            fi
            ;;
        *)
            install_docker
            ;;
    esac

else
    echo -e "${CYAN}Docker found. Checking if Docker daemon is running...${RESET}"
    if ! timeout 2 docker info >/dev/null 2>&1; then
        echo -e "${YELLOW}Docker daemon does not appear to be running. Attempting to start it...${RESET}"
        echo
        case "$HOST_ENV" in
            linux)
                sudo systemctl start docker
                ;;
            *)
                echo "Please start Docker-Desktop from your applications."
                echo "The script will wait until Docker engine is ready to continue."
                echo
                wait_for_confirmation "Waiting for Docker engine to be ready... (CTRL+C to exit script)" "timeout 2 docker info" "Docker daemon is now running."
                ;;
        esac

    else
        echo -e "${GREEN}Docker daemon is running.${RESET}"
    fi
fi


echo
echo -e "${WHITE}---------------------------- Checking VS-Code ----------------------------${RESET}"
echo

if ! command -v code >/dev/null 2>&1; then
    echo -e "${YELLOW}VS Code not found.${RESET}"
    echo "VS Code is required for this project..."
    echo
    case "$HOST_ENV" in
        linux|macos)
            if ask_yes_no "Do you want to install VS Code now? (y/n): "; then
                install_vscode
            else
                echo "Please install VS Code manually: https://code.visualstudio.com/"
                echo "The script will wait until VS Code is available to continue."
                echo
                wait_for_confirmation \
                    "Waiting for VS Code to be available... (CTRL+C to exit script)" \
                    "command -v code" \
                    "VS Code found."
            fi
            ;;
        *)
            install_vscode
            ;;
    esac
else
    echo -e "${GREEN}VS Code found${RESET}"
fi

echo
echo -e "${CYAN}Checking Dev Containers extension...${RESET}"
if code --list-extensions 2>/dev/null | grep -q "ms-vscode-remote.remote-containers"; then
    echo -e "${GREEN}Dev Containers extension is already installed${RESET}"
else
    echo -e "${YELLOW}Dev Containers extension not found.${RESET}"
    if ask_yes_no "Install Dev Containers extension now? (y/n): "; then
        echo "Installing Dev Containers extension..."
        if code --install-extension ms-vscode-remote.remote-containers; then
            echo -e "${GREEN}Dev Containers extension installed successfully${RESET}"
        else
            echo -e "${RED}Failed to install Dev Containers extension. Please install manually.${RESET}"
        fi
    else
        echo -e "${YELLOW}Warning: Dev Containers extension is required for this project.${RESET}"
        echo "   You can install it later from VS Code extensions marketplace."
    fi
fi


echo
echo -e "${BOLD}${WHITE}============================ Setting up Git  =============================${RESET}"
echo

TARGET_REPO="riserobotics/rise-motion"
TARGET_DIR="rise-motion"
TOKEN_VAR="RISE_MOTION_ACCESS_TOKEN"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

USER_SHELL=$(basename "$(getent passwd "$USER" | cut -d: -f7)")
SUPPORTED_SHELLS=("bash" "zsh" "fish" "sh" "ksh" "csh" "tcsh")
if [[ ! " ${SUPPORTED_SHELLS[@]} " =~ " ${USER_SHELL} " ]]; then
    echo "Detected shell: $USER_SHELL"
    echo
    echo -e "${YELLOW}Warning: Detected shell '$USER_SHELL' may not be fully supported by this setup script.${RESET}"
    echo "You may need to manually set the RISE_MOTION_ACCESS_TOKEN environment variable in your shell configuration file."
    echo
fi
RC=$(get_shell_rc)

GIT_REMOTE=$(git config --get remote.origin.url 2>/dev/null || echo "")
IN_TARGET_REPO=false

if git rev-parse --git-dir > /dev/null 2>&1 && [ "$SCRIPT_DIR" = "$(git rev-parse --show-toplevel)" ]; then
    if [[ "$GIT_REMOTE" == *"$TARGET_REPO"* ]]; then
        IN_TARGET_REPO=true
        echo -e "${GREEN}rise-motion repository already cloned.${RESET}"
    fi
fi


if [ "$IN_TARGET_REPO" = true ]; then

    echo -e "${CYAN}Checking Git remote configuration...${RESET}"

    if [[ "$GIT_REMOTE" =~ ^https://[^@]+@github\.com/ ]]; then
        echo
        echo "Repository remote uses an embedded access token for HTTPS."
    else
        echo
        echo -e "${YELLOW}WARNING: Repository remote is NOT configured with an embedded HTTPS access token.${RESET}"
        echo "Dev container environments require token-based HTTPS remotes (https://<token>@github.com/owner/repo.git)."
        echo "Current remote URL: $GIT_REMOTE"
        echo
        echo "Please update the remote to use an embedded personal access token, or re-clone the repository using such a URL."
        echo
        if ask_yes_no "Update the remote URL to HTTPS with a personal access token now? (y/n): "; then
            setup_token "$RC" "$TOKEN_VAR"

            if [ -z "${!TOKEN_VAR}" ]; then
                echo -e "${RED}ERROR: Access token required. Aborting.${RESET}"
                exit 1
            fi

            NEW_URL="https://${!TOKEN_VAR}@github.com/${TARGET_REPO}.git"

            echo "Updating remote URL to token-based HTTPS URL: https://***@github.com/${TARGET_REPO}.git"
            git remote set-url origin "$NEW_URL"
            GIT_REMOTE="$NEW_URL"
            echo -e "${GREEN}Remote URL updated successfully!${RESET}"
            echo

            if [ -n "${!TOKEN_VAR}" ]; then
                save_token_to_rc "$RC" "$TOKEN_VAR"
            fi

        else
            echo -e "${YELLOW}Continuing without updating remote.${RESET}"
            echo -e "${YELLOW}Be aware that Git operations inside the dev container will fail due to lack of authentication.${RESET}"
            echo "Git operations must be performed from the host environment, outside the dev container."
        fi
    fi
else
    echo -e "${YELLOW}[Warning] This script was not run within the ${TARGET_REPO} repository.${RESET}"
    echo
    if ! ask_yes_no "Do you want to clone the ${TARGET_REPO} repository? (y/n): "; then
        echo -e "${RED}Setup cancelled. Please run this script from within the ${TARGET_REPO} repository.${RESET}"
        exit 1
    fi

    setup_token "$RC" "$TOKEN_VAR"

    if [ -z "${!TOKEN_VAR}" ]; then
        echo -e "${RED}ERROR: GitHub access token is required to clone the repository.${RESET}"
        echo -e "${RED}Setup cancelled.${RESET}"
        exit 1
    fi

    echo
    read -e -p "Enter the directory where you want to clone the repository to [$(pwd)]: " CLONE_DIR
    CLONE_DIR=${CLONE_DIR:-$(pwd)}

    echo
    git clone --recurse-submodules \
        "https://${!TOKEN_VAR}@github.com/${TARGET_REPO}.git" \
        "$CLONE_DIR/$TARGET_DIR"

    echo
    echo -e "${GREEN}Repository cloned successfully!${RESET}"

    if [ -n "${!TOKEN_VAR}" ]; then
        save_token_to_rc "$RC" "$TOKEN_VAR"
    fi
fi

echo
echo -e "${GREEN}GitHub access token setup complete!${RESET}"

echo
echo -e "${BOLD}${WHITE}===================== Setting up Docker environment ======================${RESET}"
echo

echo -e "${CYAN}Pulling RISE-OS Docker image from GitHub Container Registry...${RESET}"
echo "Image: ghcr.io/riserobotics/rise-os:latest"
echo

if ! sudo docker pull ghcr.io/riserobotics/rise-os:latest; then
    echo
    echo -e "${RED}Failed to pull Docker image.${RESET}"
    echo "Please check your network connection."
    exit 1
else
    echo
    echo -e "${GREEN}Successfully pulled RISE-OS Docker image${RESET}"
fi

echo
echo -e "${BOLD}${WHITE}============================= Setup Complete! ============================${RESET}"
echo
echo "You can now use the RISE-OS development environment."
echo "Open the repository in VS Code and press F1, then select 'Dev Containers: Reopen and rebuild in Container'"
echo
echo "Notes:"
echo "  - All new/edited code MUST be able to compile in the dev container environment."
echo "  - New installations, packages etc. are NOT persistent when rebuilding the dev container."
echo "  - If you need new packages/tools, create a new branch in: https://github.com/riserobotics/rise-docker-ros2-container/resources"
echo "    Add your new requirements to the requirement files and create a pull request."
echo "    Make sure all new requirements are available for both ARM64 and AMD64 architectures!"
echo "    The 'build_docker.sh' MUST succeed for both architectures before committing."

if [ "$IN_TARGET_REPO" = true ]; then
    REPO_DIR="$SCRIPT_DIR"
else
    REPO_DIR="$CLONE_DIR/$TARGET_DIR"
fi

echo
if ask_yes_no "Open the repository in VS Code now? (y/n): "; then
    code "$REPO_DIR"
fi
