#!/usr/bin/zsh

function extensionInstallation {
    echo -e "RMCS: start vscode extension installation."
    code --install-extension streetsidesoftware.code-spell-checker
    code --install-extension llvm-vs-code-extensions.vscode-clangd
    code --install-extension twxs.cmake
    echo -e "RMCS: finish vscode extension installation."
}

# @todo(creeper) collect words
function codeSpellCheckSetup {
    echo "code spell check setup."
}

function clangdInstallation {
    echo -e "RMCS: start clangd installation."

    if [[ -f /etc/apt/sources.list.d/llvm-apt.list ]]; then
        echo -e "RMCS: \033[32m<llvm-apt.list>\033[0m exits, if you want to update this file," \
                "try \033[31m\"sudo rm /etc/apt/sources.list.d/llvm-apt.list\"\033[0m"
    else
        sudo apt-get install -y gnupg wget
        wget -O /tmp/llvm-snapshot.gpg.key https://apt.llvm.org/llvm-snapshot.gpg.key
        sudo apt-key add /tmp/llvm-snapshot.gpg.key
        rm /tmp/llvm-snapshot.gpg.key

        sudo touch /etc/apt/sources.list.d/llvm-apt.list
        sudo echo "deb https://mirrors.tuna.tsinghua.edu.cn/llvm-apt/jammy/ llvm-toolchain-jammy main" |\
        sudo tee -a  /etc/apt/sources.list.d/llvm-apt.list
        sudo apt-get update -y
    fi

    version=`apt-cache search clangd- | grep clangd- | awk -F' ' '{print $1}' | sort -V | tail -1 | cut -d- -f2`
    if command -v clangd-$version > /dev/null 2>&1; then
        echo -e "RMCS:\033[32m clangd-$version\033[0m and\033[32m clang-format-$version\033[0m prepared." 
    else
        sudo apt-get install -y clangd-$version clang-format-$version
    fi

    if [[ -f /usr/bin/clangd ]]; then
        echo -e "RMCS: \033[32m/usr/bin/clangd\033[0m exits, give up linking the clangd-20 to clangd."
    else
        sudo ln -s /usr/bin/clangd-$version /usr/bin/clangd
        sudo ln -s /usr/bin/clang-format-$version /usr/bin/clang-format
    fi

    echo -e "RMCS: finish clangd installation."
}

function rmcsDependencyInstallation {
    echo -e "RMCS: start rmcs installation."
    cd /workspaces/rmcs/rmcs_ws/
    sudo rosdep install --from-paths src --ignore-src -r -y
    echo -e "RMCS: finish rmcs installation."
}

sudo apt-get update -y
clangdInstallation
extensionInstallation
rmcsDependencyInstallation