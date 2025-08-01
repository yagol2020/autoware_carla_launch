#!/usr/bin/env bash
set -e

function install_python()
{
    # Install poetry
    if [ -f /.dockerenv ]; then
        # Create folder for poetry installation
        mkdir -p ${POETRY_HOME}
    fi
    curl -sSL https://install.python-poetry.org | python3 -
    poetry config virtualenvs.in-project true

    # Install python3.8.10
    # Check if installed python3.8.*
    version=$(python -V 2>&1)

    if [[ "$version" =~ ^Python\ 3\.8\. ]]; then
        echo "Python 3.8.x is installed: $version"
    else
        echo "Python 3.8.x is NOT installed (found: $version)"
        curl -L https://raw.githubusercontent.com/pyenv/pyenv-installer/master/bin/pyenv-installer | bash
        pyenv install -v 3.8.10
        pyenv global 3.8.10
    fi
    cd ${AUTOWARE_CARLA_ROOT}/external/zenoh_carla_bridge/carla_agent && poetry env use $(which python)
    cd ${AUTOWARE_CARLA_ROOT}/external/zenoh_autoware_v2x && poetry env use $(which python) && poetry install --no-root
}

function install_rust()
{
    # Install RUST
    if [ -f /.dockerenv ]; then
        # Create folder for RUST installation
        mkdir -p ${RUSTUP_HOME}
    fi
    curl https://sh.rustup.rs -sSf | bash -s -- -y
}

if [ "$1" = "rust" ]; then
    install_rust
elif [ "$1" = "python" ]; then
    install_python
else
    install_rust
    install_python
fi

