#!/bin/sh

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}"  )" &> /dev/null && pwd  )"
SCRIPT_PATH="${SCRIPT_DIR}/build_env.sh"
VENV_DIR="${SCRIPT_DIR}/venv"

false=0
true=1

venv_ready=$false

while [ $venv_ready == $false ]
do
    if [ -d ${VENV_DIR} ]
    then
        read -p "A virtual environment already exists, should I delete it and start over? [y/N]" refresh
        if [ "$refresh" == 'y' ] || [ "$refresh" == 'Y' ]
        then
            echo 'Deleting current virtual environment...'
            rm -r ${VENV_DIR}
            echo 'Creating a new virtual environment\n'
            python3 -m virtualenv ${SCRIPT_DIR}/venv
            venv_ready=$true
        elif [ "$refresh" == 'n' ] || [ "$refresh" == 'N' ] || [ "$refresh" == '' ]
        then
            echo 'Continuing with current virtual environment'
            venv_ready=$true
        else
            echo 'Enter a valid option [y/N]'
        fi

    else
        echo 'Creating a virtual environment\n'
        python3 -m virtualenv ${SCRIPT_DIR}/venv
        venv_ready=$true
    fi
done

source "${VENV_DIR}/bin/activate"

REQUIREMENTS_PATH="${SCRIPT_DIR}/requirements.txt"
REQUIREMENTS_DEV_PATH="${SCRIPT_DIR}/requirements_dev.txt"

if [ -e $REQUIREMENTS_PATH ]
then
    echo 'Installing requirements'
    pip install -r ${REQUIREMENTS_PATH}
fi

if [ -e $REQUIREMENTS_DEV_PATH ]
then
    echo 'Installing development requirements'
    pip install -r ${REQUIREMENTS_DEV_PATH}
fi

SETUP_PATH="${SCRIPT_DIR}/setup.py"

if [ -e  ${SETUP_PATH} ]
then
    echo 'Installing repo package'
    pip install -e ${SCRIPT_DIR}
fi

echo 'Done!'
