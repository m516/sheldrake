#!/bin/sh
# A script to automatically install libraries if needed

# Warn if not on supported 
if [  -n "$(uname -a | grep Ubuntu)" ]; then
    sudo apt-get update && sudo apt-get upgrade 
else

# A function to install Drake
install_drake () { 
    sudo apt-get update
    sudo apt-get install --no-install-recommends ca-certificates gnupg lsb-release wget
    wget -qO- https://drake-apt.csail.mit.edu/drake.asc | gpg --dearmor - | sudo tee /etc/apt/trusted.gpg.d/drake.gpg
    echo "deb [arch=amd64] https://drake-apt.csail.mit.edu/$(lsb_release -cs) $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/drake.list
    sudo apt-get update
    sudo apt-get install --no-install-recommends drake-dev
}

use_drake () {
    export PATH="/opt/drake/bin${PATH:+:${PATH}}"
    export PYTHONPATH="/opt/drake/lib/python$(python3 -c 'import sys; print("{0}.{1}".format(*sys.version_info))')/site-packages${PYTHONPATH:+:${PYTHONPATH}}"
}

if dpkg -s drake-dev > /dev/null; then
    echo "Drake installed"
else
    echo "Drake not found!"
    echo "I can install the package by following the recommended installation instructions"
    echo "These instructions can be found at: https://drake.mit.edu/apt.html#stable-releases"
    echo "Auto-install with default settings? (y/n) \c"
    read REPLY
    if [ $REPLY = "y" ]; then
        install_drake
    fi
fi