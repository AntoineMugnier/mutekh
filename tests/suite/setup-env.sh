#!/bin/bash

export PATH

if [ -f $HOME/soclib/env.sh ]; then
    . $HOME/soclib/env.sh
fi

PATH=$PATH:/opt/mutekh/bin:$HOME/bin

# Load Ruby environment
if [ -f $HOME/.rvm/scripts/rvm ]; then
    . $HOME/.rvm/scripts/rvm
else
    echo "ERROR: missing rvm installation!"
    echo "       Please install RVM with a valid Ruby release."
fi
