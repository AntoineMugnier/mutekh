#!/bin/bash

# Load ruby environment
. $HOME/.rvm/scripts/rvm

here=$(dirname `readlink -f $0`)
$here/scripts/cleanup.rb
