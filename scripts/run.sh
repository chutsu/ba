#!/bin/bash
set -e
# ctags -R .

# make clean
# make ba
# make run
# make runc
make unittest

# cd ba; octave-cli sim_data.m; cd -
# cd bin; octave-cli sim_plot.m; cd -
