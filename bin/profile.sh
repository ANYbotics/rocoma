#!/bin/bash
source ~/.bashrc
CPUPROFILE=/tmp/locomotion_controller_node.out LD_PRELOAD=/usr/lib/libprofiler.so.0 CPUPROFILE_FREQUENCY=800 ./devel/lib/locomotion_controller/locomotion_controller_node
google-pprof --pdf ./devel/lib/locomotion_controller/locomotion_controller_node /tmp/locomotion_controller_node.out > /tmp/locomotion_controller_node.pdf
okular /tmp/locomotion_controller_node.pdf &