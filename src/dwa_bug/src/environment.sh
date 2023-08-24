#!/bin/bash

#conda init bash
source activate sim2
#sleep 2s
wait
cd
wait
#sleep 2s
source /home/z_lin/mapless/devel/setup.bash
wait
#sleep 2s
roslaunch dwa_bug static_dwa_bug.launch

