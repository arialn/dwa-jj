#!/bin/bash


#jv_list=(0.01 0.05 0.10)
#jw_list=(0.01 0.1 0.2 0.3)

jv_list=(0.1)
jw_list=(0.1)
for jv in "${jv_list[@]}"
do
    for jw in "${jw_list[@]}"
    do
        for i in {1..10}
        do
            echo "$i"	
            gnome-terminal -- ./environment.sh
            wait
            python dwa_diff0530.py --jv "$jv" --jw "$jw"
            wait
            killall gnome-terminal-server
            sleep 5s
        done	
    done	
done
