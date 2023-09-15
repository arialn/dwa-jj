#!/bin/bash


#jv_list=(0.01 0.05 0.10)
#jw_list=(0.01 0.1 0.2 0.3)

jv_list=(-1.0 -0.5 0.00001 0.5 1.0)
#jw_list=(tangent secant)
jw_list=(s m e sm me se sme)
for jv in "${jv_list[@]}"
do
    for jw in "${jw_list[@]}"
    do
        #python dwa_static.py --av "$jv" --model "$jw"
        python dwa_static.py --av "$jv" --pm "$jw"
        wait
        sleep 2s
    done	
done
