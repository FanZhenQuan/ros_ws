#!/usr/bin/env bash

echo '------------'
echo 'Mappa .yaml e .pgm devono essere sulla Scrivania'
echo '------------'
echo ''
python /home/davide/ros_ws/src/libs/spqrel_navigation/src/topological_navigation/scripts/map_editor.py --empty true --map /home/davide/Scrivania/mymap.yaml --outmap /home/davide/Scrivania/topomap.tpg