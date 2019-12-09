#!/usr/bin/env bash

echo '------------'
echo 'Mappa .yaml e .pgm devono essere sulla Scrivania'
echo '------------'
echo ''

echo 'Press (a) to add node
Press (d) to remove node
Press (e) to add edge between to nodes (click origin first and then destination)
Press (h) for this Menu
Press (m) to move a node
Press (w) to write map to outmap'

python /home/davide/ros_ws/src/libs/spqrel_navigation/src/topological_navigation/scripts/map_editor.py --empty true --map /home/davide/Scrivania/mymap.yaml --outmap /home/davide/Scrivania/topomap.tpg