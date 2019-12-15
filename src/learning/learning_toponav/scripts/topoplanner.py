#!/usr/bin/env python

import rospy
import argparse
import networkx as nx

from toponodes_publisher import INTEREST_POINTS
from topological_node import TopologicalNode


class Planner(object):
    def __init__(self, edgelist):
        self.graph = nx.read_adjlist(edgelist, delimiter=', ', nodetype=str)
        
        self.nodes = rospy.get_param(INTEREST_POINTS)
        
    def _find_node_by_name(self, name):
        for n in self.nodes:
            if n['name'] == name:
                return n
            
    def _find_pos_by_nodename(self, name):
        for n in self.nodes:
            if n['name'] == name:
                return n['pose']['position']
            
    def _find_verts_by_nodename(self, name):
        for n in self.nodes:
            if n['name'] == name:
                return n['verts']
        
    def find_path(self, source, dest):
        """
        :param source: source of planning (type: str, indicates a WayPoint)
        :param dest: destination of planning (type: str, indicates a WayPoint)
        :return: list of WayPoints (type:str) to traverse in order to reach the goal
        """
        return nx.dijkstra_path(self.graph, source, dest)
    
    # def

    
def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--edgelist', type=str, required=True)
    parser.add_argument('--source', type=str, required=False)
    parser.add_argument('--dest', type=str, required=False)
    args, unknown = parser.parse_known_args()
    
    return args


if __name__ == '__main__':
    rospy.init_node('topoplanner')
    
    args = parse_args()
    
    planner = Planner(args.edgelist)
    
    if args.source and args.dest:
        source = 'WayPoint' + args.source
        dest = 'WayPoint' + args.dest
        
        path = planner.find_path(source, dest)
        rospy.loginfo(str(path))
    else:
        rospy.logwarn('Planner: no source and destination provided at launch')
        pass
    
    rospy.spin()