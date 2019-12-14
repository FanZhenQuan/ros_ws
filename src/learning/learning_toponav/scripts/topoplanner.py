#!/usr/bin/env python

import rospy
import argparse
import networkx as nx


class Planner(object):
    def __init__(self, edgelist):
        self.graph = nx.read_adjlist(edgelist, delimiter=', ', nodetype=str)
        
    def find_path(self, source, dest):
        return nx.dijkstra_path(self.graph, source, dest)

    
def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--edgelist', type=str, default=None, help='Path to edgelist')
    parser.add_argument('--source', type=str, default=None, help='Source of planning')
    parser.add_argument('--dest', type=str, default=None, help='Dest of planning')
    args, unknown = parser.parse_known_args()
    
    return args


if __name__ == '__main__':
    rospy.init_node('topoplanner')
    
    args = parse_args()
    
    source = 'WayPoint' + args.source
    dest = 'WayPoint' + args.dest
    
    planner = Planner(args.edgelist)
    path = planner.find_path(source, dest)
    
    rospy.loginfo(str(path))
    
    rospy.spin()