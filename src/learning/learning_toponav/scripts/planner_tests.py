#!/usr/bin/env python

import rospy


class Graph(dict):
    def add_values(self, ipoints):
        for i in ipoints:
            key = i['name']
            edges = [e['node'] for e in i['edges']]
            self.__setitem__(key, edges)
    
    def bfs(self, start, goal):
        # keep track of all visited nodes
        explored = []
        # keep track of nodes to be checked
        queue = [start]
        
        # keep looping until there are nodes still to be checked
        while queue:
            # pop shallowest node (first node) from queue
            node = queue.pop(0)
            if node == goal:
                return explored
            if node not in explored:
                # add node to list of checked nodes
                explored.append(node)
                neighbours = self.__getitem__(node)
                
                # add neighbours of node to queue
                for neighbour in neighbours:
                    queue.append(neighbour)
        # return explored
    
    def __setitem__(self, key, item):
        self.__dict__[key] = item
    
    def __getitem__(self, key):
        return self.__dict__[key]
    
    def __repr__(self):
        return repr(self.__dict__)
    
    def __len__(self):
        return len(self.__dict__)
    
    def __delitem__(self, key):
        del self.__dict__[key]
    
    def clear(self):
        return self.__dict__.clear()
    
    def copy(self):
        return self.__dict__.copy()
    
    def has_key(self, k):
        return k in self.__dict__
    
    def update(self, *args, **kwargs):
        return self.__dict__.update(*args, **kwargs)
    
    def keys(self):
        return self.__dict__.keys()
    
    def values(self):
        return self.__dict__.values()
    
    def items(self):
        return self.__dict__.items()
    
    def pop(self, *args):
        return self.__dict__.pop(*args)
    
    def __cmp__(self, dict_):
        return self.__cmp__(self.__dict__, dict_)
    
    def __contains__(self, item):
        return item in self.__dict__
    
    def __iter__(self):
        return iter(self.__dict__)
    
    def __unicode__(self):
        return unicode(repr(self.__dict__))


class Graph2(object):
    def __init__(self, ipoints):
        self.graph = ipoints
    
    def get_node_by_name(self, name):
        for n in self.graph:
            if n['name'] == name:
                return n
        
        return None
    
    def bfs(self, start_name, goal_name):
        # keep track of visited nodes
        explored = []
        # keep track of nodes to be checked
        queue = [start_name]
        
        if start_name == goal_name:
            return []
        
        # keep looping until there are nodes still to be checked
        while queue:
            # pop first node from queue
            node = self.get_node_by_name(queue.pop(0))
            if node['name'] == goal_name:
                explored.append(goal_name)
                return explored
            if node['name'] not in explored:
                # add node to list of checked nodes
                # except the start node
                if node['name'] != start_name:
                    explored.append(node['name'])
                
                # add neighbours of node to queue (to visit)
                neighbours = node['edges']
                for neighbour in neighbours:
                    queue.append(neighbour['node'])


def ipoints_to_json(ipoints, path):
    dump = json.dumps(ipoints, sort_keys=True, indent=2)
    
    with open(path, 'w') as f:
        f.write(dump)


def graph2():
    ipoints = rospy.get_param(INTEREST_POINTS)
    g = Graph2(ipoints)
    
    if len(sys.argv) < 3:
        rospy.logerr('Fornire source e dest via argv')
        rospy.signal_shutdown('Fornire source e dest via argv')
    
    source = 'WayPoint' + sys.argv[1]
    dest = 'WayPoint' + sys.argv[2]
    
    path = g.bfs(source, dest)
    print path


if __name__ == '__main__':
    rospy.init_node('')