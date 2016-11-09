from collections import defaultdict


class Graph(object):
    def __init__(self):
        self._graph = {}

    def add(self, node1, node2, info):
        if node1 not in self._graph:
            self._graph[node1] = {}

        self._graph[node1][node2] = info

    def is_connected(self, node1, node2):
        return node1 in self._graph and node2 in self._graph[node1]

    def get_info(self, node1, node2):
        return self._graph[node1][node2]

    def find_path(self, node1, node2, direction, path=[]):
        # currently not the shortest path..
        
        path = path + [node1]
        if node1 == node2:
            return path
        if node1 not in self._graph:
            return None
        for node in self._graph[node1]:
            if node not in path and self._graph[node1][node]['direction'] == direction:
                new_path = self.find_path(node, node2, direction, path)
                if new_path:
                    return new_path
        return None

    def __str__(self):
        return '{}({})'.format(self.__class__.__name__, dict(self._graph))
