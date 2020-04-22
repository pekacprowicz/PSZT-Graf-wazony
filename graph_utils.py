import pandas as pd


class Heuristics:

    @staticmethod
    def first_best(current, s):
        pass

class Graph:

    def __init__(self, file_graph_definition):
        self.vertices_connection = dict()
        self.edges_dict = dict()
        self.construct_graph(file_graph_definition)

    def construct_graph(self, definition):
        input_data = pd.read_csv(definition, sep=',')
        for edge_ in input_data.iterrows():
            edge = edge_[1]
            self.add_update_vertex(edge[0], edge[1])
            self.add_update_vertex(edge[1], edge[0])
            self.edges_dict[f"{edge[0]}-{edge[1]}"] = edge[2]

    def get_edge_weight(self, start, end):
        try:
            weight = self.edges_dict[f"{start}-{end}"]
        except KeyError:
            try:
                weight = self.edges_dict[f"{end}-{start}"]
            except KeyError:
                weight = float('inf')
        return weight

    def get_path_weight(self, path):
        weight = 0
        if len(path) != 1:
            for i in range(0, len(path)-2, 2):
                start = path[i]
                end = path[i+2]
                weight += self.get_edge_weight(start, end)

        return weight

    def add_update_vertex(self, vertex_id, connected_vertex):
        try:
            self.vertices_connection[vertex_id].add(connected_vertex)
        except KeyError:
            self.vertices_connection[vertex_id] = set()
            self.vertices_connection[vertex_id].add(connected_vertex)

    def get_shortest_path(self, begin, end, method="heuristic"):
        if method == "heuristic":
            return self.heuristic_search(begin, end)

    @staticmethod
    def check_for_terminal_path(paths, end):
        result = False
        for path in paths.keys():
            if end in path:
                result = True
        return result

    def get_terminal_paths(self, paths, end):
        terminal_paths = dict()
        for path in paths.keys():
            if end in path:
                terminal_paths.update({path: self.get_path_weight(path)})
        return terminal_paths

    def heuristic_search(self, begin, end, heuristic_function="first_best", **kwargs):
        paths = dict()
        paths[begin] = 0
        terminal_paths = dict()
        extended_paths = dict()
        while True:  # not self.check_for_terminal_path(extended_paths, end):
            path_to_extend, minimal_cost = self.choose_next_path(paths)
            extended_paths = self.extend_paths(path_to_extend, heuristic_function)
            paths.update(extended_paths)
            del paths[path_to_extend]
            terminal_paths.update(self.get_terminal_paths(extended_paths, end))
            if terminal_paths and min(terminal_paths.values()) <= min(paths.values()):
                return min(terminal_paths, key=terminal_paths.get), min(terminal_paths.values())



    def get_outgoing_edges(self, vertex):
        edges = dict()
        for connected_vertex in self.vertices_connection[vertex]:
            edges[f"{connected_vertex}"] = self.get_edge_weight(vertex, connected_vertex)
        return edges

    @staticmethod
    def choose_next_path(paths):
        return min(paths, key=paths.get), min(paths.values())

    def extend_paths(self, path_to_extend, heuristic_function):
        extended_paths = dict()
        existing_path_cost = self.get_path_weight(path_to_extend)
        last_vertex = path_to_extend[-1]
        outgoing_vertices = self.vertices_connection[last_vertex]
        for vertex in outgoing_vertices:
            if vertex in path_to_extend:
                continue
            extended_paths[f"{path_to_extend}-{vertex}"] = existing_path_cost + self.get_heuristic_value(f"{path_to_extend}-{vertex}", heuristic_function)
        return extended_paths

    def get_heuristic_value(self, path, heuristic_function):
        if heuristic_function == "some":
            pass
        elif heuristic_function == "first_best":
            return self.get_edge_weight(path[-1], path[-3])
