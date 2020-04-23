import pandas as pd

# todo no path leading to end vertex
# todo vertice names not only letters
# todo no need to keep all terminal paths in get_shortest_path, only the shortest one is needed


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

    #
    def add_update_vertex(self, vertex_id, connected_vertex):
        try:
            self.vertices_connection[vertex_id].add(connected_vertex)
        except KeyError:
            self.vertices_connection[vertex_id] = set()
            self.vertices_connection[vertex_id].add(connected_vertex)

    # returns edge cost
    def get_edge_weight(self, start, end):
        try:
            weight = self.edges_dict[f"{start}-{end}"]
        except KeyError:
            try:
                weight = self.edges_dict[f"{end}-{start}"]
            except KeyError:
                weight = float('inf')
        return weight

    # finds paths that lead to end vertex
    @staticmethod
    def get_terminal_paths(paths, end):
        terminal_paths = dict()
        for path in paths.keys():
            if end in path:
                terminal_paths.update({path: paths[path]})
        return terminal_paths

    # finds shortest path based on chosen algorithm
    def get_shortest_path(self, start, end, algorithm="brute_force"):
        paths = dict()
        paths[start] = 0
        terminal_paths = dict()
        while True:
            path_to_extend = self.choose_next_path(paths, algorithm)
            extended_paths = self.extend_path(path_to_extend, paths[path_to_extend])
            paths.update(extended_paths)
            del paths[path_to_extend]
            terminal_paths.update(self.get_terminal_paths(extended_paths, end))
            if terminal_paths and min(terminal_paths.values()) <= min(paths.values()):
                shortest_path = min(terminal_paths, key=terminal_paths.get)
                return shortest_path, terminal_paths[shortest_path]

    # chooses path to be extended next based on chosen algorithm
    @staticmethod
    def choose_next_path(paths, algorithm):
        if algorithm == "brute_force":  # returns oldest path
            return next(iter(paths))
        elif algorithm == "first_best":  # returns path with min value
            return min(paths, key=paths.get)
        elif algorithm == "a_star":  # uses heuristic function to evaluate next best path todo implement
            return next(iter(paths))

    # extends path to all not visited outgoing vertices
    def extend_path(self, path_to_extend, path_to_extend_cost):
        extended_paths = dict()
        last_vertex = path_to_extend[-1] # todo
        outgoing_vertices = self.vertices_connection[last_vertex]
        for next_vertex in outgoing_vertices:
            if next_vertex in path_to_extend:
                continue
            extended_paths[f"{path_to_extend}-{next_vertex}"] \
                = path_to_extend_cost + self.get_edge_weight(last_vertex, next_vertex)
        return extended_paths
