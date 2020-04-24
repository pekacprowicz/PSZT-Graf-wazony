import pandas as pd
import statistics

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
            self.add_or_update_vertex_neighbours(edge[0], edge[1])
            self.add_or_update_vertex_neighbours(edge[1], edge[0])
            self.edges_dict[f"{edge[0]}-{edge[1]}"] = edge[2]

    def add_or_update_vertex_neighbours(self, vertex_id, connected_vertex):
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

    @staticmethod
    def is_terminal_path(path, end):
        if end in path:
            return True
        else:
            return False

    # finds paths that lead to end vertex
    @staticmethod
    def get_terminal_paths(paths, end):
        terminal_paths = dict()
        for path in paths.keys():
            if Graph.is_terminal_path(path, end):
                terminal_paths.update({path: paths[path]})
        return terminal_paths

    # finds shortest path based on chosen algorithm
    def get_shortest_path(self, start, end, algorithm="a_star"):
        paths = dict()
        paths[tuple([start])] = 0
        terminal_paths = dict()
        while paths:
            path_to_extend = self.choose_next_path(paths, algorithm, end)
            extended_paths = self.extend_path(path_to_extend, paths[path_to_extend])
            paths.update(extended_paths)
            del paths[path_to_extend]   # todo terminal paths should be removed from paths to extend
            terminal_paths.update(self.get_terminal_paths(extended_paths, end))
            if terminal_paths and min(terminal_paths.values()) <= min(paths.values()):
                shortest_path = min(terminal_paths, key=terminal_paths.get)
                return shortest_path, terminal_paths[shortest_path]

    # chooses path to be extended next based on chosen algorithm
    def choose_next_path(self, paths, algorithm, end):
        if algorithm == "brute_force":  # returns oldest path
            return next(iter(paths))
        elif algorithm == "first_best":  # returns path with min value
            return min(paths, key=paths.get)
        elif algorithm == "a_star":  # uses heuristic function to evaluate next best path todo implement
            return self.a_star(paths, end)

    def a_star(self, considered_paths, end):
        final_paths = dict()
        for path in considered_paths:
            extended_paths = self.extend_n_times(path, considered_paths[path], 3, end)  # todo change depth
            # policz średnią z extended
            mean_cost = self.count_mean_cost(extended_paths)
            # dodaj do path_cost średnią i update na final
            final_paths[path] = mean_cost
        return min(final_paths, key=final_paths.get)

    @staticmethod
    def count_mean_cost(paths):
        if paths:
            return statistics.mean(paths.values())
        else:
            return float('inf')

    def extend_n_times(self, path, path_cost, depth, end):
        paths = dict({path: path_cost})
        extended_paths = dict()
        extended_terminal_paths = dict()
        for n in range(depth):
            new_paths = dict()
            for path_ in paths:  # rename
                extended_paths.update(self.extend_path(path_, paths[path_]))  # todo manage path cost
                # del paths[path_]
            for extended_path in extended_paths:
                if self.is_terminal_path(extended_path, end):
                    extended_terminal_paths[extended_path] = extended_paths[extended_path]
                else:
                    new_paths.update(extended_paths)
            paths = new_paths

        paths.update(extended_terminal_paths)
        return paths

    # extends path to all not visited outgoing vertices
    def extend_path(self, path_to_extend, path_to_extend_cost):
        extended_paths = dict()
        last_vertex = path_to_extend[-1]
        outgoing_vertices = self.vertices_connection[last_vertex]
        for next_vertex in outgoing_vertices:
            if next_vertex in path_to_extend:
                continue
            extended_paths[path_to_extend + tuple([next_vertex])] \
                = path_to_extend_cost + self.get_edge_weight(last_vertex, next_vertex)
        return extended_paths
