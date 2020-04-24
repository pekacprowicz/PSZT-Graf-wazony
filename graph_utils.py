import pandas as pd
import statistics


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

    # finds shortest path based on chosen algorithm
    def get_shortest_path(self, start, end, algorithm="first_best", depth=3):
        paths = dict()
        paths[tuple([start])] = 0
        shortest_terminal_path = ("", float('inf'))
        steps = 0
        while paths:
            steps += 1

            path_to_extend = self.choose_next_path(paths, algorithm, end, depth)
            extended_paths = self.extend_path(path_to_extend, paths[path_to_extend])

            for extended_path in extended_paths:
                if self.is_terminal_path(extended_path, end):
                    if extended_paths[extended_path] <= shortest_terminal_path[1]:
                        shortest_terminal_path = (extended_path, extended_paths[extended_path])
                else:
                    paths.update(extended_paths)

            if shortest_terminal_path[1] <= min(paths.values()):
                return shortest_terminal_path[0], shortest_terminal_path[1], steps

            del paths[path_to_extend]

    # chooses path to be extended next based on chosen algorithm
    def choose_next_path(self, paths, algorithm, end, depth):
        if algorithm == "brute_force":  # returns oldest path
            return next(iter(paths))
        elif algorithm == "first_best":  # returns path with min value
            return min(paths, key=paths.get)
        elif algorithm == "a_star":     # uses heuristic function to evaluate next best path
            return self.cheapest_n_deep(paths, end, depth)

    # chooses next path based on n time extended paths' mean cost
    def cheapest_n_deep(self, considered_paths, end, depth):
        final_paths = dict()
        for path in considered_paths:
            extended_paths = self.extend_path_n_times(path, considered_paths[path], end, depth)
            mean_cost = self.count_paths_mean_cost(extended_paths)
            final_paths[path] = mean_cost
        return min(final_paths, key=final_paths.get)

    # counts mean cost of paths
    @staticmethod
    def count_paths_mean_cost(paths):
        if paths:
            return statistics.mean(paths.values())
        else:
            return float('inf')

    # extends path n times or till terminal vertex
    def extend_path_n_times(self, base_path, base_path_cost, end, depth):
        paths_to_extend = dict({base_path: base_path_cost})
        extended_paths = dict()
        extended_terminal_paths = dict()
        for n in range(depth):
            new_paths = dict()
            for path in paths_to_extend:  # rename
                extended_paths.update(self.extend_path(path, paths_to_extend[path]))
            for extended_path in extended_paths:
                if self.is_terminal_path(extended_path, end):
                    extended_terminal_paths[extended_path] = extended_paths[extended_path]
                else:
                    new_paths.update(extended_paths)
            paths_to_extend = new_paths

        paths_to_extend.update(extended_terminal_paths)
        return paths_to_extend

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
