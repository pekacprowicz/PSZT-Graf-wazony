import graph_utils

if __name__ == "__main__":
    graph = graph_utils.Graph("graph.input")
    print(graph.edges_dict)
    print(graph.vertices_connection)
    print(graph.get_shortest_path("A", "G"))
