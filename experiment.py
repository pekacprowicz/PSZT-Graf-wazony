import graph_utils
import time

if __name__ == "__main__":
    graph = graph_utils.Graph("graph.input")
    print(graph.edges_dict)
    print(graph.vertices_connection)
    start_time = time.time()
    print(graph.get_shortest_path("A", "G"))
    print("--- %s seconds ---" % (time.time() - start_time))
