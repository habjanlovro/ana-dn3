""" Algorithms for metric TSP """


import random
import math
import time
import networkx as nx
import matplotlib.pyplot as plt


def generate_graph(n, max_x, max_y):
    """
    Generates a fully connected graph of n points by creating them in R^2
    and then calculating the Euclidian distance
    """

    points = [(random.uniform(-max_x, max_x), random.uniform(-max_y, max_y))
              for _ in range(n)]
    edges = []
    for i in range(len(points)-1):
        for j in range(i + 1, len(points)):
            x = points[i][0] - points[j][0]
            y = points[i][1] - points[j][1]
            d = math.sqrt(x ** 2 + y ** 2)
            edges.append((i, j, d))

    G = nx.Graph()
    G.add_weighted_edges_from(edges)
    return G


def nearest_neighbor(G):
    """
    Computes an approxmation of metric TSP by starting in a city and then going
    always travelling to the closest city that hasn't been visited yet.
    """
    def find_min_edge(v, vs):
        m_v = None
        m_d = -1
        for u in vs:
            d = G.edges[v, u]['weight']
            if m_d < 0 or d < m_d:
                m_d = d
                m_v = u
        return m_v, m_d

    vertices = list(G.nodes)
    if len(vertices) == 1:
        return vertices[0], 0

    first = vertices.pop(0)
    visited = [first]
    full_distance = 0
    while len(vertices) > 0:
        v = visited[-1]
        min_vertex, min_distance = find_min_edge(v, vertices)
        visited.append(min_vertex)
        vertices.remove(min_vertex)
        full_distance += min_distance

    full_distance += G.edges[visited[0], visited[-1]]['weight']
    return visited, full_distance


def apx_2(G):
    """
    2-approximation algorithm for metric TSP using minimum spanning tree
    """
    T = nx.algorithms.tree.mst.minimum_spanning_tree(G)
    vertices = list(nx.algorithms.traversal.depth_first_search.dfs_preorder_nodes(T))
    distance = G.edges[vertices[0], vertices[-1]]['weight']
    for i in range(len(vertices) - 1):
        distance += G.edges[vertices[i], vertices[i+1]]['weight']
    return vertices, distance


def christofides(G):
    """
    Christofides 1.5-approximation algorithm for metric TSP
    """
    T = nx.algorithms.tree.mst.minimum_spanning_tree(G)
    O = filter(lambda v : T.degree(v) % 2 == 1, T.nodes)
    subgraph = G.subgraph(O)
    for v, u in subgraph.edges:
        subgraph.edges[v, u]['weight'] *= -1
    M = nx.algorithms.matching.max_weight_matching(subgraph, maxcardinality=True)
    for v, u in subgraph.edges:
        subgraph.edges[v, u]['weight'] *= -1

    multigraph = nx.MultiGraph()
    multigraph.add_edges_from(T.edges)
    multigraph.add_edges_from(M)

    vertices = []
    for u, v in nx.algorithms.euler.eulerian_circuit(multigraph):
        if u not in vertices:
            vertices.append(u)

    distance = G.edges[vertices[0], vertices[-1]]['weight']
    for i in range(len(vertices) - 1):
        distance += G.edges[vertices[i], vertices[i+1]]['weight']
    return vertices, distance


def run_get_results(f, G):
    t_start = time.time()
    _, d = f(G)
    t_end = time.time()
    return (d, t_end - t_start)


def run_once(n, x, y):
    time_nn = 0
    results_nn = []
    time_a2 = 0
    results_a2 = []
    time_cris = 0
    results_cris = []
    for _ in range(10):
        G = generate_graph(n, x, y)

        d, t = run_get_results(nearest_neighbor, G)
        time_nn += t
        results_nn.append(d)

        d, t = run_get_results(apx_2, G)
        time_a2 += t
        results_a2.append(d)

        d, t = run_get_results(christofides, G)
        time_cris += t
        results_cris.append(d)
    time_nn /= 10
    time_a2 /= 10
    time_cris /= 10
    return [(time_nn, results_nn), (time_a2, results_a2), (time_cris, results_cris)]


def run_tests(size_x, size_y):
    n = 10
    times_nn = []
    times_a2 = []
    times_cris = []
    ns = []

    distances = []

    for i in range(30):
        results = run_once(n, size_x, size_y)

        times_nn.append(results[0][0])
        times_a2.append(results[1][0])
        times_cris.append(results[2][0])

        distances.append((results[0][1], results[1][1], results[2][1]))

        ns.append(n)
        n += 10

    plt.figure(1)
    plt.plot(ns, times_nn, label = "greedy")
    plt.plot(ns, times_a2, label = "2-APX")
    plt.plot(ns, times_cris, label = "1.5-APX")
    plt.xlabel("Size of graph")
    plt.ylabel("Execution time")
    plt.legend()
    plt.suptitle("Execution time of algorithms")

    figure_index = 2
    for i in range(0, len(ns), 10):
        plot_case(ns[i], figure_index, distances[i][0], distances[i][1], distances[i][2])
        figure_index += 1
    plot_case(ns[-1], figure_index, distances[-1][0], distances[-1][1], distances[-1][2])

    plt.show()

def plot_case(n, fig_num, results_nn, results_a2, results_cris):
    n_cases = range(1, len(results_nn) + 1)
    plt.figure(fig_num)
    plt.plot(n_cases, results_nn, label = "greedy")
    plt.plot(n_cases, results_a2, label = "2-APX")
    plt.plot(n_cases, results_cris, label = "1.5-APX")
    plt.xlabel("Test number")
    plt.ylabel("Solutions")
    plt.legend()
    plt.suptitle("Comparison of solutions for n = " + str(n))


if __name__ == '__main__':
    run_tests(1000, 1000)
