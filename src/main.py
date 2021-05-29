""" Algorithms for metric TSP """


import random
import math
import networkx as nx


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

    first = vertices.pop()
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
    pass

def christofides(G):
    pass


if __name__ == '__main__':
    pass
