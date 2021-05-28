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
    print(points)
    print(edges)

    G = nx.Graph()
    G.add_weighted_edges_from(edges)
    return G


def nearest_neighbor(G):
    pass

def apx_2(G):
    pass

def christofides(G):
    pass


if __name__ == '__main__':
    pass
