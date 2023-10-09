# Define the nodes (states) with their heuristic values
nodes = {
    'S': 7,
    'B': 7,
    'A': 5,
    'C': 4,
    'D': 1,
    'G': 0
}

# Define the edges with their path costs
edges = {
    ('S', 'A'): 3,
    ('S', 'B'): 1,
    ('A', 'B'): 2,
    ('A', 'C'): 2,
    ('B', 'C'): 3,
    ('C', 'D'): 4,
    ('C', 'G'): 4,
    ('D', 'G'): 1
}

# Set to store the nodes
nodes_set = set(nodes.keys())

# Dictionary to represent the graph
graph = {}

# Add nodes to the graph with their heuristic values
for node, heuristic in nodes.items():
    graph[node] = {
        'heuristic': heuristic,
        'neighbors': {}
    }

# Add edges to the graph with their path costs
for (source, target), cost in edges.items():
    graph[source]['neighbors'][target] = cost

# Printing the graph structure
print(graph)
