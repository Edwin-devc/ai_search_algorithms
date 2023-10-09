from collections import deque
import heapq

# Weighted graph representation
graph = {
    'S': {'A': 3, 'B': 1},
    'A': {'S': 3, 'B': 2, 'C': 2},
    'B': {'B': 1, 'A': 2, 'C': 3},
    'C': {'A': 2, 'B': 3, 'D': 4, 'G': 4},
    'D': {'C': 4, 'G': 1},
    'G': {'C': 4, 'D': 1}
}

# Heuristics for each node
heuristics = {
    'S': 7,
    'A': 5,
    'B': 7,
    'C': 4,
    'D': 1,
    'G': 0
}

# Define a function to perform search algorithms
def search_algorithm(graph, start, goal, heuristic=None, priority_func=None):
    visited = set()
    expanded_states = []
    start_priority = 0

    if heuristic is not None:
        start_priority += heuristic[start]

    if priority_func is not None:
        priority_queue = [(start_priority, start, [start])]
    else:
        priority_queue = deque([(start, [start])])

    while priority_queue:
        if priority_func is not None:
            _, node, path = heapq.heappop(priority_queue)
        else:
            node, path = priority_queue.popleft()

        expanded_states.append(node)

        if node == goal:
            return path, expanded_states, []  # Return an empty list for unexpanded states

        if node not in visited:
            visited.add(node)
            neighbors = sorted(graph[node].keys(), key=lambda n: graph[node][n])

            for neighbor in neighbors:
                if neighbor not in visited:
                    new_cost = path_cost(path, graph) + graph[node][neighbor]
                    new_priority = new_cost

                    if heuristic is not None:
                        new_priority += heuristic[neighbor]

                    if priority_func is not None:
                        heapq.heappush(priority_queue, (new_priority, neighbor, path + [neighbor]))
                    else:
                        priority_queue.append((neighbor, path + [neighbor]))

    return None, expanded_states, []  # Return an empty list for unexpanded states

# Calculate path cost
def path_cost(path, graph):
    cost = 0
    for i in range(len(path) - 1):
        cost += graph[path[i]][path[i + 1]]
    return cost

# Perform searches
start_node = 'S'
goal_node = 'G'

# Breadth-First Search
bfs_path, bfs_expanded, _ = search_algorithm(graph, start_node, goal_node, priority_func="bfs")
if bfs_path is not None:
    print("BFS Path:", bfs_path)
else:
    print("BFS did not find a path.")
print("BFS Expanded States:", bfs_expanded)

# Depth-First Search
dfs_path, dfs_expanded, _ = search_algorithm(graph, start_node, goal_node, priority_func="dfs")
if dfs_path is not None:
    print("DFS Path:", dfs_path)
else:
    print("DFS did not find a path.")
print("DFS Expanded States:", dfs_expanded)

# Uniform Cost Search
ucs_path, ucs_expanded, _ = search_algorithm(graph, start_node, goal_node)
if ucs_path is not None:
    print("UCS Path:", ucs_path)
else:
    print("UCS did not find a path.")
print("UCS Expanded States:", ucs_expanded)

# A* Search
astar_path, astar_expanded, _ = search_algorithm(graph, start_node, goal_node, heuristics)
if astar_path is not None:
    print("A* Search Path:", astar_path)
else:
    print("A* Search did not find a path.")
print("A* Search Expanded States:", astar_expanded)

# Greedy Search
greedy_path, greedy_expanded, _ = search_algorithm(graph, start_node, goal_node, heuristics, priority_func="greedy")
if greedy_path is not None:
    print("Greedy Search Path:", greedy_path)
else:
    print("Greedy Search did not find a path.")
print("Greedy Search Expanded States:", greedy_expanded)
