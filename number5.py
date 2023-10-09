import heapq

# Define the graph with nodes, neighbors, edge costs, and heuristic values
graph = {
    'S': {'neighbors': {'A': 3, 'B': 1}, 'heuristic': 7},
    'A': {'neighbors': {'B': 2, 'C': 2}, 'heuristic': 5},
    'B': {'neighbors': {'C': 3}, 'heuristic': 7},
    'C': {'neighbors': {'D': 4, 'G': 4}, 'heuristic': 4},
    'D': {'neighbors': {'G': 1}, 'heuristic': 1},
    'G': {'neighbors': {}, 'heuristic': 0}
}

# Depth-First Search (Tree Search)
def depth_first_search_tree(graph, start, goal):
    stack = [(start, [])]  # Initialize a stack for DFS with the start node and an empty path
    visited = set()  # Create a set to keep track of visited nodes
    expanded_states = []  # To track the order of expanded states

    while stack:
        node, path = stack.pop()  # Pop the node and path from the stack
        expanded_states.append(node)
        if node == goal:
            return (
                "Depth-First Search (Tree Search):",
                expanded_states,
                path + [node],
                list(set(graph.keys()) - set(expanded_states))
            )
        if node not in visited:
            visited.add(node)  # Mark the node as visited
            neighbors = [neighbor for neighbor in graph[node]['neighbors']]  # Get the neighbors of the current node
            stack.extend((neighbor, path + [node]) for neighbor in reversed(neighbors) if neighbor not in visited)

# Breadth-First Search (Tree Search)
def breadth_first_search_tree(graph, start, goal):
    queue = [(start, [])]  # Initialize a queue for BFS with the start node and an empty path
    visited = set()  # Create a set to keep track of visited nodes
    expanded_states = []  # To track the order of expanded states

    while queue:
        node, path = queue.pop(0)  # Dequeue the node and path from the queue
        expanded_states.append(node)
        if node == goal:
            return (
                "Breadth-First Search (Tree Search):",
                expanded_states,
                path + [node],
                list(set(graph.keys()) - set(expanded_states))
            )
        if node not in visited:
            visited.add(node)  # Mark the node as visited
            neighbors = [neighbor for neighbor in graph[node]['neighbors']]  # Get the neighbors of the current node
            queue.extend((neighbor, path + [node]) for neighbor in neighbors if neighbor not in visited)

# Uniform Cost Search (Tree Search)
def uniform_cost_search_tree(graph, start, goal):
    priority_queue = [(0, start, [])]  # Initialize a priority queue for UCS with cost, node, and an empty path
    visited = set()  # Create a set to keep track of visited nodes
    expanded_states = []  # To track the order of expanded states

    while priority_queue:
        cost, node, path = heapq.heappop(priority_queue)  # Pop the cost, node, and path from the priority queue
        expanded_states.append(node)
        if node == goal:
            return (
                "Uniform Cost Search (Tree Search):",
                expanded_states,
                path + [node],
                list(set(graph.keys()) - set(expanded_states))
            )
        if node not in visited:
            visited.add(node)  # Mark the node as visited
            neighbors = [(neighbor, cost + graph[node]['neighbors'][neighbor]) for neighbor in graph[node]['neighbors']]  # Get neighbors and their costs
            for neighbor, new_cost in neighbors:
                heapq.heappush(priority_queue, (new_cost, neighbor, path + [node]))  # Push neighbors to the priority queue

# Greedy Search (Tree Search)
def greedy_search_tree(graph, start, goal):
    priority_queue = [(graph[start]['heuristic'], start, [])]  # Initialize a priority queue for Greedy Search
    visited = set()  # Create a set to keep track of visited nodes
    expanded_states = []  # To track the order of expanded states

    while priority_queue:
        _, node, path = heapq.heappop(priority_queue)  # Pop the heuristic, node, and path from the priority queue
        expanded_states.append(node)
        if node == goal:
            return (
                "Greedy Search (Tree Search):",
                expanded_states,
                path + [node],
                list(set(graph.keys()) - set(expanded_states))
            )
        if node not in visited:
            visited.add(node)  # Mark the node as visited
            neighbors = [neighbor for neighbor in graph[node]['neighbors']]  # Get the neighbors of the current node
            priority_queue.extend((graph[neighbor]['heuristic'], neighbor, path + [node]) for neighbor in neighbors if neighbor not in visited)

# A* Search (Tree Search)
def a_star_search_tree(graph, start, goal):
    priority_queue = [(graph[start]['heuristic'], 0, start, [])]  # Initialize a priority queue for A* Search
    visited = set()  # Create a set to keep track of visited nodes
    expanded_states = []  # To track the order of expanded states

    while priority_queue:
        _, cost_so_far, node, path = heapq.heappop(priority_queue)  # Pop heuristic, cost, node, and path from the priority queue
        expanded_states.append(node)
        if node == goal:
            return (
                "A* Search (Tree Search):",
                expanded_states,
                path + [node],
                list(set(graph.keys()) - set(expanded_states))
            )
        if node not in visited:
            visited.add(node)  # Mark the node as visited
            neighbors = [(neighbor, cost_so_far + graph[node]['neighbors'][neighbor]) for neighbor in graph[node]['neighbors']]  # Get neighbors and their costs
            for neighbor, new_cost in neighbors:
                heapq.heappush(priority_queue, (new_cost + graph[neighbor]['heuristic'], new_cost, neighbor, path + [node]))  # Push neighbors to the priority queue

# Format and print search results
def print_search_results(result):
    search_name, expanded_states, path, not_expanded = result
    print(f"{search_name}\nOrder: {expanded_states}\nPath: {path}\nNot Expanded: {not_expanded}\n")

# Test the search algorithms for Tree Search
goal_state = 'G'
dfs_result = depth_first_search_tree(graph, 'S', goal_state)
bfs_result = breadth_first_search_tree(graph, 'S', goal_state)
ucs_result = uniform_cost_search_tree(graph, 'S', goal_state)
greedy_result = greedy_search_tree(graph, 'S', goal_state)
a_star_result = a_star_search_tree(graph, 'S', goal_state)

# Printing results
print_search_results(dfs_result)
print_search_results(bfs_result)
print_search_results(ucs_result)
print_search_results(greedy_result)
print_search_results(a_star_result)
