from collections import deque
import heapq

# Define a graph as a dictionary where each node is a key, and its neighbors are values with associated costs.
graph = {
    'S': {'A': 3, 'B': 1},
    'A': {'S': 3, 'B': 2, 'C': 2},
    'B': {'C': 3, 'S': 1, 'A': 2},
    'C': {'A': 2, 'D': 4, 'B': 3, 'G': 4},
    'D': {'C': 4, 'G': 1},
    'G': {'D': 1, 'C': 4},
}

# Depth-First Search (Graph Search)
def depth_first_search(graph, start, goal):
    stack = [(start, [])]  # Initialize a stack for DFS with the start node and an empty path
    visited = set()  # Create a set to keep track of visited nodes

    while stack:
        node, path = stack.pop()  # Pop the node and path from the stack
        if node == goal:
            path.append(node)  # Include the goal state 'G' in the path
            return "Depth-First Search (Graph Search):", path
        if node not in visited:
            visited.add(node)  # Mark the node as visited
            neighbors = [neighbor for neighbor in graph[node].keys()]  # Get the neighbors of the current node
            stack.extend((neighbor, path + [node]) for neighbor in reversed(neighbors) if neighbor not in visited)

    # If the goal is not found, return an appropriate message
    return "Depth-First Search (Graph Search): Goal not found"

# Breadth-First Search (Graph Search)
def breadth_first_search(graph, start, goal):
    queue = [(start, [])]  # Initialize a queue for BFS with the start node and an empty path
    visited = set()  # Create a set to keep track of visited nodes

    while queue:
        node, path = queue.pop(0)  # Dequeue the node and path from the queue
        if node == goal:
            path.append(node)  # Include the goal state 'G' in the path
            return "Breadth-First Search (Graph Search):", path
        if node not in visited:
            visited.add(node)  # Mark the node as visited
            neighbors = [neighbor for neighbor in graph[node].keys()]  # Get the neighbors of the current node
            queue.extend((neighbor, path + [node]) for neighbor in neighbors if neighbor not in visited)

    # If the goal is not found, return an appropriate message
    return "Breadth-First Search (Graph Search): Goal not found"

# Uniform Cost Search (Graph Search)
def uniform_cost_search(graph, start, goal):
    priority_queue = [(0, start, [])]  # Initialize a priority queue for UCS with cost, node, and an empty path
    visited = set()  # Create a set to keep track of visited nodes

    while priority_queue:
        cost, node, path = heapq.heappop(priority_queue)  # Pop the cost, node, and path from the priority queue
        if node == goal:
            path.append(node)  # Include the goal state 'G' in the path
            return "Uniform Cost Search (Graph Search):", path
        if node not in visited:
            visited.add(node)  # Mark the node as visited
            neighbors = [(neighbor, cost + graph[node][neighbor]) for neighbor in graph[node].keys()]  # Get neighbors and their costs
            for neighbor, new_cost in neighbors:
                heapq.heappush(priority_queue, (new_cost, neighbor, path + [node]))  # Push neighbors to the priority queue

    # If the goal is not found, return an appropriate message
    return "Uniform Cost Search (Graph Search): Goal not found"

# Greedy Search (Graph Search)
def greedy_search(graph, start, goal):
    priority_queue = [(graph[start].get('heuristic', 0), start, [])]  # Initialize a priority queue for Greedy Search
    visited = set()  # Create a set to keep track of visited nodes

    while priority_queue:
        _, node, path = heapq.heappop(priority_queue)  # Pop the heuristic, node, and path from the priority queue
        if node == goal:
            path.append(node)  # Include the goal state 'G' in the path
            return "Greedy Search (Graph Search):", path
        if node not in visited:
            visited.add(node)  # Mark the node as visited
            neighbors = [neighbor for neighbor in graph[node].keys()]  # Get the neighbors of the current node
            priority_queue.extend((graph[neighbor].get('heuristic', 0), neighbor, path + [node]) for neighbor in neighbors if neighbor not in visited)

    # If the goal is not found, return an appropriate message
    return "Greedy Search (Graph Search): Goal not found"

# A* Search (Graph Search)
def a_star_search(graph, start, goal):
    priority_queue = [(graph[start].get('heuristic', 0), 0, start, [])]  # Initialize a priority queue for A* Search
    visited = set()  # Create a set to keep track of visited nodes

    while priority_queue:
        _, cost_so_far, node, path = heapq.heappop(priority_queue)  # Pop heuristic, cost, node, and path from the priority queue
        if node == goal:
            path.append(node)  # Include the goal state 'G' in the path
            return "A* Search (Graph Search):", path
        if node not in visited:
            visited.add(node)  # Mark the node as visited
            neighbors = [(neighbor, cost_so_far + graph[node][neighbor]) for neighbor in graph[node].keys()]  # Get neighbors and their costs
            for neighbor, new_cost in neighbors:
                heapq.heappush(priority_queue, (new_cost + graph[neighbor].get('heuristic', 0), new_cost, neighbor, path + [node]))  # Push neighbors to the priority queue

    # If the goal is not found, return an appropriate message
    return "A* Search (Graph Search): Goal not found"

# Test the search algorithms for Graph Search
goal_state = 'G'
print(*depth_first_search(graph, 'S', goal_state), "\n")
print(*breadth_first_search(graph, 'S', goal_state), "\n")
print(*uniform_cost_search(graph, 'S', goal_state), "\n")
print(*greedy_search(graph, 'S', goal_state), "\n")
print(*a_star_search(graph, 'S', goal_state), "\n")
