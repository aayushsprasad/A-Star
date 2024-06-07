import heapq



def get_direction_string(x1, y1, x2, y2):
    if x2 > x1:
        return "down"
    elif x2 < x1:
        return "up"
    elif y2 > y1:
        return "right"
    elif y2 < y1:
        return "left"
    else:
        return "no movement"

def get_directions_list(path):
    directions_list = []
    for i in range(len(path) - 1):
        x1, y1 = path[i]
        x2, y2 = path[i + 1]
        direction = get_direction_string(x1, y1, x2, y2)
        directions_list.append(direction)
    return directions_list

def astar(maze, start, goal):
    open_set = [(0, start)]
    came_from = {}
    cost_so_far = {start: 0}

    while open_set:
        current_cost, current_node = heapq.heappop(open_set)

        if current_node == goal:
            path = []
            while current_node in came_from:
                path.append(current_node)
                current_node = came_from[current_node]
            return path[::-1]

        for neighbor in neighbors(current_node, maze):
            new_cost = cost_so_far[current_node] + 1
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost + heuristic(goal, neighbor)
                heapq.heappush(open_set, (priority, neighbor))
                came_from[neighbor] = current_node

    return None

def neighbors(node, maze):
    x, y = node
    possible_neighbors = [(x+1, y), (x-1, y), (x, y+1), (x, y-1)]
    return [neighbor for neighbor in possible_neighbors if is_valid(neighbor, maze)]

def is_valid(node, maze):
    x, y = node
    return 0 <= x < len(maze) and 0 <= y < len(maze[0]) and maze[x][y] != 1

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def print_maze_with_path(maze, path):
    for i, row in enumerate(maze):
        for j, cell in enumerate(row):
            if (i, j) in path:
                print("*", end=" ")
            elif cell == 1:
                print("X", end=" ")
            else:
                print(" ", end=" ")
        print()

if __name__ == "__main__":
    maze = [
    [0, 1, 0, 0, 0],
    [0, 1, 0, 1, 0],
    [0, 1, 0, 1, 0],
    [0, 1, 0, 1, 0],
    [0, 0, 0, 1, 0]

]


    start = (0, 0)
    goal = (4, 4)



def main():
    path = astar(maze, start, goal)
    directions_list = get_directions_list(path)
    if path:
        print(directions_list)
    else:
        print("No path found.")


if __name__ == "__main__":
    main()
