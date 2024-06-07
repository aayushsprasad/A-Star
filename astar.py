import heapq
from pololu_3pi_2040_robot import robot
import time

def get_direction_string(x1, y1, x2, y2):
    """if x2 == x1 or y2 == y1:
        return "straight" """
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
#print(directions_list)

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
    return 0 <= x < len(maze) and 0 <= y < len(maze[0]) and maze[x][y] == 0

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def update_configuration_space(grid, config_space, robot_radius, square_size):
    for i in range(len(grid)):
        for j in range(len(grid[0])):
            if grid[i][j] == 1:  # Obstacle
                for di in range(-robot_radius // square_size, robot_radius // square_size + 1):
                    for dj in range(-robot_radius // square_size, robot_radius // square_size + 1):
                        ni, nj = i + di, j + dj
                        if 0 <= ni < len(grid) and 0 <= nj < len(grid[0]):
                            config_space[ni][nj] = 1

def create_configuration_space(grid, robot_radius, square_size):
    config_space = [[0] * len(grid[0]) for _ in range(len(grid))]
    update_configuration_space(grid, config_space, robot_radius, square_size)
    return config_space


if __name__ == "__main__":
    maze = [
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,],
    [0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0,],
    [0, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0,],
    [0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,],
    [0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0,],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0,],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0,],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0,],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1,],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0,],
    [0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0,],
    [0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0,],
    [0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0,],
    [0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 1,],
    [0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0,],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,],
    [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,]
]  
    robot_radius = 4.85
    square_size = 10


    config_space = create_configuration_space(maze, int(robot_radius), square_size)



    for row in config_space:
        print(row)

    start = (0, 0)
    goal = (len(maze) - 1, len(maze[0]) - 1)

   
    
    
   

# Define your Pololu robot class and its methods

        

def follow_directions(directions_list):
    motors = robot.Motors()
    #buzzer = robot.Buzzer()
    display = robot.Display()
    max_speed = 1000
    turn_time = 250
    #time.sleep_ms(1000)
    for direction in directions_list:
        time.sleep_ms(1000)
        if direction == "up":
            motors.set_speeds(max_speed, max_speed)
            #buzzer.play("a32")
            display.fill(0)
            display.text("Moving forward", 0, 0)
            display.show()
        elif direction == "down":
            motors.set_speeds(-max_speed, -max_speed)
            #buzzer.play("a32")
            display.fill(0)
            display.text("Moving backward", 0, 0)
            display.show()
        elif direction == "right":
            motors.set_speeds(-max_speed, max_speed)
            #buzzer.play("a32")
            display.fill(0)
            display.text("Turning Left", 0, 0)
            display.show()
        elif direction == "left":
            motors.set_speeds(max_speed, -max_speed)
            #buzzer.play("a32")
            display.fill(0)
            display.text("Moving backward", 0, 0)
            display.show()
        else:
            print("Invalid direction:", direction)

def main():
    path = astar(maze, start, goal)
    directions_list = get_directions_list(path)
    if path:
        print("Path found:", path)
        print("Directions:", directions_list)
        follow_directions(directions_list)
    else:
        print("No path found.")


if __name__ == "__main__":
    main()
