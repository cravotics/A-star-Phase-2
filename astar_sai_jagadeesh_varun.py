import numpy as np
from queue import PriorityQueue
import matplotlib.pyplot as plt
import cv2
import time

#--------------------------------------------------Creating the Canvas----------------------------------------------------#
height = 200
width = 600
Graph_map = np.ones((height, width, 3), dtype=np.uint8)*255
robot_radius = 22
#user input for RPM1 and RPM2
RPM1 = int(input("Enter the RPM1: "))
RPM2 = int(input("Enter the RPM2: "))

#--------------------------------------------------Creating the Robot----------------------------------------------------#
R = 3.3  # Wheel radius in cm

# Convert RPM to radians per second
ul = RPM1 * ((2 * np.pi) / 60)
ur = RPM2 * ((2 * np.pi) / 60)

# Wheel base of the waffle robot
L = 28.7 # cm
heuristic_cache = {}
#--------------------------------------------------Creating the User Interface-------------------------------------------------#

## Taking input from the user for start and goal nodes.
# User  input for x and y coordinates of start node.
def start_node(width, height, canvas):
    while True:
        try:
            Xs = int(input("Enter the x-coordinate of the start node(Xs): "))
            start_y = int(input("Enter the y-coordinate of the start node(Ys): "))
            Ys = height - start_y
            start_theta = int(input("Enter the angle of the start_node: "))
            
            if Xs < 0 or Xs >= width or Ys < 0 or Ys >= height:
                print("The x and y coordinates of the start node is out of range.Try again!!!")
            elif np.any(canvas[Ys, Xs] != [255, 255,255]):
                print("The x or y or both coordinates of the start node is on the obstacle.Try again!!!")
            elif start_theta % 30 != 0:
                print("The angle of the start node is out of range.Try again!!!")
            else:
                return Xs, Ys, start_theta
        except ValueError:
            print("The x and y coordinates of the start node is not a number. Try again!!!")
        

def goal_node(width, height, canvas):
    while True:
        try:
            Xg = int(input("Enter the x-coordinate of the goal node(Xg): "))
            goal_y = int(input("Enter the y-coordinate of the goal node(Yg): "))
            Yg = height - goal_y
            
            if Xg < 0 or Xg >= width or Yg < 0 or Yg >= height:
                print("The x and y coordinates of the goal node is out of range.Try again!!!")
            elif np.any(canvas[Yg,Xg] != [255,255,255]):
                print("The x or y or both coordinates of the goal node is on the obstacle.Try again!!!")
            else:
                return Xg, Yg
        except ValueError:
            print("The x and y coordinates of the goal node is not a number. Try again!!!")

clearance = int(input("Enter the clearance of the robot: "))
r = robot_radius
Total_clearance = clearance + r
#--------------------------------------------------Creating the Obstacles-------------------------------------------------#

for x in range(width):
    for y in range(height):
        y_transform = 200 - y

        # Wall clearance.
        if (x <= 0 + Total_clearance or x >= 600 - Total_clearance or y_transform <= 0 + Total_clearance or y_transform >= 200 - Total_clearance):
            Graph_map[y,x] = [0,255,0]
        
        # object 1(rectangle)
        if (x >= 150 and x <= 175  and y_transform >= 100 and y_transform <= 200 ):
            Graph_map[y,x] = [0,0,0]
        elif (x >= 150 - Total_clearance  and x <= 175 + Total_clearance and y_transform >= 100 - Total_clearance and y_transform <= 200 + Total_clearance):
            Graph_map[y,x] = [0, 255, 0]
        
        # object 2(rectangle)
        if (x >= 250 and x <= 275 and y_transform >= 0 and y_transform <= 100):
            Graph_map[y,x] = [0,0,0]
        elif(x >= 250 - Total_clearance and x <= 275 + Total_clearance and y_transform >= 0 - Total_clearance and y_transform <= 100 + Total_clearance):
             Graph_map[y,x] = [0, 255, 0] 

#--------------------------------------------------Creating the Obstacles-------------------------------------------------#

# object 3(circle) using half planes method 
for x in range(width):
    for y in range(height):
        y_transform = 200 - y
    
        if ((x-420)**2 + (y_transform-120)**2 <= (60 + Total_clearance)**2):
            Graph_map[y, x] = [0,255,0] 

        if ((x-420)**2 + (y_transform -120)**2 <= (60)**2):
            Graph_map[y, x] = [0,0,0]  


#--------------------------------------------------Creating the User Interface-------------------------------------------------#
Xs, Ys, start_theta = start_node(width, height, Graph_map) # Getting the start node from the user
Xg, Yg = goal_node(width, height, Graph_map) # Getting the goal node from the user

start_node = (Xs, Ys, start_theta)
goal_node = (Xg, Yg, )
start_x, start_y, start_theta = start_node
goal_x, goal_y = goal_node
#--------------------------------------------------Creating the Graph-------------------------------------------------#
def draw_start_goal_points(canvas, start_x, start_y, goal_x, goal_y):
    # Draw the start point in red
    cv2.circle(canvas, (start_x, start_y), 10, (0, 0, 255), -1)
    # Draw the goal point in blue
    cv2.circle(canvas, (goal_x, goal_y), 10, (255, 0, 0), -1)
draw_start_goal_points(Graph_map, start_x, start_y, goal_x, goal_y)



# a and b are the coordinates of the center of the circle.
# Function to save the canvas to a file
def save_canvas(canvas, file_name):
    cv2.imwrite(file_name, canvas)
    print(f"Canvas saved to {file_name}")

# Example usage
file_name = "canvas.png"
save_canvas(Graph_map, file_name)

cv2.imshow("Resized Canvas", Graph_map)
cv2.waitKey(0)
cv2.destroyAllWindows()



def possible_node(node, ul, ur):
    t = 0
    new_nodes = []
    x, y, theta = node
    theta = np.deg2rad(theta)
    dt = 0.1
    rows, columns, _ = Graph_map.shape
    while t < 1:
        t = t + dt
        xs = x
        ys = y
        x += 0.5 * R * (ul + ur) * np.cos(theta) * dt
        y += 0.5 * R * (ul + ur) * np.sin(theta) * dt
        theta += (R / L) * (ur - ul) * dt
        plt.plot([xs, x], [ys, y], color="blue")
        new_node = (x, y, np.rad2deg(theta)% 360)
        next_x, next_y = new_node[0], new_node[1]
        cost= L      
        actions = [[ul, ul], [ur, ur], [ul, 0], [0, ul], [ul, ur], [ur, ul]]

        for action in actions:
            new_nodes = possible_node(start_node, action[0], action[1])
            for new_node in new_nodes:
                possible_node(new_node, action[0], action[1])
                if 0 <= next_x <= columns and 0 <= next_y < rows and np.all(Graph_map[int(next_y), int(next_x)] == [255, 255, 255]) and not visited_check(new_node):
                     new_nodes.append((cost, new_node))

    return new_nodes

G = np.zeros((200, 600, 12), dtype=np.uint8)

# Getting the indices of the matrix.
def matrix_indices(node):
    x, y, theta = node
    x = round(x)
    y = round(y)
    i = int(2 * y) 
    j = int(2 * x)  
    k = int(theta / 30) % 12  
    return i, j, k

# Marking the visited nodes.
def marking_visited(node):
    i, j, k = matrix_indices(node)
    if 0 <= i < 200 and 0 <= j < 600: 
        G[i, j, k] = 1

# Checking the visited nodes.
def visited_check(node):
    i, j, k = matrix_indices(node)
    return G[i, j, k] == 1




plt.grid()
plt.xlim(0, width)
plt.ylim(0, height)
plt.gca().invert_yaxis()
plt.show()
plt.close()

def heuristic(node, goal):
    if node in heuristic_cache:
        return heuristic_cache[node]
    else:
        heuristic_value = np.sqrt((node[0] - goal[0])**2 + (node[1] - goal[1])**2)
        heuristic_cache[node] = heuristic_value
        return heuristic_value

output = cv2.VideoWriter('A_star_Varun_Lakshmanan_Sai_Jagadeesh_Muralikrishnan.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 30, (width, height))

def A_star(start_node, goal_node):
    parent = {}
    cost_list = {start_node:0}
    closed_list = set()
    open_list = PriorityQueue()
    open_list.put(((0 + heuristic(start_node, goal_node)), start_node))
    map_visualization = np.copy(Graph_map)
    marking_visited(start_node)
    step_count = 0 
    
    # While loop to check the open_list is empty or not.
    while not open_list.empty():
        current_cost, current_node = open_list.get()
        closed_list.add(current_node)
        
        # If the current node is equal to goal node, then it will break the loop and return the path along with writing the path to the video.
        if heuristic(current_node, goal_node) < 1.5 and current_node[2] == goal_node[2]:
            path = A_star_Backtracting(parent, start_node, current_node, map_visualization, step_count)
            for _ in range(80):
               output.write(map_visualization)
            return path
        
        # If the current node is not equal to goal node, then it will check the possible nodes and add it to the open_list along with visulizing the node exploration.   
        for cost, new_node in possible_node(current_node):
            cost_to_come = cost_list[current_node] + cost
            if new_node not in cost_list or cost_to_come < cost_list[new_node]:
                cost_list[new_node] = cost_to_come
                parent[new_node] = current_node
                cost_total = cost_to_come + heuristic(new_node, goal_node) 
                open_list.put((cost_total, new_node))
                marking_visited(new_node)
    return None

def A_star_Backtracting(parent, start_node, end_node, map_visualization, step_count):
    path = [end_node] # Adding end node to the path
    while end_node != start_node: # If the end node is not equal to start_node, parent of the end_node is added to path and continues.
        path.append(parent[end_node])
        end_node = parent[end_node] # The parent of end node becomes the current node.
    path.reverse()
    for i in range(len(path) - 1):
        start_point = (int(path[i][0]), int(path[i][1]))  # Converting the coordinates for visualization.
        end_point = (int(path[i + 1][0]), int(path[i + 1][1]))
        cv2.arrowedLine(map_visualization, start_point, end_point, (255, 0, 0), 1, tipLength=0.3)
        if step_count % 5 == 0:
            output.write(map_visualization)
        step_count += 1 
    return path
Xs, Ys, start_theta = start_node(width, height, Graph_map) # Getting the start node from the user
Xg, Yg, goal_theta = goal_node(width, height, Graph_map) # Getting the goal node from the user


#--------------------------------------Initializing the nodes------------------------------------#
start_node = (Xs, Ys, start_theta)
goal_node = (Xg, Yg)

start_time = time.time()   # Starting to check the runtime.
path = A_star(start_node, goal_node)

if path is None:
    print("No optimal path found")
else:
    print("Path found")

end_time = time.time()    # end of runtime
print(f'Runtime : {((end_time-start_time)/60):.2f} Minutes')