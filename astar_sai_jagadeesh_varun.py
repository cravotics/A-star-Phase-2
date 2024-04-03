import numpy as np
from queue import PriorityQueue
import cv2
import time
import matplotlib.pyplot as plt

#--------------------------------------------------Creating the Canvas----------------------------------------------------#
height = 200
width = 600
Graph_map = np.ones((height, width, 3), dtype=np.uint8)*255
robot_radius = 5

RPM1 = 20
RPM2 = 30
R = 3.8  # Wheel radius

# Convert RPM to radians per second
ul = RPM1 * ((2 * np.pi) / 60)
ur = RPM2 * ((2 * np.pi) / 60)

# Wheel base of the waffle robot
L = 30.6  

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
    cv2.circle(canvas, (start_x, start_y), 1, (0, 0, 255), -1)
    # Draw the goal point in blue
    cv2.circle(canvas, (goal_x, goal_y), 1, (255, 0, 0), -1)
draw_start_goal_points(Graph_map, start_x, start_y, goal_x, goal_y)

# a and b are the coordinates of the center of the circle.
# Function to save the canvas to a file
def save_canvas(canvas, file_name):
    cv2.imwrite(file_name, canvas)
    print(f"Canvas saved to {file_name}")

# Example usage
file_name = "canvas.png"
save_canvas(Graph_map, file_name)

cv2.imshow("Canvas", Graph_map)
cv2.waitKey(0)
cv2.destroyAllWindows()


def plot_curves(node, ul, ur):
    t = 0
    new_nodes = []
    x, y, theta = node
    theta = np.deg2rad(theta)
    dt = 0.1

    while t < 1:
        t = t + dt
        xs = x
        ys = y
        x += 0.5 * r * (ul + ur) * np.cos(theta) * dt
        y += 0.5 * r * (ul + ur) * np.sin(theta) * dt
        theta += (r / L) * (ur - ul) * dt
        plt.plot([xs, x], [ys, y], color="blue")
        new_nodes.append((x, y, np.rad2deg(theta)))

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
    if 0 <= i < 1000 and 0 <= j < 2400: 
        G[i, j, k] = 1

# Checking the visited nodes.
def visited_check(node):
    i, j, k = matrix_indices(node)
    return G[i, j, k] == 1


actions = [[ul, ul], [ur, ur], [ul, 0], [0, ul], [ul, ur], [ur, ul]]

for action in actions:
    new_nodes = plot_curves(start_node, action[0], action[1])
    for new_node in new_nodes:
        plot_curves(new_node, action[0], action[1])

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