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
def RPM_input():
    print("Enter Wheel RPMS, 2 Unique, Separated By Spaces")
    RPMs = [int(x) for x in input().split()]
    return  RPMs

#--------------------------------------------------Creating the Robot----------------------------------------------------#
R = 3.3  # Wheel radius in cm

# Convert RPM to radians per second
# ul = RPM1 * ((2 * np.pi) / 60)
# ur = RPM2 * ((2 * np.pi) / 60)

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
            Ys = start_y
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
            Yg =  goal_y
            
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
        y_transform = y

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
        y_transform = y
    
        if ((x-420)**2 + (y_transform-120)**2 <= (60 + Total_clearance)**2):
            Graph_map[y, x] = [0,255,0] 

        if ((x-420)**2 + (y_transform -120)**2 <= (60)**2):
            Graph_map[y, x] = [0,0,0]  


#--------------------------------------------------Creating the Graph-------------------------------------------------#

# This function checks if the new position is valid within the map constraints and is not colliding with obstacles.
def is_valid_position(x, y, graph_map):
    rows, cols, _ = graph_map.shape
    if 0 <= x < cols and 0 <= y < rows and np.array_equal(graph_map[int(y), int(x)], [255, 255, 255]):
        return True
    return False

def possible_nodes(current_node, RPMS, Graph_map):
    global R
    global L
    t = 0 
    dt = 0.1 
    ul = RPMS[0] * ((2 * np.pi) / 60)
    ur = RPMS[1] * ((2 * np.pi) / 60)
    Curr_Node_X = current_node[0] #Grab Current Node X
    Curr_Node_Y = current_node[1] #Grad Current Node Y
    Curr_Node_Theta = np.deg2rad(current_node[2]) #Grab Current Node Theta, convert to radians.

    distance_cost = 0.0 #Init Cost

    next_x = Curr_Node_X #Set New Node Start Point X
    next_y = Curr_Node_Y #Set New Node Start Point Y
    next_theta = Curr_Node_Theta #Set New Node Start Point Theta

    ##----------------Euler Integration to Generate Curvature----------------##
    while t < 1:
        t += dt
        del_x = 0.5*R*(ul+ur)*np.cos(next_theta)*dt
        del_y = 0.5*R*(ul+ur)*np.sin(next_theta)*dt
        del_theta = (R/L)*(ur-ul)*dt

        next_x += del_x
        next_y += del_y
        next_theta += del_theta

        distance_cost += np.sqrt((del_x)**2 + (del_y)**2)

        
        if is_valid_position(next_x, next_y, Graph_map) == False:
            return None, None
        
    next_theta = int(np.rad2deg(next_theta)) #Convert back to Degrees

    if next_theta >= 360:
        next_theta = next_theta - 360
    if next_theta < -360:
        next_theta = next_theta + 360

    return [next_x, next_y, next_theta], distance_cost

def check_goal_reached(current_node, goal_node):
    if heuristic(current_node, goal_node) < 1.5:
        return True

def heuristic(node, goal_node):
    cost_to_go = 0.0
    X_Current = node[0]
    Y_Current = node[1]
    X_Goal = goal_node[0]
    Y_Goal = goal_node[1]
    if node is not None:
        cost_to_go = np.sqrt((X_Goal-X_Current)**2 + (Y_Goal- Y_Current)**2)
    return cost_to_go
#--------------------------------------------------Creating the Matrix-------------------------------------------------#
Thrshold_x = 0.5
Thrshold_y = 0.5
Thrshold_theta = 30
G = np.array([[[ 0 for k in range(int(360/Thrshold_theta))] for j in range(int(width/Thrshold_x))] for i in range(int(height/Thrshold_y))])

#--------------------------------------------------Plotting the Path-------------------------------------------------#
def PlotCurves(current_node, RPMS, Graph_map):
    global R
    global L
    ul = RPMS[0] * ((2 * np.pi) / 60)
    ur = RPMS[1] * ((2 * np.pi) / 60)
    Color = 'blue'
    t = 0
    dt = 0.1
    current_nodfe_x = current_node[0]
    current_nodfe_y = current_node[1]
    current_nodfe_theta = np.deg2rad(current_node[2])

    new_node_x = current_nodfe_x
    new_node_y = current_nodfe_y
    new_node_theta = current_nodfe_theta

    while t < 1:
        t += dt
        X_Start = new_node_x
        Y_Start = new_node_y
        del_x = 0.5*R*(ul+ur)*np.cos(new_node_theta)*dt
        del_y = 0.5*R*(ul+ur)*np.sin(new_node_theta)*dt
        del_theta = (R/L)*(ur-ul)*dt

        new_node_x += del_x
        new_node_y += del_y
        new_node_theta += del_theta
        if is_valid_position(new_node_x, new_node_y, Graph_map) == True:
            plt.plot([X_Start, new_node_x], [Y_Start, new_node_y], color = Color, linewidth = 0.75)


def moves(current_node, RPMs, Graph_map):
    Rpm1 = RPMs[0]
    Rpm2 = RPMs[1]
    actions = [[0, Rpm1] , [Rpm1, 0] , [Rpm1, Rpm1] , [0, Rpm2] , [Rpm2, 0] , [Rpm2, Rpm2] , [Rpm1, Rpm2], [Rpm2, Rpm1]]
    new_nodes = []
    rows, columns, _ = Graph_map.shape
    for action in actions:
        new_node, cost = possible_nodes(current_node, action, Graph_map)
        if new_node is not None:  # Check if new_node is not None
            next_x, next_y, next_theta = new_node
            if 0 <= next_x <= columns and 0 <= next_y < rows and np.all(Graph_map[int(next_y), int(next_x)] == [255, 255, 255]) and not visited_check(new_node):
                new_nodes.append((cost, new_node))
    return new_nodes

output = cv2.VideoWriter('a_star_varun_lakshmanan_sai_jagadeesh_muralikrishnan.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 30, (width, height))

def A_star(start_node, goal_node, RPMs):
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
        closed_list.add(tuple(current_node))
        print(current_node)
        
        # If the current node is equal to goal node, then it will break the loop and return the path along with writing the path to the video.
        if heuristic(current_node, goal_node) < 1.5 and current_node[2] == goal_node[2]:
            path = A_star_Backtracting(parent, start_node, current_node, map_visualization, step_count)
            for _ in range(80):
               output.write(map_visualization)
            return path
        
        # If the current node is not equal to goal node, then it will check the possible nodes and add it to the open_list along with visulizing the node exploration.   
        for cost, new_node in moves(current_node, RPMs, Graph_map):
            cost_to_come = cost_list[tuple(current_node)] + cost

            if tuple(new_node) not in cost_list or cost_to_come < cost_list[tuple(new_node)]:
                cost_list[tuple(new_node)] = cost_to_come

                parent[tuple(new_node)] = current_node

                cost_total = cost_to_come + heuristic(new_node, goal_node) 
                open_list.put((cost_total, new_node))
                marking_visited(new_node)
                cv2.arrowedLine(map_visualization, (int(current_node[0]), int(current_node[1])), (int(new_node[0]), int(new_node[1])), (0, 0, 255), 1, tipLength=0.3)
                if step_count % 2000 == 0:
                    output.write(map_visualization)
                step_count += 1
    
    output.release()
    return None
#-------------------------Creating the Matrix using second method----------------------------------#
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

#---------------------------Creating the Backtracking Function--------------------------------------#
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
Xg, Yg = goal_node(width, height, Graph_map) # Getting the goal node from the user
RPMs = RPM_input()

#--------------------------------------Initializing the nodes------------------------------------#
start_node = (Xs, Ys, start_theta)
goal_node = (Xg, Yg)
plt.imshow(Graph_map, origin='lower') #Show Initial Arena Setup
plt.show()

start_time = time.time()   # Starting to check the runtime.
path = A_star(start_node, goal_node, RPMs)

if path is None:
    print("No optimal path found")
else:
    print("Path found")

end_time = time.time()    # end of runtime
print(f'Runtime : {((end_time-start_time)/60):.2f} Minutes')