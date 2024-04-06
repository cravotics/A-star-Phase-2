import numpy as np
from queue import PriorityQueue
import cv2
import time
import matplotlib.pyplot as plt

#--------------------------------------------------Creating the Canvas----------------------------------------------------#
height = 200
width = 600

Graph_map = np.ones((height, width, 3), dtype=np.uint8)*255

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
                print("The angle of the start node is out of range or not a multiple of 30.Try again!!!")
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

# User input for step size.
def step_size_function():
    while True:
        try:
            RPM1 = int(input("Enter the RPM 1 : "))
            RPM2 = int(input("Enter the RPM 2  "))
            if 1 <= RPM1 <= 200 and 1 <= RPM2 <= 200:
                return RPM1, RPM2
            else:
                print("The RPMS are nnot above 1 and below 200.")
        except ValueError:
            print("The step size is not a number. Try again!!!")

def print_a_star_ascii():
    print("""
    ___                _____  __                                __   __       ____ _             __           
   /   |              / ___/ / /_ ____ _ _____   ____   ____ _ / /_ / /_     / __/(_)____   ____/ /___   _____
  / /| |    ______    \__ \ / __// __ `// ___/  / __ \ / __ `// __// __ \   / /_ / // __ \ / __  // _ \ / ___/
 / ___ |   /_____/   ___/ // /_ / /_/ // /     / /_/ // /_/ // /_ / / / /  / __// // / / // /_/ //  __// /    
/_/  |_|            /____/ \__/ \__,_//_/     / .___/ \__,_/ \__//_/ /_/  /_/  /_//_/ /_/ \__,_/ \___//_/     
                                             /_/                                                                  """)

print_a_star_ascii()
# User input for radius of the robot.
radius_of_robot = 22
clearance = int(input("Enter the clearance of the obstacles: "))
step_size = 28.7
Total_clearance = radius_of_robot + clearance

# Creating a matrix to store the visited nodes.
G = np.zeros((400, 1200, 12), dtype=np.uint8)

# Creating a cache to store the heuristic values.
heuristic_cache = {}



#---------------------------------------Creating the Rectangles using half planes-------------------------------------------------#
for x in range(600):
    for y in range(200):
        y_transform = abs(200 - y)

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

# object 3(circle) using half planes method 
for x in range(width):
    for y in range(height):
        y_transform = abs(200 - y)
    
        if ((x-420)**2 + (y_transform-120)**2 <= (60 + Total_clearance)**2):
            Graph_map[y, x] = [0,255,0] 

        if ((x-420)**2 + (y_transform -120)**2 <= (60)**2):
            Graph_map[y, x] = [0,0,0]  

# plot the graph map


cv2.imshow("Graph Map", Graph_map)
cv2.waitKey(0)
def is_valid(x, y, Graph_map):
    if x >= 0 and x < width and y >= 0 and y < height:
        # return np.all(Graph_map[int(y), int(x)] == [255, 255, 255])
        print(Graph_map[y, x])
        return all(Graph_map[y, x] == [255, 255, 255])
    else:
        return False
    
def possible_node(current_node, RPM1, RPM2, Graph_map, L = 28.7, R = 3.3):  
    dt = 0.1 
    ul_vel = RPM1*(2*np.pi/60)
    ur_vel = RPM2*(2*np.pi/60)
    Curr_Node_X = current_node[0] #Grab Current Node X
    Curr_Node_Y = current_node[1] #Grad Current Node Y
    Curr_Node_Theta = np.deg2rad(current_node[2]) #Grab Current Node Theta, convert to radians.

    distance_cost = 0.0 #Init Cost
    actions = [(0, ul_vel), (ul_vel,0) ,(ul_vel,ul_vel), (0,ur_vel), (ur_vel,0), (ur_vel,ur_vel), (ul_vel,ur_vel), (ur_vel,ul_vel)] #All Possible Actions
    for action in actions:
        ul, ur = action
        t = 0
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
            

        next_x = int(round(next_x))  # Convert to integer
        next_y = int(round(next_y))  # Convert to integer
        next_theta = np.rad2deg(next_theta)

        if (next_theta < 0):
             next_theta += 360
        if (next_theta >= 360):
             next_theta -= 360
        next_theta = int(round(next_theta))

        print(next_x, next_y, next_theta)

        # Ensure next_x and next_y are within bounds
        if 0 <= next_x < width and 0 <= next_y < height:
            if is_valid(next_x, next_y, Graph_map) and not visited_check((next_x, next_y, next_theta)):
                distance_cost = np.sqrt((current_node[0] - next_x)**2 + (current_node[1] - next_y)**2)
                yield (distance_cost, (next_x, next_y, next_theta))

#-----------------------------------------Creating the Video File-------------------------------------------------#
# Creating a video file to store the output.
output = cv2.VideoWriter('a_star_varun_lakshmanan_sai_jagadeesh_muralikrishnan.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 30, (width, height))

#------------------------------------Creating the Heuristic Function--------------------------------#
def heuristic(node, goal):
    if node in heuristic_cache:
        return heuristic_cache[node]
    else:
        heuristic_value = np.sqrt((node[0] - goal[0])**2 + (node[1] - goal[1])**2)
        heuristic_cache[node] = heuristic_value
        return heuristic_value

#----------------------------------------- the A* Algorithm-----------------------------------------#
def A_star(start_node, goal_node, RPM1, RPM2):
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
        if heuristic(current_node, goal_node) < 3.0:
            path = A_star_Backtracting(parent, start_node, current_node, map_visualization, step_count)
            for _ in range(80):
               output.write(map_visualization)
            return path
        
        # If the current node is not equal to goal node, then it will check the possible nodes and add it to the open_list along with visulizing the node exploration.   
        for cost, new_node in possible_node(current_node, RPM1, RPM2, map_visualization):
            cost_to_come = cost_list[current_node] + cost
            if new_node not in cost_list or cost_to_come < cost_list[new_node]:
                cost_list[new_node] = cost_to_come
                parent[new_node] = current_node
                cost_total = cost_to_come + heuristic(new_node, goal_node) 
                open_list.put((cost_total, new_node))
                marking_visited(new_node)
                map_visualization[new_node[1], new_node[0]] = [0, 0, 255]
                if step_count % 100 == 0:
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
    if 0 <= i < 400 and 0 <= j < 600: 
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
        cv2.circle(map_visualization, start_point, 1, (0, 255, 0), 1) # Drawing the path.
        if step_count % 5 == 0:
            output.write(map_visualization)
        step_count += 1 
    return path
    
Xs, Ys, start_theta = start_node(width, height, Graph_map) # Getting the start node from the user
Xg, Yg = goal_node(width, height, Graph_map) # Getting the goal node from the user


#--------------------------------------Initializing the nodes------------------------------------#
start_node = (Xs, Ys, start_theta)
goal_node = (Xg, Yg)
RPM1, RPM2 = step_size_function()

start_time = time.time()   # Starting to check the runtime.
path = A_star(start_node, goal_node, RPM1, RPM2)

if path is None:
    print("No optimal path found")
else:
    print("Path found")

end_time = time.time()    # end of runtime
print(f'Runtime : {((end_time-start_time)/60):.2f} Minutes')

#--------------------------------------------------End of the Program-------------------------------------------------#