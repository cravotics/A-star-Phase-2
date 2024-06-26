#!/usr/bin/env python3
import numpy as np
import rclpy
from queue import PriorityQueue
import cv2
import time
import matplotlib.pyplot as plt
from rclpy.node import Node
from geometry_msgs.msg import Twist

# to publish the velocities to the robot

class Publisher(Node):
    def __init__(self): # Initializing the publisher
        super().__init__('publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10) # Creating the publisher
        time.sleep(2)

    def publish_feed(self, linear_velocity, angular_vel):
        msg = Twist()
        msg.linear.x = linear_velocity
        msg.angular.z = angular_vel
        self.publisher_.publish(msg)# Publishing the message
        self.get_logger().info(
            f'ros2 topic pub \\cmd_vel geometry_msgs/msg/Twist "linear:\n'
            f'  x: {msg.linear.x}\n'
            f'  y: {msg.linear.y}\n'
            f'  z: {msg.linear.z}\n'
            f'angular:\n'
            f'  x: {msg.angular.x}\n'
            f'  y: {msg.angular.y}\n'
            f'  z: {msg.angular.z}"'
        )
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
            Xs = int(input("Enter the x-coordinate of the start node(Xs) in cm: "))
            start_y = int(input("Enter the y-coordinate of the start node(Ys) in cm: "))
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
            Xg = int(input("Enter the x-coordinate of the goal node(Xg)in cm: "))
            goal_y = int(input("Enter the y-coordinate of the goal node(Yg) in cm: "))
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
            RPM2 = int(input("Enter the RPM 2 :  "))
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
                                             /_/                                                                  
                                                |--[]--|
                                                |      |
                                          WITH  |      | WAFFLE BOT
                                                |______|
                                                  \__/
          Hello, welcome to our A* Algorithm implementation.
          This program will help you to find the optimal path from start node to goal node.
          The program will ask you to enter the start node and goal node along with the RPM values.""")

print_a_star_ascii()
# User input for radius of the robot.
radius_of_robot = 22.0
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

#--------------------------------------Checking the Validity of the Node-----------------------------------------------#
def is_valid(x, y, Graph_map):
    if x >= 0 and x < width and y >= 0 and y < height:
        # return np.all(Graph_map[int(y), int(x)] == [255, 255, 255])
        return all(Graph_map[y, x] == [255, 255, 255])
    else:
        return False
    
def possible_node(current_node, RPM1, RPM2, Graph_map, L = 28.7, R = 3.3):  
    dt = 0.1 
    # Getting the current RPM values in rad/s
    ul_vel = RPM1*R*(2*np.pi/60)
    ur_vel = RPM2*R*(2*np.pi/60)
    currn_node_x = current_node[0] 
    currn_node_y = current_node[1] 
    currn_node_theta = np.deg2rad(current_node[2]) 
    distance_cost = 0.0 
    actions = [(0, ul_vel), (ul_vel,0) ,(ul_vel,ul_vel), (0,ur_vel), (ur_vel,0), (ur_vel,ur_vel), (ul_vel,ur_vel), (ur_vel,ul_vel)] 
    for action in actions:
        ul, ur = action
        t = 0
        velocities = []
        next_x = currn_node_x 
        next_y = currn_node_y 
        next_theta = currn_node_theta 
#--------------------------------------Calculating the next node using the kinematic model---------------------------------#
        while t < 1:
            t += dt
            # Calculating the linear and angular velocities
            linear_vel = 0.5*(ul+ur)
            angular_vel = (ul-ur)/(L)
            del_x = 0.5*(ul+ur)*np.cos(next_theta)*dt
            del_y = 0.5*(ul+ur)*np.sin(next_theta)*dt
            del_theta = (1/L)*(ur-ul)*dt
            velocities.append(((linear_vel), (angular_vel)))
            next_x += del_x
            next_y += del_y
            next_theta += del_theta
            

        next_x = int(round(next_x))  
        next_y = int(round(next_y))  
        next_theta = np.rad2deg(next_theta)
#--------------------------------------Checking the next node is within the bounds---------------------------------#
        if (next_theta < 0):
             next_theta += 360
        if (next_theta >= 360):
             next_theta -= 360
        next_theta = int(round(next_theta))

        if 0 <= next_x < width and 0 <= next_y < height:
            if is_valid(next_x, next_y, Graph_map) and not visited_check((next_x, next_y, next_theta)):
                distance_cost = np.sqrt((current_node[0] - next_x)**2 + (current_node[1] - next_y)**2)
                cv2.line(Graph_map, (current_node[0], current_node[1]), (next_x, next_y), (37, 78, 66), 1)
                print("Distance Cost: ", distance_cost, "Next X: ", next_x, "Next Y: ", next_y, "Next Theta: ", next_theta)
                yield (distance_cost, (next_x, next_y, next_theta), velocities)

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
        if heuristic(current_node, goal_node) < 1.5:
            all_vels = A_star_Backtracting(parent, start_node, current_node, map_visualization, step_count)
            for _ in range(80):
               output.write(map_visualization)
            return all_vels, parent
        
        # If the current node is not equal to goal node, then it will check the possible nodes and add it to the open_list along with visulizing the node exploration.   
        for cost, new_node,velocities in possible_node(current_node, RPM1, RPM2, map_visualization): # Getting the possible nodes
            cost_to_come = cost_list[current_node] + cost
            if new_node not in cost_list or cost_to_come < cost_list[new_node]:
                cost_list[new_node] = cost_to_come
                parent[new_node] = (current_node, velocities)
                cost_total = cost_to_come + heuristic(new_node, goal_node) 
                open_list.put((cost_total, new_node))
                marking_visited(new_node)
                if step_count % 100 == 0:
                    output.write(map_visualization)
                step_count += 1
    
    output.release()
    return None, None
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
    parent_and_velocities = [(end_node, parent[end_node][1] if end_node in parent else (0, 0))]
    while end_node != start_node: # If the end node is not equal to start_node, parent of the end_node is added to path and continues.
        temp_nodes = parent[end_node] # The parent of end node becomes the current node.
        parent_node, velocity= temp_nodes
        end_node = parent_node
        parent_and_velocities.append((end_node, velocity))
        
    parent_and_velocities.reverse()
    all_vels = []
    for i in range(len(parent_and_velocities) - 1):
        start_point, init_vel = ((parent_and_velocities[i][0][0]), parent_and_velocities[i][0][1]), parent_and_velocities[i][1]
        end_point = (parent_and_velocities[i + 1][0][0]), (parent_and_velocities[i + 1][0][1])
        x_start, y_start = start_point
        print("Start Point: ", x_start, y_start)
        x_end, y_end = end_point
        print("Start Point: ", x_end, y_end)
        cv2.arrowedLine(map_visualization, start_point, end_point, (255, 255, 0), 1, tipLength=0.2)
        for lin_vel, ang_vel in init_vel:
            all_vels.append((lin_vel/100, ang_vel))
           
            if step_count % 5 == 0:
                output.write(map_visualization)
            step_count += 1 
    return all_vels
    
Xs, Ys, start_theta = start_node(width, height, Graph_map) # Getting the start node from the user
Xg, Yg = goal_node(width, height, Graph_map) # Getting the goal node from the user


#--------------------------------------Initializing the nodes------------------------------------#
start_node = (Xs, Ys, start_theta)
goal_node = (Xg, Yg)
RPM1, RPM2 = step_size_function()

start_time = time.time()   # Starting to check the runtime.
all_vels, parents = A_star(start_node, goal_node, RPM1, RPM2)

if parents is None:
    print("No optimal path found")
else:
    print("Path found for the waffle bot")

end_time = time.time()    # end of runtime
print(f'Runtime : {((end_time-start_time)/60):.2f} Minutes')

#--------------------------------------Creating the main function------------------------------------#
def main(all_vels):
    rclpy.init()
    publisher = Publisher()# Creating the publisher
    try:
        for vel in all_vels:
            lin_vel, ang_vel = vel # Getting the linear and angular velocities
            publisher.publish_feed(lin_vel, ang_vel)
            time.sleep(0.1)
        publisher.publish_feed(0.0,0.0)# Stopping the robot
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main(all_vels)

#--------------------------------------------------End of the Program-------------------------------------------------#