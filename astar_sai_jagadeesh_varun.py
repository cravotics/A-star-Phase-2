import numpy as np
from queue import PriorityQueue
import cv2
import time

#--------------------------------------------------Creating the Canvas----------------------------------------------------#
height = 2000
width = 6000
Graph_map = np.ones((height, width, 3), dtype=np.uint8)*255
robot_radius = 5
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
        y_transform = 2000 - y

        # Wall clearance.
        if (x <= 0 + Total_clearance or x >= 6000 - Total_clearance or y_transform <= 0 + Total_clearance or y_transform >= 2000 - Total_clearance):
            Graph_map[y,x] = [0,255,0]
        
        # object 1(rectangle)
        if (x >= 1500 and x <= 1750  and y_transform >= 1000 and y_transform <= 2000 ):
            Graph_map[y,x] = [0,0,0]
        elif (x >= 1500 - Total_clearance  and x <= 1750 + Total_clearance and y_transform >= 1000 - Total_clearance and y_transform <= 2000 + Total_clearance):
            Graph_map[y,x] = [0, 255, 0]
        
        # object 2(rectangle)
        if (x >= 2500 and x <= 2750 and y_transform >= 0 and y_transform <= 1000):
            Graph_map[y,x] = [0,0,0]
        elif(x >= 2500 - Total_clearance and x <= 2750 + Total_clearance and y_transform >= 0 - Total_clearance and y_transform <= 1000 + Total_clearance):
             Graph_map[y,x] = [0, 255, 0] 

#--------------------------------------------------Creating the Obstacles-------------------------------------------------#

# object 3(circle) using half planes method 
for x in range(width):
    for y in range(height):
        y_transform = 2000 - y
    
        if ((x-4200)**2 + (y_transform-1200)**2 <= (600 + Total_clearance)**2):
            Graph_map[y, x] = [0,255,0] 

        if ((x-4200)**2 + (y_transform -1200)**2 <= (600)**2):
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


def resize_canvas(canvas, scale_percent):
    """
    Resizes the canvas by a specified percentage, maintaining the aspect ratio.
    
    Args:
    - canvas (np.array): The original canvas to resize.
    - scale_percent (float): The percentage (as a decimal) to scale the canvas. 
      For example, 0.5 for 50% size reduction.
      
    Returns:
    - resized_canvas (np.array): The resized canvas.
    """
    width = int(canvas.shape[1] * scale_percent)
    height = int(canvas.shape[0] * scale_percent)
    dim = (width, height)

    resized_canvas = cv2.resize(canvas, dim, interpolation=cv2.INTER_AREA)
    
    return resized_canvas

# Example usage
scale_percent = 0.5  # For example, to reduce the canvas size to 70% of its original size
resized_canvas = resize_canvas(Graph_map, scale_percent)


# a and b are the coordinates of the center of the circle.
# Function to save the canvas to a file
def save_canvas(canvas, file_name):
    cv2.imwrite(file_name, canvas)
    print(f"Canvas saved to {file_name}")

# Example usage
file_name = "canvas.png"
save_canvas(Graph_map, file_name)

cv2.imshow("Resized Canvas", resized_canvas)
cv2.waitKey(0)
cv2.destroyAllWindows()
