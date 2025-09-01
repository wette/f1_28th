import math
import cv2 as cv
import numpy as np

def distance(point1, point2):
    dx = point1[0] - point2[0]
    dy = point1[1] - point2[1]

    return math.sqrt(dx**2 + dy**2)

#get yaw from two 2d points (pointing from point1 to point2)
def getyaw(point1, point2):
    a = point2[1] - point1[1]
    b = point2[0] - point1[0]
    yaw = -1*math.atan2(b, a)  + math.pi/2.0
    return yaw

#rotate vector by angle
def rotate(x: float, y: float, alpha:float) -> tuple[float, float]:
    xrot = x*math.cos(alpha) - y*math.sin(alpha)
    yrot = x*math.sin(alpha) + y*math.cos(alpha)
    return xrot, yrot

#h is hue in range 0 to 180
#returns bgr in rage 0 to 255
def hueToBGR(h: float):
    color = cv.cvtColor(np.uint8([[np.array([int(h), 255,  255], dtype=np.uint8)]]), cv.COLOR_HSV2BGR)
    return int(color[0][0][0]), int(color[0][0][1]), int(color[0][0][2])

#shift angle in range [-pi, +pi]
def unwind_angle(alpha_rad : float, start=-math.pi, end=math.pi) -> float:
    while alpha_rad < start:
        alpha_rad += math.pi*2
    while alpha_rad > end:
        alpha_rad -= math.pi*2
    
    return alpha_rad

def get_average_angle(angles_rad: float) -> float:
    x = 0.0
    y = 0.0
    for alpha in angles_rad:
        x += math.cos(alpha)
        y += math.sin(alpha)
    average_angle_rad = -1*math.atan2(x,y)  + math.pi/2.0
    return average_angle_rad

def difference_in_angle(alpha_rad, beta_rad):
    alpha_rad = unwind_angle(alpha_rad, 0, 2*math.pi)
    beta_rad = unwind_angle(beta_rad, 0, 2*math.pi)
    diff = abs(alpha_rad-beta_rad)
    diff_alt = 2*math.pi - diff
    return unwind_angle(min(diff, diff_alt), -math.pi, math.pi)

def createPlot(frame, 
               plot_x:int, 
               plot_y:int, 
               plot_height:int, 
               title:str, 
               colors : list[list[int]], 
               values : list[list[float]], 
               dx=3,
               moving_average=1):
    dy = plot_height

    cv.putText(frame, title, (plot_x, plot_y-dy-10), cv.FONT_HERSHEY_SIMPLEX, 0.5,
            colors[0], 1, cv.LINE_AA)

    numMaxValues = len(values[0])

    #draw background(semi transparent):
    # First we crop the sub-rect from the image
    x, y, w, h = plot_x, plot_y-dy, dx*numMaxValues, dy
    sub_img = frame[y:y+h, x:x+w]
    white_rect = np.ones(sub_img.shape, dtype=np.uint8) * 255

    res = cv.addWeighted(sub_img, 0.3, white_rect, 0.7, 1.0)

    # Putting the image back to its position
    frame[y:y+h, x:x+w] = res


    #draw x and y axis:
    cv.line(frame, [plot_x, plot_y], [plot_x+dx*numMaxValues, plot_y], (255,255,255), 1) #x-axis
    cv.line(frame, [plot_x, plot_y], [plot_x, plot_y-dy], (255,255,255), 1) #y-axis

    #plot actual data
    for j in range(0, len(values)):
        for i in range(0, len(values[0])-1-moving_average):
            last_val = int(sum(values[j][i : i+moving_average])/moving_average)
            act_val = int(sum(values[j][i+1 : i+1+moving_average])/moving_average)
            cv.line(frame, 
                    [plot_x + i*dx, plot_y - last_val], 
                    [plot_x + (i+1)*dx, plot_y - act_val], 
                    colors[j], 
                    1)

    return frame