import numpy as np
import cv2 as cv
import time
import colorsys

from .helper_functions import *
from .vehicle import Vehicle

class Camera:
    #Hue from (HSV model) - HSV in opencv is from 0-179 - not 0-359 as per default
    ColorMap = {"green" : 120/2,
                "blue"  : 240/2,
                "yellow" : 60/2,
                "red"    : 360/2,
                #"orange" : 30/2,
                }
    
    def __init__(self,
                 vertical_resolution_px = 1200,
                 horizontal_resolution_px = 1920,
                 frames_per_seconds = 30,#90,
                 opening_angle_vertical_degrees = 88.0,
                 opening_angle_horizontal_degrees = 126.0,
                 meters_to_pixels = 666, #1075,
                 max_speed_vehicle_mps = 2.0,
                 minimum_brightness = 1.2, #2.7,
                 threshold_brightness_of_black = 150,
                 threshold_brightness_of_white = 200,
                 circle_diameter_meters = 0.023,
                 size_between_black_and_white_center_meters = 0.08,
                 height_over_ground_black_meters = 0.042,
                 height_over_ground_white_meters = 0.025,
                 cameraMatrix = np.array([[1.19164513e+03, 0.00000000e+00, 9.32255365e+02],
                             [0.00000000e+00, 1.19269246e+03, 5.44789222e+02],
                            [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]),
                 distortionCoefficients = np.array([[ 0.02473071, -0.39668063,  0.00151336,  0.00085757,  0.25759047]]),
                 create_debug_video=False,
                 from_file=None
                 ):
        
        if from_file is None:
            #self.cap = cv.VideoCapture(0) #use default camera driver (might not support 90fps)
            self.cap = cv.VideoCapture(0, cv.CAP_V4L) #use V4L to access the camera
        else:
            self.cap = cv.VideoCapture(from_file)
        
        if not self.cap.isOpened():
            print("Cannot open camera")
            exit()

        self.DEBUG = True

        #camera stream properties
        self.vertical_resolution_px = vertical_resolution_px
        self.horizontal_resolution_px = horizontal_resolution_px
        self.frames_per_seconds = frames_per_seconds
        self.opening_angle_vertical_degrees = opening_angle_vertical_degrees
        self.opening_angle_horizontal_degrees = opening_angle_horizontal_degrees
        
        #physical scenario: configuration variables
        self.meters_to_pixels = meters_to_pixels        #how many pixels are in one meter?
        self.max_speed_vehicle_mps = max_speed_vehicle_mps    #max speed of a car in meters per second
        self.minimum_brightness = minimum_brightness       #used to brighten the image of the webcam
        self.threshold_brightness_of_black = threshold_brightness_of_black
        self.threshold_brightness_of_white = threshold_brightness_of_white
        self.size_between_black_and_white_center_px = size_between_black_and_white_center_meters * self.meters_to_pixels #8cm from center to center
        self.height_over_ground_black_meters = height_over_ground_black_meters #how high is the black dot on the vehicle measured from the ground
        self.height_over_ground_white_meters = height_over_ground_white_meters #how high is the white dot on the vehicle measured from the ground
        self.circle_diameter_px = circle_diameter_meters * self.meters_to_pixels  #diameter of black and white dots (2cm)

        #debug video saver
        self.video_out = None
        self.create_debug_video = create_debug_video
        if create_debug_video:
            self.setup_debug_video()

        
        self.frames_to_save = []
        


        #intrinsic camera calibration
        #Camera matrix : 
        self.cameraMatrix = cameraMatrix

        #Distortion coefficients : 
        self.distortionCoefficients = distortionCoefficients

        #apply camera correction
        h,  w = self.horizontal_resolution_px, self.vertical_resolution_px
        newcameramtx, _ =cv.getOptimalNewCameraMatrix(self.cameraMatrix,self.distortionCoefficients,(w,h),1,(w,h))
        self.remapX, self.remapY = cv.initUndistortRectifyMap(self.cameraMatrix,self.distortionCoefficients, None, newcameramtx,(h, w),5)


        #state tracked by the camera
        self.tracked_vehicles : list[Vehicle] = []  #vehicles found by the camera
        self.color_correct_ratio : float = 1.0
        self.current_time : float = 0
        self.current_frame = None

        #for calculating FPS
        self.time_end = 0
        self.fps_buffer = []

        self.__setup_video_stream__()

        self.video_stream_active = True

    def setup_debug_video(self):
        self.create_debug_video = True
        fourcc = cv.VideoWriter_fourcc(*'XVID')
        self.video_out = cv.VideoWriter('last_video.avi', fourcc, self.frames_per_seconds, (self.horizontal_resolution_px,self.vertical_resolution_px))

    def is_video_stream_active(self):
        return self.video_stream_active
    
    def get_last_frame(self):
        return self.current_frame

    def get_frame(self, initializeColorCorrection=False, colorCorrect=True):
        ret, frame = self.cap.read()
        
        # if frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            self.video_stream_active = False
            return None
        
        if self.create_debug_video:
            self.frames_to_save.append(frame)

        #apply camera correction to frame
        frame = cv.remap(frame, self.remapX, self.remapY, cv.INTER_LINEAR)

        #color correct the image
        if colorCorrect:
            frame = self.colorCorrectImage(frame, initializeRatio=initializeColorCorrection)

        return frame

    #configure video stream using V4L
    def __setup_video_stream__(self):
        self.cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M', 'J', 'P', 'G')) # depends on fourcc available camera
        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, self.horizontal_resolution_px)
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, self.vertical_resolution_px)
        self.cap.set(cv.CAP_PROP_FPS, self.frames_per_seconds)

    #destructor
    def __del__(self):
        # When everything done, release the capture
        self.cap.release()

        #save frames to video file:
        if self.create_debug_video:
            print("Saving video to file....")
            for frame in self.frames_to_save:
                self.video_out.write(frame)
            print("done.")
            self.video_out.release()
        cv.destroyAllWindows()

    #display visual aid to adjust pitch, yaw, and roll of physical camera mounted above racetrack
    #also computes the correct value for meters_to_pixels
    def cameraCalibrationWizard(self):
        print("position the four 3d-printed chessboard pieces in the four corners of the racetrack.")
        print("press any key if done.")
        input()

        print("Starting calibration process.\nIf you are happy with the result, exit routine by pressing q.")

        while True:
            # read frame
            frame = self.get_frame(initializeColorCorrection=True)
        
            # if frame is read correctly ret is True
            if frame is None:
                print("Can't receive frame (stream end?). Exiting ...")
                self.video_stream_active = False
                break

            # Our operations on the frame come here
            gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            #gray = cv.medianBlur(gray, 5)

            #find chessboard in each corner

            h, w = gray.shape[:2]
            h2, w2 = h // 2, w // 2

            # Dividing image in quadrants subimages with offsets clockwise starting top left
            regions = [
                (gray[0:h2, 0:w2], 0, 0),
                (gray[0:h2, w2:w], 0, w2),
                (gray[h2:h, w2:w], h2, w2),
                (gray[h2:h, 0:w2], h2, 0),
            ]

            regionCenters = []
            for img, xoff, yoff in regions:
                ret, corners = cv.findChessboardCorners(img, (3, 3), None)

                if ret:
                    regionCenters.append( (int(corners[4][0][0] + yoff),
                                           int(corners[4][0][1] + xoff)) )
                    
                    cv.drawChessboardCorners(frame, (3,3), np.array([[[c[0][0]+yoff, c[0][1]+xoff]] for c in corners], dtype=np.float32), ret)

            if len(regionCenters) != 4:
                print(f"Could not find all chessboard patterns ({len(regionCenters)}/4 detected)")
                cv.imshow('frame', frame)
                cv.waitKey(1)
                continue

            #compute distances between markers to find out if camera is mounted in parallel to the racetrack
            distHorizontalTop    = math.sqrt((regionCenters[0][0] - regionCenters[1][0])**2 + \
                                             (regionCenters[0][1] - regionCenters[1][1])**2)
            
            distHorizontalBottom = math.sqrt((regionCenters[2][0] - regionCenters[3][0])**2 + \
                                             (regionCenters[2][1] - regionCenters[3][1])**2)
            
            distVerticalLeft     = math.sqrt((regionCenters[0][0] - regionCenters[3][0])**2 + \
                                             (regionCenters[0][1] - regionCenters[3][1])**2)
            
            distVerticalRight    = math.sqrt((regionCenters[1][0] - regionCenters[2][0])**2 + \
                                             (regionCenters[1][1] - regionCenters[2][1])**2)
            
            diffHorizontal = abs( distHorizontalTop - distHorizontalBottom )
            diffVertical   = abs( distVerticalLeft  - distVerticalRight )

            colorHueHorizontal = Camera.ColorMap["green"]
            colorHueVertical   = Camera.ColorMap["green"]
            
            if diffHorizontal > 10:
                colorHueHorizontal = Camera.ColorMap["yellow"]
            if diffHorizontal > 20:
                colorHueHorizontal = Camera.ColorMap["red"]
            if diffVertical > 10:
                colorHueVertical = Camera.ColorMap["yellow"]
            if diffVertical > 20:
                colorHueVertical = Camera.ColorMap["red"]

            #convert hsv to rgb:
            colorHueVertical = colorsys.hsv_to_rgb(colorHueVertical/360, 1, 1)
            colorHueHorizontal = colorsys.hsv_to_rgb(colorHueHorizontal/360, 1, 1)
            #convert rgb to bgr
            colorHueVertical = (colorHueVertical[2]*255, colorHueVertical[1]*255, colorHueVertical[0]*255)
            colorHueHorizontal = (colorHueHorizontal[2]*255, colorHueHorizontal[1]*255, colorHueHorizontal[0]*255)

            diffVertical = min(max(int(diffVertical), 1), 100)
            diffHorizontal = min(max(int(diffHorizontal), 1), 100)

            #color in values
            cv.line(frame, regionCenters[0], regionCenters[3], color=colorHueVertical, thickness=diffVertical)
            cv.line(frame, regionCenters[1], regionCenters[2], color=colorHueVertical, thickness=diffVertical)

            cv.line(frame, regionCenters[0], regionCenters[1], color=colorHueHorizontal, thickness=diffHorizontal)
            cv.line(frame, regionCenters[3], regionCenters[2], color=colorHueHorizontal, thickness=diffHorizontal)

            # Display the resulting frame
            cv.imshow('frame', frame)
            if cv.waitKey(1) == ord('q'):

                #calibrate pixel to meters by using the cross measurements (top-left to bottom-right corners)
                dist1    = math.sqrt((regionCenters[0][0] - regionCenters[2][0])**2 + \
                                     (regionCenters[0][1] - regionCenters[2][1])**2)
                dist2    = math.sqrt((regionCenters[1][0] - regionCenters[3][0])**2 + \
                                     (regionCenters[1][1] - regionCenters[3][1])**2)
                
                meanDist = (dist1+dist2)/2

                #how many pixels are in a meter?
                real_world_distance_across_meters = 2.36 #measured on racetrack from chessboard corner to chessboard corner
                self.meters_to_pixels = meanDist / real_world_distance_across_meters
                print("################")
                print(f"meters_to_pixels is {self.meters_to_pixels}. Adjust constant in code acordingly!!!!!!!!")
                print("################")

                return



    #project a point seen at a known height to the floor
    def correctForHeightOfVehicle(self, xPos: int, yPos: int, height_meters: float):

        #find out in which angle the object is seen
        #as an approximation: use linear interpolation
        x_distance_from_center = int( xPos - self.horizontal_resolution_px/2)
        y_distance_from_center = int( yPos - self.vertical_resolution_px/2)

        #linear interpolation
        x_angle_deg = (x_distance_from_center/ (self.horizontal_resolution_px/2) ) * self.opening_angle_horizontal_degrees/2
        y_angle_deg = (y_distance_from_center/ (self.vertical_resolution_px/2) )   * self.opening_angle_vertical_degrees/2


        #compute correction for height as projection into the floor
        tanx = math.tan(math.radians(90.0 - x_angle_deg))
        tany = math.tan(math.radians(90.0 - y_angle_deg))
        if tanx == 0:
            tanx = 0.00001
        if tany == 0:
            tany = 0.00001
        correction_x_meters = height_meters/tanx
        correction_y_meters = height_meters/tany

        #convert back to pixels
        correction_x_px = correction_x_meters * self.meters_to_pixels
        correction_y_px = correction_y_meters * self.meters_to_pixels

        #print(f"x, und y, winkel der Kamera: {x_angle_deg: .3f}, {y_angle_deg: .3f}")
        #print(f"x, und y, Korrektur: {correction_x_meters: .3f}, {correction_y_meters: .3f}")

        return xPos - correction_x_px, yPos - correction_y_px

    def updateVehiclePosition(self, xPos : int, yPos : int, yaw : float, vehicle: Vehicle = None, detect_new_vehicles: bool = True, color: str = None):
        threshold = (self.max_speed_vehicle_mps * self.meters_to_pixels) / self.frames_per_seconds

        if vehicle is not None:
            #check if what we found is remotely near the vehicle we want to update
            if distance( (xPos, yPos), list(vehicle.getPosition()) ) < threshold and \
                (abs((vehicle.getOrientation() - yaw)) < 0.3 or abs((vehicle.getOrientation() - yaw)) > (2*math.pi - 0.3)):

                vehicle.updatePose(xPos, yPos, yaw, self.current_time)
                return True
        else:
            if detect_new_vehicles:
                #only detect new vehicle ifs "far" away from other vehicles:
                min_dist_px = 0.1 * self.meters_to_pixels
                d = None
                for v in self.tracked_vehicles:
                    dist = distance(list(v.getPosition()), (xPos, yPos))
                    if d is None or d > dist:
                        d = dist

                if d is not None and d < min_dist_px:
                    return False
                
                vehicle = Vehicle(xPos, yPos, yaw, self.meters_to_pixels)
                vehicle.color = color
                if self.DEBUG: print(f"Found new Vehicle at {vehicle.getPosition()}, yaw angle {math.degrees(vehicle.getOrientation())}")
                self.tracked_vehicles.append(vehicle)
                return True
        return False



    def colorCorrectImage(self, frame, initializeRatio=False):
        #return frame
        if initializeRatio:
            cols, rows, _ = frame.shape
            brightness = np.sum(frame) / (255 * cols * rows)
            
            self.color_correct_ratio = brightness / self.minimum_brightness
        frame = cv.convertScaleAbs(frame, alpha = 1 / self.color_correct_ratio, beta = 0)
        
        return frame
    
    def classifyDotsFromCircles(self, circles, frame):
        black_dots = []
        white_dots = []

        # loop over the (x, y) coordinates and radius of the circles
        for (x, y, r) in circles:
            
            if x < frame.shape[1] and y < frame.shape[0]:
                color = frame[y,x]

                #black
                thres = self.threshold_brightness_of_black
                if color[0] < thres and color[1] < thres and color[2] < thres:
                    #black dot found
                    black_dots.append( [x,y] )
                    #cv.circle(frame, (x, y), r, (0, 0, 0), 4)
                    #cv.rectangle(frame, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
                    #print(f"Found black Dot at {x}, {y}")
                
                #white
                thres = self.threshold_brightness_of_white
                if color[0] > thres and color[1] > thres and color[2] > thres:
                    #white dot found
                    white_dots.append( [x,y] )
                    #cv.circle(frame, (x, y), r, (255, 255, 255), 4)
                    #cv.rectangle(frame, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
                    #print(f"Found white Dot at {x}, {y}")
        return black_dots, white_dots, frame
    
    #figure out color of a vehicle at a given position
    def getColorOfVehicle(self, frame, x: int, y: int, yaw: float) -> str:
        #assumption: black circle is surrounded by vehicle color.
        # hence, add a vector of size 1cm to the vehicle position (position of black circle) pointing in yaw direction
        """x_a, y_a = rotate(x=0.01*self.meters_to_pixels + self.circle_diameter_px/2.0, y=0, alpha=yaw)
        x = int(x+x_a)
        y = int(y+y_a)"""
        #new way: use the color of the darker "black" marker to determine vehicle color.
        if 0 < x < frame.shape[1] and 0 < y < frame.shape[0]:
                color = frame[y,x]
                color_hsv = cv.cvtColor(np.uint8([[color]]), cv.COLOR_BGR2HSV)
                hue = float(color_hsv[0][0][0])
                threshold = 20
                for name, val in Camera.ColorMap.items():
                    #check for modulo 180 in range...
                    diff = abs(val-hue)
                    if diff > 180/2:
                        diff = 180-diff

                    if diff < threshold:  
                        return name
                    
                print(f"Error: Could not figure out which color the vehicle has: {color} (hue {hue}) at ({x},{y})")
                return None

        print("Error: Vehicle out of bounds: ({x},{y})")
        return None

    def detectVehicles(self):
        stopDetectionTime = time.time() + 0.5 # 1 seconds to detect vehicles

        while stopDetectionTime > time.time():
            # Capture frame-by-frame
            frame = self.get_frame(initializeColorCorrection=True)
            self.current_time = time.time()

            # if frame is read correctly ret is True
            if frame is None:
                print("Can't receive frame (stream end?). Exiting ...")
                self.video_stream_active = False
                break

            if self.DEBUG: print(f"new frame at {self.current_time}-----------------------")

           
            self.current_frame = frame

            # Our operations on the frame come here
            gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            gray = cv.medianBlur(gray, 5)

            #find chessboard
            #ret, corners = cv.findChessboardCorners(gray, (chessboard_n, chessboard_m), None)
            #if ret:
            #    cv.drawChessboardCorners(frame, (chessboard_n, chessboard_m), corners, ret)


            # detect circles in the image
            #circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT_ALT, 1.5, minDist=30, param1=300, param2=0.8, minRadius=int((self.circle_diameter_px/2.0)*0.9), maxRadius=int((self.circle_diameter_px/2.0)*1.1))
            circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT_ALT, 1.5, minDist=20, param1=300, param2=0.8, minRadius=int((self.circle_diameter_px/2.0)*0.5), maxRadius=int((self.circle_diameter_px/2.0)*1.5))
            
            # ensure at least some circles were found
            if circles is not None:
                # convert the (x, y) coordinates and radius of the circles to integers
                circles = np.round(circles[0, :]).astype("int")

                #black_dots, white_dots, frame = self.classifyDotsFromCircles(circles, frame)

                #find all darker ("black") circles which is close to a white one
                threshold = 0.4
                for dot1 in circles:
                    #cv.circle(frame, (dot1[0], dot1[1]), dot1[2], (0, 0, 0), 4)
                    for dot2 in circles:
                        if dot1[0] == dot2[0] and dot1[1] == dot2[1] :
                            continue
                        minDist = self.size_between_black_and_white_center_px * (1-threshold)
                        maxDist = self.size_between_black_and_white_center_px * (1+threshold)
                        if minDist < distance(dot1, dot2) < maxDist:
                            #black and white match: we found a vehicle.
                            color1 = gray[dot1[1],dot1[0]]
                            color2 = gray[dot2[1],dot2[0]]

                            if abs(float(color1) -float(color2)) < 20:
                                #two circles of same color
                                continue

                            black = dot1.copy()
                            white = dot2.copy()
                            if color1 > color2:
                                black = dot2.copy()
                                white = dot1.copy()
                            #black and white match: we found a vehicle.
                            color = self.getColorOfVehicle(frame, int(black[0]), int(black[1]), getyaw(black, white))
                            if color is not None:
                                self.updateVehiclePosition(black[0], black[1], getyaw(black, white), color=color)
                            else:
                                print("Not tracking vehicle, as its color is unknown.")
                        #else:
                        #    if self.DEBUG: print(f"Distance too far between black and white circle: {distance(dot1, dot2)}")

            # Display the resulting frame
            #cv.imshow('frame', frame)
            #cv.imshow('frame', gray)
            #if cv.waitKey(1) == ord('q'):
            #    break

        return len(self.tracked_vehicles)

    #extract part of image which contains all points from points
    def getSubimageFromPoints(self, points, frame):
        xs = []
        ys = []
        for p in points:
            xs.append(p[0])
            ys.append(p[1])

        margin_px = 100 #expand image by amount of pixels in any direction
        xstart = int(max(min(xs)-margin_px, 0))
        xend   = int(min(max(xs)+margin_px, frame.shape[1]))
        ystart = int(max(min(ys)-margin_px, 0))
        yend   = int(min(max(ys)+margin_px, frame.shape[0]))

        return frame[ystart:yend, xstart:xend], xstart, ystart

    @staticmethod
    def drawBoundingBox(frame, boundingbox, color=(0, 255, 0)):
        for i in range(len(boundingbox)):
            from_pt = boundingbox[i]
            to_pt = boundingbox[0]
            if i + 1 < len(boundingbox):
                to_pt = boundingbox[i+1]
                
            cv.line(frame, [int(x) for x in from_pt], [int(x) for x in to_pt], color)

    #if in simulation, use dt for the inter frame time of the camera (1.0/fps) to be independent of the actual walltime.
    def trackVehicles(self, dt=None, colorCorrect=False):
    
        # Capture frame-by-frame
        frame = self.get_frame(initializeColorCorrection=False, colorCorrect=colorCorrect)
        if frame is None:
            print("End-of-Stream detected. Stop tracking!")
            self.video_stream_active = False
            return
        
        if dt is not None:
            self.current_time += dt
        else:
            self.current_time = time.time()

        if self.DEBUG: print("new frame -----------------------")

        #apply camera correction to frame
        self.current_frame = frame
        
        time_for_one_pass = -1
    
        # if frame is read correctly ret is True

        for vehicle in self.tracked_vehicles:

            updatedVehicle = False

            #estimate image boundaries where to search for the vehicle
            #there will probably be an offset which we need to track to convert back to full-image-coordinates
            boundingbox = vehicle.getBoundingBox(self.current_time + 1.0/self.frames_per_seconds)
            predicted_vehicle_position = vehicle.getPositionEstimate(self.current_time + 1.0/self.frames_per_seconds)

            subimage, x_offset, y_offset = self.getSubimageFromPoints(boundingbox, frame)

            if subimage.shape[0] < 2 or subimage.shape[1] < 2:
                #we lost a vehicle!
                print(f"lost vehicle {vehicle.color}, as it drove out-of-frame.")
                self.tracked_vehicles.remove(vehicle)
                continue


            #subimage_color = self.colorCorrectImage(subimage, initializeRatio=False)
            subimage_color = subimage

            subimage_color = cv.medianBlur(subimage_color, 5)
            #subimage_color = cv.medianBlur(subimage_color, 5) #2nd time blur
            subimage_gray = cv.cvtColor(subimage_color, cv.COLOR_BGR2GRAY)

            # detect circles in the subimage
            circles = cv.HoughCircles(subimage_gray, cv.HOUGH_GRADIENT_ALT, 1.2, minDist=20, param1=300, param2=0.8, minRadius=int((self.circle_diameter_px/2.0)*0.5), maxRadius=int((self.circle_diameter_px/2.0)*3))
            
            # find matching pairs of circles
            if circles is not None:
                # convert the (x, y) coordinates and radius of the circles to integers
                circles = np.round(circles[0, :]).astype("int")
                if self.DEBUG: print(f"Detected {len(circles)} circles.")
                if self.DEBUG and len(circles) < 2:
                    print(f"cv.HoughCircles(subimage_gray, cv.HOUGH_GRADIENT_ALT, 1.5, minDist=30, param1=300, param2=0.8, minRadius={int((self.circle_diameter_px/2.0)*0.5)}, maxRadius={int((self.circle_diameter_px/2.0)*1.5)})")
                

                #black_dots, white_dots, subimage_color = self.classifyDotsFromCircles(circles, subimage_color)
                #if self.DEBUG: print(f"Detected {len(black_dots)} black, and {len(white_dots)} white circles.")

                #find a darker ("black") circle which is close to a white one
                #decide on the pair that is closest to the known position of the vehicle and update its position accordingly.
                nearest_found_pair = None
                nearest_found_distance = None
                threshold = 0.4                 #TODO: threshold could be much tighter, if first height of vehice is corrected for!
                for dot1 in circles:
                    for dot2 in circles:
                        if dot1[0] == dot2[0] and dot1[1] == dot2[1] :
                            continue
                        minDist = self.size_between_black_and_white_center_px * (1-threshold)
                        maxDist = self.size_between_black_and_white_center_px * (1+threshold)

                        if minDist < distance(dot1, dot2) < maxDist:
                            #black and white match: we found a vehicle.
                            color1 = subimage_gray[dot1[1],dot1[0]]
                            color2 = subimage_gray[dot2[1],dot2[0]]

                            if abs(int(color1) -int(color2)) < 30:
                                #two circles of same color - did NOT find a vehicle...
                                continue

                            black = dot1.copy()
                            white = dot2.copy()
                            if color1 > color2:
                                black = dot2.copy()
                                white = dot1.copy()

                            #correct for height
                            black[0], black[1] = self.correctForHeightOfVehicle(black[0]+x_offset, black[1]+y_offset, self.height_over_ground_black_meters)
                            white[0], white[1] = self.correctForHeightOfVehicle(white[0]+x_offset, white[1]+y_offset, self.height_over_ground_white_meters)


                            d = distance(black, predicted_vehicle_position)
                            if nearest_found_pair is None or d < nearest_found_distance:
                                nearest_found_pair = [black, white]
                                nearest_found_distance = d

                        else:
                            if self.DEBUG: print(f"too far apart: {distance(dot1, dot2)} px does not meet threshold {self.size_between_black_and_white_center_px}")


                            

                            
                if not updatedVehicle and nearest_found_pair is not None:
                    black, white = nearest_found_pair
                    updatedVehicle = self.updateVehiclePosition(black[0], black[1], getyaw(black, white), vehicle=vehicle, detect_new_vehicles=False)
                
            
            if not updatedVehicle:
                print(f"did not find vehicle at {vehicle.getPosition()}, yaw: {vehicle.getOrientation()}")
                #we lost a vehicle!
                vehicle.ttl -= 1
                if vehicle.ttl < 0:
                    self.tracked_vehicles.remove(vehicle)
                    continue
            else:
                vehicle.ttl = 15


            """if updatedVehicle:
                print(f"updated vehicle at {vehicle.getPosition()}, yaw: {vehicle.getOrientation()}")"""


            #show portion of image which we used
            #frame[y_offset:y_offset+subimage_color.shape[0], x_offset:x_offset+subimage_color.shape[1]] = subimage_color

        time_for_one_pass = time.time() - self.time_end
        self.time_end = time.time()
        #debug info


        """for vehicle in self.tracked_vehicles:
            boundingbox = vehicle.getBoundingBox(self.current_time)
            color = (0, 255, 0)
            if vehicle.ttl < 15:
                color = (0, 0, 255)
            self.drawBoundingBox(frame, boundingbox, color=color)
            cv.putText(frame, f"Speed: {vehicle.getSpeed():.2f} m/s", (int(boundingbox[0][0]), int(boundingbox[0][1])), cv.FONT_HERSHEY_SIMPLEX, 1,
                        (0,255,0), 2, cv.LINE_AA)"""
        #print FPS
        if time_for_one_pass > 0.0:
            fps = 1.0/time_for_one_pass
            self.fps_buffer.append(fps)
            self.fps_buffer = self.fps_buffer[-self.frames_per_seconds:len(self.fps_buffer)] #keep last second

        if len(self.fps_buffer) > 0:
            cv.putText(frame, f"Localization frequency: {sum(self.fps_buffer)/len(self.fps_buffer):.1f} Hz", (50, 50), cv.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0,255,0), 1, cv.LINE_AA)

        #print(f"Procesing time: {(time_end-time_start)*1000}ms")
        # Display the resulting frame
        """cv.imshow('frame', frame)
        if cv.waitKey(1) == ord('q'):
            return"""

        
    def checkFinishLine(self, top_left=(1000, 900), bottom_right=(1050, 1100)):
        #debug finishline:
        cv.line(self.current_frame, top_left, (top_left[0], bottom_right[1]), (0,255,255), 3)

        
        for v in self.tracked_vehicles:
            currently_on_finishline = False
            if top_left[0] < v.x < bottom_right[0] and \
               top_left[1] < v.y < bottom_right[1]:
                currently_on_finishline = True

            if not v.is_on_finish_line and currently_on_finishline:
                #just passed the line:
                v.is_on_finish_line = True
                if v.time_start_of_lap > 0:
                    v.last_laptime = time.time() - v.time_start_of_lap
                v.time_start_of_lap = time.time()

            if v.is_on_finish_line and not currently_on_finishline:
                v.is_on_finish_line = False
        

if __name__ == "__main__":

    cam = Camera()

    #calibrate Camera
    cam.cameraCalibrationWizard()

    while True:
        if cam.detectVehicles() > 0:
            while len(cam.tracked_vehicles) > 0:
                cam.trackVehicles()
            while cv.waitKey(1) != ord('n'):
                pass
        else:
            print("Lost track - restarting.")
