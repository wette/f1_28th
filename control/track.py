import cv2 as cv
from shapely.geometry import LinearRing, Polygon, Point
import pickle

class Track:
    def __init__(self):

        #coordinates of polygon represending the left, resp. right track borders.
        self.outerBounds = []
        self.innerBounds = []

        #polygon objects from shapely.
        self.outerRing = None
        self.innerRing = None
        self.polygon = None

        self.img = None #used to store image for picking borders manually

        self.mode = None

    def pointInsideTrack(self, point : Point):
        return self.polygon.contains(point)


    def saveToFile(self, filename):
        with open(filename, 'wb') as file:
            track = dict()
            track['outerBounds'] = self.outerBounds
            track['innerBounds'] = self.innerBounds
            pickle.dump(track, file)

    def loadFromFile(self, filename):
        with open(filename, 'rb') as file:
            track = pickle.load(file)
            self.outerBounds = track['outerBounds']
            self.innerBounds = track['innerBounds']

            self.convertToPolygon()

    def convertToPolygon(self):
        try:
            self.innerRing = LinearRing(self.innerBounds)
            self.outerRing = LinearRing(self.outerBounds)

            
            #test: make inner ring a litte larger:
            """newInnerRing = self.innerRing.buffer(20, single_sided=False).exterior
            newOuterRing = self.outerRing.buffer(10, single_sided=True).exterior
            
            self.innerRing = newInnerRing
            self.outerRing = newOuterRing
            """

            """import matplotlib.pyplot as plt
            from shapely.plotting import plot_polygon, plot_line
            fig = plt.figure(1, figsize=(10, 10), dpi=90)
            ax = fig.add_subplot(111)
            plot_line(self.innerRing, ax=ax, add_points=False, color="black", linewidth=1)
            plot_line(self.outerRing, ax=ax, add_points=False, color="black", linewidth=1)
            plot_line(newInnerRing, ax=ax, add_points=False, color="red", linewidth=1)
            plot_line(newOuterRing, ax=ax, add_points=False, color="blue", linewidth=1)
            plt.show()"""


        except Exception as e:
            print(f"Failure generating linear ring: {e}")

        try:
            self.polygon = Polygon(shell=self.outerRing, holes=[self.innerRing])
        
        except Exception as e:
            print(f"Failure generating polygon: {e}")


    def mouseCallback(self, event, x, y, flags, param):
        """
        Store and show clicked position of track bounds

        Parameter:
        --------------
        event:  int
            type of mouse event
        x, y:   int
            mouse position in window
        flags:  int
            Bitmask for further information of event
        param:

        Returns:
        --------------
        self.outerBound:  list of tuple
            location of clicked outerBound coordinates
        self.innerBound:  list of tuple
            location of clicked innerBound coordinates
        """

        if event == cv.EVENT_LBUTTONDOWN:
            if self.mode == "outer":
                self.outerBounds.append((x, y))
            else:
                self.innerBounds.append((x, y))

            color = (0, 0, 255) if self.mode == "outer" else (0, 255, 0)
            cv.circle(self.img, (x, y), 5, color, -1)

    def manuallyPickTrackBorders(self, img):

        print("Mouse-leftclick to set controlpoints for the track boundaries.\n",
              "First, select the OUTER outline of the track. Then switch to the inner boundary.\n"
            "press n to change mode to inner bounding\n",
            "press b to return to mode outer bounding\n",
            "press q to finish the process.\n",)

        windowName = "Pick Track Borders"
        cv.namedWindow(windowName, cv.WINDOW_NORMAL)
        cv.setMouseCallback(windowName, self.mouseCallback)

        self.mode = "outer"

        self.img = img

        while True:
            cv.imshow(windowName, img)
            key = cv.waitKey(1) & 0xFF

            # key to change mode for inner bound
            if key == ord("n") and self.mode == "outer":
                self.mode = "inner"
                print("Switched to inner outline.")

            # key to terminate localisation
            elif key == ord("q"):
                break

            # key to change back to outer mode
            elif key == ord("b") and self.mode != "outer":
                self.mode = "outer"

        cv.destroyWindow(windowName)

        #convert points to polygon:
        self.convertToPolygon()