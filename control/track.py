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

        self.mode = None

    def pointInsideTrack(self, point : Point):
        return self.outerRing.within(point) and not self.innerRing.within(point)
    
    def pointInsideTrack(self, x : int, y : int):
        return self.pointInsideTrack(Point(x, y))

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

        except Exception as e:
            print(f"Failure generating linear ring: {e}")

        try:
            self.polygon = Polygon(shell=self.outerRing, holes=self.innerRing)
        
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
            cv.circle(self.display, (x, y), 5, color, -1)

    def manuallyPickTrackBorders(self, img):
        windowName = "Pick Track Borders"
        cv.namedWindow(windowName, cv.WINDOW_NORMAL)
        cv.setMouseCallback(windowName, self.mouseCallback)

        self.mode = "outer"

        while True:
            cv.imshow(windowName, img)
            key = cv.waitKey(1) & 0xFF

            # key to change mode for inner bound
            if key == ord("n") and self.mode == "outer":
                self.mode = "inner"

            # key to terminate localisation
            elif key == ord("q"):
                break

            # key to change back to outer mode
            elif key == ord("b") and self.mode != "outer":
                self.mode = "outer"

        cv.destroyWindow(windowName)

        #convert points to polygon:
        self.convertToPolygon()