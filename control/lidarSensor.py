from .track import Track
import numpy as np
import shapely

class LidarSensor:
    def __init__(self, track: Track, field_of_view_deg: float, numRays: int, rayLength_px: int):
        self.track = track
        self.fov_rad = np.deg2rad(field_of_view_deg)
        self.numRays = numRays
        self.rayLength = rayLength_px

    def __generateRays(self, xPos:int, yPos:int, angle_rad:float):
        rays = []
        ray_angle = angle_rad - angle_rad / 2
        for i in range(self.numRays):
            angle_rad = ray_angle
            delta_x = np.cos(angle_rad) * self.rayLength
            delta_y = -np.sin(angle_rad) * self.rayLength
            rays.append(
                shapely.LineString(
                    [
                        [xPos, yPos],
                        [xPos + delta_x, yPos + delta_y],
                    ]
                )
            )
            ray_angle += self.fov_rad / (self.numRays - 1)

        return rays
    
    def __computeIntersectionsWithTrack(self, rays):
        distances = []
        for ray in rays:
            dist = []
            for bound in [self.track.innerBounds, self.track.outerBounds]:
                intersection = bound.intersection(ray)
                if intersection.is_empty:
                    dist.append(self.rayLength)
                else:
                    dist.append(intersection)
            distances.append(min(dist))

        return distances

    def getReadings(self, xPos:int, yPos:int, angle:float):
        rays = self.__generateRays(xPos, yPos, angle)
        distances = self.__computeIntersectionsWithTrack(rays)

        return distances, rays




