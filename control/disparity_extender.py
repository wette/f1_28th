from sensor import Sensor
import numpy as np
import shapely
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time


class DisparityExtender:
    def __init__(self, car_width, disparity_threshold, tolerance):
        self.car_width = car_width
        self.disparity_threshold = disparity_threshold
        self.tolerance = tolerance

    def find_disparities(self, distances):
        """
        Find disparities between distances from lidar simulation

        Parameters
        -----------
        distances: list of float
            Distances as sent by the sensor. Their indices should be counted in the mathematical positive rotational sense.
            (Furthest right ray would be index 0)

        Returns
        -------
        disparities_indexes: list of int
            Indexes pointing out where in the distances list there are disparities.
            The indexes point to the shorter end of the disparity.
        directions: list of bool
            Each disparity has a direction in which the larger distance falls.
            True if distance left of the disparity point is larger and False otherwise.
        """
        disparities_indexes = []
        directions = []
        for i in range(len(distances) - 1):
            if abs(distances[i] - distances[i + 1]) >= self.disparity_threshold:
                if distances[i] < distances[i + 1]:
                    disparities_indexes.append(i)
                    directions.append(True)
                else:
                    disparities_indexes.append(i + 1)
                    directions.append(False)
        return disparities_indexes, directions

    def extend_disparities(self, distances, rays, disparities_indexes, directions):
        overwritten_indexes = []

        # Check only the rays going into the "open" direction
        for i, disparity_index in enumerate(disparities_indexes):
            if directions[i]:
                indices = range(disparity_index + 1, len(rays))
            else:
                indices = range(disparity_index - 1, -1, -1)

            # Vector from origin point to short side of disparity
            disp_vec = np.array(
                [
                    rays[disparity_index].xy[0][1] - rays[disparity_index].xy[0][0],
                    rays[disparity_index].xy[1][1] - rays[disparity_index].xy[1][0],
                ]
            )
            for j in indices:
                # Vector from origin point to intersection next to short side of disparity
                ray_vec = np.array(
                    [
                        rays[j].xy[0][1] - rays[j].xy[0][0],
                        rays[j].xy[1][1] - rays[j].xy[1][0],
                    ]
                )
                # Calculate angle between the rays
                angle = np.arccos(
                    np.dot(disp_vec, ray_vec)
                    / (rays[disparity_index].length * rays[j].length)
                )

                # Calculate distance to disparity point
                d = np.sin(angle) * distances[disparity_index]
                epsilon = 0.0001
                if d < (self.car_width / 2 + self.tolerance):
                    # Dont overwrite closer distances with further ones
                    if distances[j] <= distances[disparity_index]:
                        continue
                    distances[j] = distances[disparity_index] + j * epsilon
                    overwritten_indexes.append(j)
                else:
                    break
        max_distance_index = np.argmax(distances)
        t = distances[max_distance_index] / rays[max_distance_index].length
        setpoint = [
            (1 - t) * rays[max_distance_index].xy[0][0]
            + t * rays[max_distance_index].xy[0][1],
            (1 - t) * rays[max_distance_index].xy[1][0]
            + t * rays[max_distance_index].xy[1][1],
        ]
        angle = np.atan2(
            setpoint[1] - rays[max_distance_index].xy[1][0],
            setpoint[0] - rays[max_distance_index].xy[0][0],
        )
        return distances, setpoint, angle


def update(frame):
    global car_pos, setpoint, direction_arrow, angle

    # Jump to the current setpoint
    car_pos[:] = setpoint
    t0 = time.time()
    # Update sensor origin and angle
    sensor.update(origin=car_pos.tolist(), angle=np.rad2deg(angle))

    # Get distances from new position
    distances = sensor.get_distances(bounds)

    # Disparity extension: find new target and angle
    disparities_indexes, directions = disparityExtender.find_disparities(distances)
    modified_distances, setpoint, angle = disparityExtender.extend_disparities(
        distances, sensor.rays, disparities_indexes, directions
    )
    t1 = time.time()
    print(t1 - t0)

    setpoint = np.array(setpoint)

    # Update plot elements
    car_dot.set_data([car_pos[0]], [car_pos[1]])
    setpoint_dot.set_data([setpoint[0]], [setpoint[1]])

    # Redraw rays
    all_ray_x = []
    all_ray_y = []
    for ray in sensor.rays:
        all_ray_x += ray.xy[0]
        all_ray_y += ray.xy[1]
    rays_lines.set_data(all_ray_x, all_ray_y)

    # Update direction arrow
    direction_arrow.remove()
    direction_arrow = ax.arrow(
        car_pos[0],
        car_pos[1],
        np.cos(angle) * 10,
        np.sin(angle) * 10,
        color="green",
        head_width=5,
        length_includes_head=True,
    )

    # Update car front line
    left, right = get_car_front_line(car_pos, angle, disparityExtender.car_width)
    car_front_line.set_data([left[0], right[0]], [left[1], right[1]])

    return car_dot, setpoint_dot, rays_lines, direction_arrow


def get_car_front_line(car_pos, angle, car_width, front_offset=20):
    # Move from car_pos to the front center
    front_center = np.array(
        [
            car_pos[0] + np.cos(angle) * front_offset,
            car_pos[1] + np.sin(angle) * front_offset,
        ]
    )
    # Orthogonal direction
    ortho_angle = angle + np.pi / 2
    half_width = car_width / 2
    left = (
        front_center + np.array([np.cos(ortho_angle), np.sin(ortho_angle)]) * half_width
    )
    right = (
        front_center - np.array([np.cos(ortho_angle), np.sin(ortho_angle)]) * half_width
    )
    return left, right


if __name__ == "__main__":
    sensor = Sensor(
        num_rays=10, ray_length=500, pixel_distance=1, fov=90, origin=[500, 280]
    )
    disparityExtender = DisparityExtender(
        car_width=50, disparity_threshold=50, tolerance=20
    )
    # bounds = sensor.build_dummy_path([1000, 1000], 100, 20, 20)
    # obj1 = sensor.rectangular_obstacle([1000, 950], 10, 20)
    # obj2 = sensor.rectangular_obstacle([1000, 1030], 10, 20)
    # obj3 = sensor.rectangular_obstacle([1000, 1080], 10, 20)
    # obj4 = sensor.rectangular_obstacle([1030, 900], 10, 20)
    # bounds.append(obj1)
    # bounds.append(obj2)
    # bounds.append(obj3)
    # bounds.append(obj4)
    bounds = []
    inner = np.load("Path/inner_bounds.npy", allow_pickle=True).item()
    outer = np.load("Path/outer_bounds.npy", allow_pickle=True).item()

    bounds.append(outer)
    bounds.append(inner)

    t0 = time.time()
    sensor.set_rays(0)
    distances = sensor.get_distances(bounds)
    disparities_indexes, directions = disparityExtender.find_disparities(distances)
    modified_distances, setpoint, angle = disparityExtender.extend_disparities(
        distances, sensor.rays, disparities_indexes, directions
    )
    t1 = time.time()

    print(t1 - t0)

    # Animate virtual race track with car / sensor
    fig, ax = plt.subplots()
    ax.set_aspect("equal")
    for bound in bounds:
        shapely.plotting.plot_line(bound, ax=ax, color="blue")

    # Track car's position
    car_pos = np.array([sensor.origin.x, sensor.origin.y], dtype=float)
    setpoint = np.array(setpoint, dtype=float)

    # Plot elements (updated in animation)
    (rays_lines,) = ax.plot([], [], "ko")  # Rays as points
    (car_dot,) = ax.plot([], [], "go", label="Car")  # Car position
    (setpoint_dot,) = ax.plot([], [], "ro", label="Setpoint")
    direction_arrow = ax.arrow(0, 0, 0, 0, color="green", head_width=5)
    (car_front_line,) = ax.plot(
        [], [], "r-", linewidth=3, label="Car Front"
    )  # Car front line

    ani = FuncAnimation(fig, update, frames=500, interval=300, blit=False)
    plt.show()
