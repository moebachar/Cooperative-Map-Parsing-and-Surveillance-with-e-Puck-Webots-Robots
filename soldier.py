from controller import Supervisor
from utilities import SENSORS
from math import pi as PI
from map import Map
import numpy as np

TIME_STEP = 64
RANGES = (0, 510)
DESC_COEFF = 100
R = 0.0355
ARENA_DEF = "ARENA"
C = 127
Qr = -10_000
SENSOR_THRESHOLD = 140
ROBOT_ANGULAR_SPEED_IN_DEGREES = 283.588111888


class Soldier(Supervisor):
    def __init__(self):
        super(Soldier, self).__init__()

        self.camera = self.getDevice("camera")
        self.camera.enable(TIME_STEP)
        self.camera.recognitionEnable(TIME_STEP)

        self.speaker = self.getDevice("speaker")
        self.speaker.setEngine("pico")
        self.speaker.setLanguage("en-UK")

        self.sensors = self.initialize_sensors()

        self.DESC_COEFF = DESC_COEFF
        self.R = R
        self.Qr = Qr
        width, height = tuple(
            self.getFromDef(ARENA_DEF).getField("floorSize").getSFVec2f()
        )

        self.map = Map(
            width=int(width),
            height=int(height),
            ranges=RANGES,
            DESC_COEFF=self.DESC_COEFF,
            R=self.R,
            C=C,
        )

        self.robot_node = self.getSelf()
        self.emitter = self.getDevice("emitter")
        self.receiver = self.getDevice("receiver")
        self.receiver.enable(TIME_STEP)
        self.team_positons = {}

        self.target_angle = 0
        self.direction = "forward"
        self.maxMotorVelocity = 6.28
        self.initialVelocity = 1 * self.maxMotorVelocity
        self.SENSOR_THRESHOLD = SENSOR_THRESHOLD
        self.ROBOT_ANGULAR_SPEED_IN_DEGREES = ROBOT_ANGULAR_SPEED_IN_DEGREES

        # Get left and right wheel motors.
        self.leftMotor = self.getDevice("left wheel motor")
        self.rightMotor = self.getDevice("right wheel motor")

        # Disable motor PID control mode.
        self.leftMotor.setPosition(float("inf"))
        self.rightMotor.setPosition(float("inf"))
        # Set the speed of the motors to go forward
        self.leftMotor.setVelocity(self.initialVelocity)
        self.rightMotor.setVelocity(self.initialVelocity)

    def initialize_sensors(self):
        self.sensors = []
        for sensor in SENSORS:
            sensor = self.getDevice(sensor.name)
            sensor.enable(TIME_STEP)
            self.sensors.append(sensor)
        return self.sensors

    def convert_distance_to_meters(self, sensor_value):
        # Define the lookup table as a list of tuples (sensor_value, distance)
        lookup_table = [
            (4095.0, 0.002),
            (2133.33, 0.005),
            (1465.73, 0.01),
            (601.46, 0.015),
            (383.84, 0.02),
            (234.93, 0.03),
            (158.03, 0.04),
            (120.0, 0.05),
            (104.09, 0.06),
            (67.19, 0.07),
        ]

        # Sort the lookup table by sensor values (if not already sorted)
        lookup_table.sort()

        # Check boundaries for sensor values less than the smallest or greater than the largest in the table
        if sensor_value <= lookup_table[0][0]:
            return lookup_table[0][1]
        elif sensor_value >= lookup_table[-1][0]:
            return lookup_table[-1][1]

        # Find the right place in the table by finding the first value larger than the sensor value
        for i in range(1, len(lookup_table)):
            if sensor_value < lookup_table[i][0]:
                # Perform linear interpolation
                x0, y0 = lookup_table[i - 1]
                x1, y1 = lookup_table[i]
                return y0 + (sensor_value - x0) * (y1 - y0) / (x1 - x0)

        # If no interpolation point is found (should not happen with correct data), return None
        return None

    def calculate_map_update(self, sensor):

        sensor_value = sensor.getValue()
        distance = self.convert_distance_to_meters(sensor_value)

        THETA = (self.robot_node.getField("rotation").getSFRotation()[3]) % (2 * PI)
        s = [se for se in SENSORS if se.name == sensor.getName()][0]

        robot_position = self.robot_node.getPosition()[:2]

        s0, s1 = self.map.calculate_distance_measurements(
            s, THETA, distance, robot_position
        )

        s0 += self.map.get_footprints_indices(robot_position)

        return s0, s1

    def send_map_update(self):
        updates = [[], []]
        for sensor in self.sensors:
            update = self.calculate_map_update(sensor)
            updates[0] += update[0]
            updates[1] += update[1]
            for u in update[0]:
                i, j = u
                self.map.values[i, j] = min(
                    self.map.values[i, j] + 1, self.map.ranges[1]
                )
            for u in update[1]:
                i, j = u
                self.map.values[i, j] = max(
                    self.map.values[i, j] - 1, self.map.ranges[0]
                )
        position = self.robot_node.getPosition()[:2]
        position.append(self.robot_node.getField("name").getSFString())
        self.emitter.send(str(position).encode("utf-8"))
        self.emitter.send(str(updates).encode("utf-8"))

    def receive_and_update_map(self):
        while self.receiver.getQueueLength() > 0:
            update = eval(self.receiver.getString())
            self.receiver.nextPacket()
            if len(update) == 3:
                self.team_positons[update[2]] = np.array(update[:2])
            else:
                for u in update[0]:
                    i, j = u
                    self.map.values[i, j] = min(
                        self.map.values[i, j] + 1, self.map.ranges[1]
                    )
                for u in update[1]:
                    i, j = u
                    self.map.values[i, j] = max(
                        self.map.values[i, j] - 10, self.map.ranges[0]
                    )

    def get_sensors_conditions(self):
        # get boolean list of sensors if they pass SENSOR_THRESHOLD
        conditions = []
        for sensor in self.sensors:
            conditions.append(sensor.getValue() > self.SENSOR_THRESHOLD)
        return conditions

    def path_planing(self):
        self.map.ramp()

        q = self.map.charges

        position = np.array(self.robot_node.getPosition()[:2])

        r = position - self.map.cell_centers  # self.map.expanded_centers
        q = q[:, :, np.newaxis]

        r_norm = np.linalg.norm(r, axis=2) + 1e-10
        r_norm = r_norm[:, :, np.newaxis]

        L1 = np.sum((q * r) / (r_norm**2), axis=0)
        L1 = np.sum(L1, axis=0)

        L2 = np.zeros_like(position)
        for _, value in self.team_positons.items():
            r = position - value
            r_norm = r[0] ** 2 + r[1] ** 2 + 1e-10
            L2 += (self.Qr * r) / (r_norm)

        L = L1 + L2
        phi = np.arctan2(L[1], L[0]) + PI
        self.target_angle = phi  # if phi > 0 else PI + phi

    def turn_left(self):
        self.getDevice("right wheel motor").setVelocity(self.initialVelocity)
        self.getDevice("left wheel motor").setVelocity(self.initialVelocity * 0.5)

    def turn_right(self):
        self.getDevice("left wheel motor").setVelocity(self.initialVelocity)
        self.getDevice("right wheel motor").setVelocity(self.initialVelocity * 0.5)

    def moove_forward(self):
        self.getDevice("right wheel motor").setVelocity(self.initialVelocity)
        self.getDevice("left wheel motor").setVelocity(self.initialVelocity)

    def motor_stop(self):
        self.getDevice("right wheel motor").setVelocity(0)
        self.getDevice("left wheel motor").setVelocity(0)

    def motor_move_forward(self):
        self.getDevice("right wheel motor").setVelocity(self.maxMotorVelocity)
        self.getDevice("left wheel motor").setVelocity(self.maxMotorVelocity)

    def motor_rotate_right(self):
        self.getDevice("right wheel motor").setVelocity(-self.maxMotorVelocity)
        self.getDevice("left wheel motor").setVelocity(self.maxMotorVelocity)

    def motor_rotate_left(self):
        self.getDevice("right wheel motor").setVelocity(self.maxMotorVelocity)
        self.getDevice("left wheel motor").setVelocity(-self.maxMotorVelocity)

    def calculate_rotation_time(self, degrees):
        return abs(degrees) / self.ROBOT_ANGULAR_SPEED_IN_DEGREES

    def motor_rotate_left_in_degrees(self, degrees):
        self.motor_rotate_left()
        duration = self.calculate_rotation_time(degrees)
        start_time = self.getTime()
        self.step(TIME_STEP)
        while self.getTime() - start_time < duration:
            self.step(TIME_STEP)
        self.motor_move_forward()
        while self.getTime() - start_time < duration * 2:
            self.step(TIME_STEP)
        self.motor_stop()

    def motor_rotate_right_in_degrees(self, degrees):
        self.motor_rotate_right()
        duration = self.calculate_rotation_time(degrees)
        start_time = self.getTime()
        self.step(TIME_STEP)
        while self.getTime() - start_time < duration:
            self.step(TIME_STEP)
        self.motor_move_forward()
        while self.getTime() - start_time < duration * 2:
            self.step(TIME_STEP)
        self.motor_stop()

    def path_execution(self):

        sensors_conditions = self.get_sensors_conditions()
        if sensors_conditions[1] and sensors_conditions[6]:
            self.motor_rotate_left_in_degrees(180)
        elif sensors_conditions[0] or sensors_conditions[1]:
            self.motor_rotate_left_in_degrees(90)
        elif sensors_conditions[7] or sensors_conditions[6]:
            self.motor_rotate_right_in_degrees(90)
        else:
            THETA = -self.robot_node.getField("rotation").getSFRotation()[3]
            delta_phi = (self.target_angle - THETA) % (2 * PI)
            if abs(delta_phi) < 0.15:
                self.direction = "forward"
                self.moove_forward()

            elif 0.1 <= delta_phi < PI or -2 * PI < delta_phi < -PI:
                self.direction = "left"
                self.turn_left()

            else:
                self.direction = "right"
                self.turn_right()

    def get_camera_image(self):

        image = self.camera.getImageArray()
        return image

    def plot_map(self):
        self.map.plot_map(self.robot_node.getPosition()[:2])

    def plot_robot(self, ax):
        robot_position = self.robot_node.getPosition()[:2]
        # plot  Robot
        theta = np.linspace(0, 2 * np.pi, 100)
        x = robot_position[0] + self.R * np.cos(theta)
        y = robot_position[1] + self.R * np.sin(theta)
        ax.plot(x, y, "r")

        # plot sensorsx
        for sensor in SENSORS:
            d = 0.07
            THETA = (self.robot_node.getField("rotation").getSFRotation()[3]) % (2 * PI)
            x, y, theta = sensor.x, sensor.y, sensor.orientation
            P_oxy = np.array([np.sin(theta), -np.cos(theta)]) * d + np.array([x, y])
            transformation_matrix = np.array(
                [[np.cos(THETA), -np.sin(THETA)], [np.sin(THETA), np.cos(THETA)]]
            )
            P_OXY = transformation_matrix.dot(P_oxy) + np.array(
                [robot_position[0], robot_position[1]]
            )

            S_OXY = transformation_matrix.dot(np.array([x, y])) + np.array(
                [robot_position[0], robot_position[1]]
            )

            # descritize the line from S_OXY to P_OXY by Desc_coeff
            n = self.DESC_COEFF
            x = np.linspace(S_OXY[0], P_OXY[0], n)
            y = np.linspace(S_OXY[1], P_OXY[1], n)
            ax.plot(x, y, label=sensor.name)

        ax.set_xticks(
            np.linspace(
                -self.map.width / 2,
                self.map.width / 2,
                self.map.width * self.DESC_COEFF + 1,
            )
        )
        ax.set_yticks(
            np.linspace(
                -self.map.height / 2,
                self.map.height / 2,
                self.map.height * self.DESC_COEFF + 1,
            )
        )
