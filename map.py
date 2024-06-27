import numpy as np
from math import floor
from utilities import Sensor


class Map:

    def __init__(
        self, width=2, height=2, ranges=(0, 510), DESC_COEFF=100, R=0.0355, C=127
    ):
        self.width = width
        self.height = height
        self.ranges = ranges
        self.INITIAL_VALUES = (
            np.zeros((self.width * DESC_COEFF, self.height * DESC_COEFF))
            + sum(ranges) / 2
        )
        self.values = (
            np.zeros((self.width * DESC_COEFF, self.height * DESC_COEFF))
            + sum(ranges) / 2
        )

        self.values[:, :10] = 0
        self.values[:, -10:] = 0
        self.values[:10, :] = 0
        self.values[-10:, :] = 0

        self.INITIAL_VALUES[:, :10] = 0
        self.INITIAL_VALUES[:, -10:] = 0
        self.INITIAL_VALUES[:10, :] = 0
        self.INITIAL_VALUES[-10:, :] = 0

        self.charges = np.zeros((self.width * DESC_COEFF, self.height * DESC_COEFF))
        self.n = 0
        self.CLASSIFICATION_THRESHOLD = 0.06
        self.DESC_COEFF = DESC_COEFF
        self.R = R
        self.C = C

        self.cell_centers = np.zeros(
            (self.width * self.DESC_COEFF, self.height * self.DESC_COEFF, 2)
        )
        for i in range(self.width * self.DESC_COEFF):
            for j in range(self.height * self.DESC_COEFF):
                self.cell_centers[i, j] = np.array(
                    [
                        -self.width / 2 + (1 + 2 * j) * 1 / (2 * self.DESC_COEFF),
                        self.height / 2 - (1 + 2 * i) * 1 / (2 * self.DESC_COEFF),
                    ]
                )
        self.expanded_centers = np.zeros(
            (
                int(self.width * 1.5 * self.DESC_COEFF),
                int(self.height * 1.5 * self.DESC_COEFF),
                2,
            )
        )
        for i in range(int(self.width * 1.5 * self.DESC_COEFF)):
            for j in range(int(self.height * 1.5 * self.DESC_COEFF)):
                self.expanded_centers[i, j] = np.array(
                    [
                        (-self.width * 1.5) / 2
                        + (1 + 2 * j) * 1 / (2 * self.DESC_COEFF),
                        (self.height * 1.5) / 2
                        - (1 + 2 * i) * 1 / (2 * self.DESC_COEFF),
                    ]
                )

    def calculate_distance_measurements(self, sensor: Sensor, THETA, d, robot_position):
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

        x = np.linspace(S_OXY[0], P_OXY[0], self.DESC_COEFF)
        y = np.linspace(S_OXY[1], P_OXY[1], self.DESC_COEFF)

        s0 = set()
        s1 = []

        # adding cell indecies (i, j) of point [x[i], y[i]] to the set
        for i in range(self.DESC_COEFF):
            i, j = self.get_cell_indices([x[i], y[i]])
            if (
                0 < i < self.width * self.DESC_COEFF
                and 0 < j < self.height * self.DESC_COEFF
            ):
                s0.add((i, j))
        i, j = self.get_cell_indices(P_OXY)
        if (
            d < self.CLASSIFICATION_THRESHOLD
            and 0 < i < self.width * self.DESC_COEFF
            and 0 < j < self.height * self.DESC_COEFF
        ):
            s1.append((i, j))
            s0.remove((i, j))
        elif (
            d >= self.CLASSIFICATION_THRESHOLD
            and 0 < i < self.width * self.DESC_COEFF
            and 0 < j < self.height * self.DESC_COEFF
        ):
            s0.add((i, j))

        return list(s0), s1

    def get_cell_indices(self, P_OXY):
        P = np.array(P_OXY) + np.array([self.width / 2, -self.height / 2])
        P[1] *= -1
        cell_width = 1 / self.DESC_COEFF
        cell_height = 1 / self.DESC_COEFF
        x = P[0]
        y = P[1]
        j = floor(x / cell_width)
        i = floor(y / cell_height)
        if x == self.width:
            j = self.width * self.DESC_COEFF - 1
        if y == self.height:
            i = self.height * self.DESC_COEFF - 1
        return (i, j)

    def get_footprints_indices(self, robot_position):
        result = []
        top_left = np.array(robot_position) + np.array([-self.R, self.R])
        I, J = self.get_cell_indices(top_left)
        number_of_cells = int(2 * self.R * self.DESC_COEFF)
        for i in range(
            max(I - 1, 0), min(I + number_of_cells + 1, self.width * self.DESC_COEFF)
        ):
            for j in range(
                max(J - 1, 0),
                min(J + number_of_cells + 1, self.height * self.DESC_COEFF),
            ):
                if (robot_position[0] - self.cell_centers[i, j, 0]) ** 2 + (
                    robot_position[1] - self.cell_centers[i, j, 1]
                ) ** 2 <= self.R**2:
                    result.append((i, j))
        return result

    def get_number_of_visits(self):
        V = np.abs(self.values - self.INITIAL_VALUES)
        ratio = np.sum(self.charges < self.C) / (
            self.charges.shape[0] * self.charges.shape[1]
        )
        self.n += 1 if ratio > 0.95 else 0
        return V

    def dynamic_step(self):
        V = self.get_number_of_visits()
        self.charges = np.where(V <= self.n, self.C, 0)

    def fixed_step(self):
        V = self.get_number_of_visits()
        m = V.max()
        self.charges = np.where(V < m, self.C, 0)

    def ramp(self):
        V = self.get_number_of_visits()
        m = V.max()
        self.charges = np.where(V <= self.n, self.C, self.C * (m - V) / (m - self.n))

    def plot_map(self, ax, fig):
        y, x = np.meshgrid(
            np.linspace(-self.width / 2, self.width / 2, self.width * self.DESC_COEFF),
            np.linspace(
                -self.height / 2, self.height / 2, self.height * self.DESC_COEFF
            ),
        )

        c = ax.pcolormesh(
            x,
            y,
            np.rot90(self.values, 3),
            cmap="RdBu",
            vmin=0,
            vmax=510,
        )
        ax.set_title("World Map")
        # set the limits of the plot to the limits of the data
        ax.axis([x.min(), x.max(), y.min(), y.max()])
        fig.colorbar(c, ax=ax)

    def plot_charges(self, ax, fig):
        y, x = np.meshgrid(
            np.linspace(-self.width / 2, self.width / 2, self.width * self.DESC_COEFF),
            np.linspace(
                -self.height / 2, self.height / 2, self.height * self.DESC_COEFF
            ),
        )

        c = ax.pcolormesh(
            x,
            y,
            np.rot90(self.charges, 3),
            cmap="RdBu",
            vmin=0,
            vmax=self.C,
        )
        ax.set_title("World Map")
        # set the limits of the plot to the limits of the data
        ax.axis([x.min(), x.max(), y.min(), y.max()])
        fig.colorbar(c, ax=ax)
