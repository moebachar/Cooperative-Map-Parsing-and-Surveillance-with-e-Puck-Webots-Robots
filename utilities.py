import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
import pandas as pd
from collections import namedtuple
import os
import zipfile
from datetime import datetime
from vision import create_annotation

Sensor = namedtuple("Sensor", ["name", "x", "y", "z", "orientation"])

SENSORS = [
    Sensor(name="ps0", x=0.03, y=-0.01, z=0.033, orientation=1.27),
    Sensor(name="ps1", x=0.022, y=-0.025, z=0.033, orientation=0.77),
    Sensor(name="ps2", x=0.0, y=-0.031, z=0.033, orientation=0.0),
    Sensor(name="ps3", x=-0.03, y=-0.015, z=0.033, orientation=5.21),
    Sensor(name="ps4", x=-0.03, y=0.015, z=0.033, orientation=4.21),
    Sensor(name="ps5", x=0.0, y=0.031, z=0.033, orientation=3.14159),
    Sensor(name="ps6", x=0.022, y=0.025, z=0.033, orientation=2.37),
    Sensor(name="ps7", x=0.03, y=0.01, z=0.033, orientation=1.87),
]


def start_experiance():
    for file in os.listdir("results"):
        os.remove("results/" + file)

    for file in os.listdir("images"):
        os.remove("images/" + file)

    for file in os.listdir("charges"):
        os.remove("charges/" + file)

    for file in os.listdir("shift"):
        os.remove("shift/" + file)

    with open("record.csv", "w") as f:
        f.write("iteration,n,duration\n")

    if os.path.exists("experiance.png"):
        os.remove("experiance.png")


def record(iteration, n, duration, file_path="record.csv", delimiter=","):
    # write one line
    with open(file_path, "a") as f:
        f.write(f"{iteration}{delimiter}{n}{delimiter}{duration}\n")


def plot_experiance(file_path="record.csv", delimiter=","):
    plt.figure()
    df = pd.read_csv(file_path, delimiter=delimiter)
    plt.plot(df["duration"], df["n"])
    plt.xlabel("duration")
    plt.ylabel("n")
    plt.savefig("experiance.png", dpi=300)
    plt.close()


def save_experiance():

    plot_experiance()

    # Get the current working directory (project directory)
    base_dir = os.getcwd()

    # Define directories and file paths relative to the current working directory
    results_dir = os.path.join(base_dir, "results")
    charges_dir = os.path.join(base_dir, "charges")
    experience_png = os.path.join(base_dir, "experiance.png")
    experiences_dir = os.path.join(base_dir, "experiances")
    record_csv = os.path.join(base_dir, "record.csv")

    # Create a unique name for the ZIP file based on the current date and time
    zip_filename = datetime.now().strftime("archive_%Y%m%d_%H%M%S.zip")
    zip_path = os.path.join(experiences_dir, zip_filename)

    with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as zipf:
        # Add the results directory
        for filename in os.listdir(results_dir):
            file_path = os.path.join(results_dir, filename)
            if os.path.isfile(file_path):
                arcname = os.path.join("results", filename)
                zipf.write(file_path, arcname)

        # Add the charges directory
        for filename in os.listdir(charges_dir):
            file_path = os.path.join(charges_dir, filename)
            if os.path.isfile(file_path) and filename != "experiances":
                arcname = os.path.join("charges", filename)
                zipf.write(file_path, arcname)

        # Add the experiace.png file
        if os.path.isfile(experience_png):
            arcname = os.path.basename(experience_png)
            zipf.write(experience_png, arcname)

        # Add the record.csv file
        if os.path.isfile(record_csv):
            arcname = os.path.basename(record_csv)
            zipf.write(record_csv, arcname)


def plot_map_and_charges(robot, i):
    fig, ax = plt.subplots()
    ax.plot(
        robot.robot_node.getPosition()[0],
        robot.robot_node.getPosition()[1],
        "go",
    )

    for _, value in robot.team_positons.items():
        ax.plot(value[0], value[1], "ro")
    ax.plot(
        [
            robot.robot_node.getPosition()[0],
            robot.robot_node.getPosition()[0] + np.cos(robot.target_angle),
        ],
        [
            robot.robot_node.getPosition()[1],
            robot.robot_node.getPosition()[1] + np.sin(robot.target_angle),
        ],
        "r",
    )
    robot.map.plot_map(ax, fig)
    plt.title("Iteration : " + str(i) + " | n : " + str(robot.map.n))
    plt.savefig("results/iteration_" + str(i) + ".png", dpi=300)

    fig, ax = plt.subplots()
    ax.plot(
        robot.robot_node.getPosition()[0],
        robot.robot_node.getPosition()[1],
        "go",
    )

    for _, value in robot.team_positons.items():
        ax.plot(value[0], value[1], "ro")
    ax.plot(
        [
            robot.robot_node.getPosition()[0],
            robot.robot_node.getPosition()[0] + np.cos(robot.target_angle),
        ],
        [
            robot.robot_node.getPosition()[1],
            robot.robot_node.getPosition()[1] + np.sin(robot.target_angle),
        ],
        "r",
    )
    robot.map.plot_charges(ax, fig)
    plt.title("Iteration : " + str(i) + " | n : " + str(robot.map.n))
    # save plot
    plt.savefig("charges/iteration_" + str(i) + ".png", dpi=300)
    plt.close()


def collect_data(robot, i):
    image_width = robot.camera.getWidth()
    image_height = robot.camera.getHeight()
    robot_name = robot.robot_node.getField("name").getSFString()
    if (
        i % 10 == 0
        and robot_name == "e-puck"
        and robot.camera.getRecognitionNumberOfObjects() > 0
    ):
        robot.camera.saveImage("images/" + str(i) + ".png", quality=100)
        create_annotation(
            image_width, image_height, robot.camera.getRecognitionObjects(), i
        )

    if (
        i % 11 == 0
        and robot_name == "e-puck(1)"
        and robot.camera.getRecognitionNumberOfObjects() > 0
    ):
        robot.camera.saveImage("images/" + str(i) + ".png", quality=100)
        create_annotation(
            image_width, image_height, robot.camera.getRecognitionObjects(), i
        )

    if (
        i % 12 == 0
        and robot_name == "e-puck(2)"
        and robot.camera.getRecognitionNumberOfObjects() > 0
    ):
        robot.camera.saveImage("images/" + str(i) + ".png", quality=100)
        create_annotation(
            image_width, image_height, robot.camera.getRecognitionObjects(), i
        )
