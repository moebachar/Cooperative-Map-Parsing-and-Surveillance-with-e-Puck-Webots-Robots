from soldier import Soldier
from time import time
from utilities import (
    record,
    plot_map_and_charges,
    start_experiance,
    save_experiance,
    collect_data,
)

TIME_STEP = 64
CAT_ID = 2
COLLECT_DATA = False


def main():
    robot = Soldier()

    start_experiance()

    i = 0

    robot_name = robot.robot_node.getField("name").getSFString()

    start = time()
    while robot.step(TIME_STEP) != -1:

        if robot_name == "e-puck(2)":
            robot.camera.saveImage("shift/" + str(i) + ".png", quality=100)

        if robot.camera.getRecognitionNumberOfObjects() > 0:
            objects = robot.camera.getRecognitionObjects()
            for obj in objects:
                id = int(obj.getColors()[0] * 10)
                if id == CAT_ID and obj.getSizeOnImage()[0] > 100:
                    robot.motor_stop()
                    print("cat detected")

                    robot.speaker.playSound(
                        robot.speaker, robot.getSelf(), "alarm.wav", 1.0, 1.0, 0, False
                    )
                    while robot.camera.getRecognitionNumberOfObjects() > 0:
                        robot.step(TIME_STEP)

        if COLLECT_DATA and robot_name == "e-puck":
            collect_data(robot, i)

        robot.send_map_update()
        robot.receive_and_update_map()
        i += 1
        robot.path_planing()
        robot.path_execution()

        if robot_name == "e-puck":
            record(i, robot.map.n, time() - start)
            if i % 500 == 0:
                plot_map_and_charges(robot, i)
            if robot.map.n == 255:
                save_experiance()
                break


if __name__ == "__main__":
    main()
