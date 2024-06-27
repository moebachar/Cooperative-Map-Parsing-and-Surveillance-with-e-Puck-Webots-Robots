import cv2 as cv
from ultralytics import YOLO
import os
from time import time

# list of 4 colors
COLORS = [(0, 255, 0), (0, 0, 255), (255, 0, 0), (255, 255, 0)]
LABELS = []
with open("labels.txt") as f:
    for line in f.readlines():
        LABELS.append(line.strip())


def draw_rectangle(images_path, id, centerx, centery, sizex, sizey):
    img = cv.imread(images_path)
    cv.rectangle(
        img,
        (centerx - sizex // 2, centery - sizey // 2),
        (centerx + sizex // 2, centery + sizey // 2),
        COLORS[id],
        2,
    )
    cv.putText(
        img,
        LABELS[id],
        (centerx - sizex // 3, centery - sizey // 3),
        cv.FONT_HERSHEY_SIMPLEX,
        0.6,
        COLORS[id],
        2,
    )
    cv.imwrite(images_path, img)


def create_annotation(image_width, image_height, objects, iter):

    open(f"images/{iter}.txt", "w").close()
    annotation = ""
    for obj in objects:
        id = int(obj.getColors()[0] * 10)
        if 0 <= id <= 3:
            position = obj.getPositionOnImage()
            size = obj.getSizeOnImage()
            x = position[0] / image_width
            y = position[1] / image_height
            w = size[0] / image_width
            h = size[1] / image_height
            annotation += f"{id} {x} {y} {w} {h}\n"
            with open(f"images/{iter}.txt", "a") as f:
                f.write(annotation)


def draw_dataset(image_dir="images", output_dir="output"):
    # if output directory does not exist, create it, if so remove all files
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    else:
        for file in os.listdir(output_dir):
            os.remove(f"{output_dir}/{file}")
    i = 0
    for file in os.listdir(image_dir):
        if file.endswith(".txt"):
            continue
        img = cv.imread(f"{image_dir}/{file}")

        try:
            with open(f"{image_dir}/{file[:-4]}.txt") as f:
                for line in f.readlines():
                    id, x, y, w, h = map(float, line.split())
                    i = str(int(id)) + " " + str(file[:-4])
                    x, y, w, h = map(
                        int,
                        [
                            x * img.shape[1],
                            y * img.shape[0],
                            w * img.shape[1],
                            h * img.shape[0],
                        ],
                    )
                    cv.rectangle(
                        img,
                        (x - w // 2, y - h // 2),
                        (x + w // 2, y + h // 2),
                        COLORS[int(id)],
                        2,
                    )
                    cv.putText(
                        img,
                        LABELS[int(id)],
                        (x - w // 3, y - h // 3),
                        cv.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        COLORS[int(id)],
                        2,
                    )
        except Exception as e:
            print(e)
        cv.imwrite(f"{output_dir}/{file}", img)


def inference_image(image_path="test.png", output_path=None, show=False):
    model = YOLO("model/runs/detect/train/weights/best.pt")

    img = cv.imread(image_path)

    results = model(img)

    for result in results:
        if show:
            result.show()
        if output_path:
            result.save(output_path)


def direcotry_to_video(image_dir="shift"):
    if not os.path.exists(image_dir + "/output"):
        os.makedirs(image_dir + "/output")
    else:
        for file in os.listdir(image_dir + "/output"):
            os.remove(f"{image_dir}/output/{file}")
    # remove output.mp4 if exists
    if os.path.exists(image_dir + "/output.mp4"):
        os.remove(image_dir + "/output.mp4")
    # iterate through images paths
    images = [f for f in os.listdir(image_dir) if f.endswith(".png")]
    images.sort(key=lambda x: int(x.split(".")[0]))
    for i, image in enumerate(images):
        inference_image(
            f"{image_dir}/{image}", image_dir + "/output/" + str(i) + ".png"
        )

    # make mp4 video from images in output directory
    os.system(
        f"ffmpeg -r 10 -i {image_dir}/output/%d.png -vcodec mpeg4 -y {image_dir}/output.mp4"
    )

    # delete output directory
    for file in os.listdir(image_dir + "/output"):
        os.remove(f"{image_dir}/output/{file}")
    os.rmdir(image_dir + "/output")

    print("Video created")
