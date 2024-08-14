import torch
from torch.nn.functional import one_hot
from torch.utils.data import Dataset, DataLoader
from torchvision.io import read_image

from pathlib import Path
import os
import cv2
import numpy as np

class TrafficLightDataset(Dataset):
    def __init__(self, type):
        self.abs_path = os.path.join(Path(__file__).parent, f"source/{type}/")

        self.images_path = os.path.join(self.abs_path, "images/")
        self.labels_path = os.path.join(self.abs_path, "labels/")

        self.images = os.listdir(self.images_path)
        self.labels = os.listdir(self.labels_path)

        self.data_paths_all = []

        # read all images and save them into memory
        for image in self.images:
            # get X
            img_path = os.path.join(self.images_path, image)

            # get Y
            label_path = os.path.join(self.labels_path, image.replace(".jpg", "") + ".txt")

            self.data_paths_all.append([img_path, label_path])

        print("Loading data done")

    def __len__(self):
        return len(self.data_paths_all)

    def __getitem__(self, idx):
        img = read_image(self.data_paths_all[idx][0]) / 255 # load and normalize

        label = open(self.data_paths_all[idx][1]).read()

        labels = label.split(" ")
        cls_id = torch.Tensor([int(labels[0])]).to(torch.int64)
        cnt_x = float(labels[1])
        cnt_y = float(labels[2])
        width = float(labels[3])
        height = float(labels[4].replace("\n24", ""))

        cls_id = one_hot(torch.Tensor(cls_id).to(torch.int64), num_classes=29)

        half_width = width / 2
        half_height = height / 2

        x1 = cnt_x - half_width
        x2 = cnt_x + half_width

        y1 = cnt_y + half_height
        y2 = cnt_y - half_height

        bbox = torch.Tensor([x1, y1, x2, y2])

        return img, [cls_id, bbox]


def dataset_test(dataloader):
    for data in dataloader:
        image = data[0][0]
        obj_class = data[1][0][0]
        bbox = data[1][1][0]

        image = torch.permute(image, (1, 2, 0))

        np_image = (image.numpy() * 255).astype(np.uint8)
        np_image_mod = np_image.copy()

        height, width, _ = np_image.shape

        x1 = int(bbox[0] * width)
        y1 = int(bbox[1] * height)
        x2 = int(bbox[2] * width)
        y2 = int(bbox[3] * height)

        cv2.rectangle(np_image_mod, (x1, y1), (x2, y2), (255, 0, 0), 2)

        cv2.imshow("test", np_image_mod)
        cv2.waitKey(0)

if __name__ == "__main__":
    train_dataset = TrafficLightDataset("train")
    test_dataset = TrafficLightDataset("test")

    train_dataloader = DataLoader(train_dataset, batch_size=32, shuffle=True)
    dataset_test(train_dataloader)
