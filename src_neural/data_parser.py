import torch
from torch.nn.functional import one_hot
from torch.utils.data import Dataset, DataLoader
from torchvision.io import read_image

from pathlib import Path
import os


class TrafficLightDataset(Dataset):
    def __init__(self, type):
        self.abs_path = os.path.join(Path(__file__).parent, f"source/{type}/")

        self.images_path = os.path.join(self.abs_path, "images/")
        self.labels_path = os.path.join(self.abs_path, "labels/")

        self.images = os.listdir(self.images_path)
        self.labels = os.listdir(self.labels_path)

        self.image_data = []
        self.label_data = []

        # read all images and save them into memory
        for image in self.images:
            # get X
            img_path = os.path.join(self.images_path, image)
            img = read_image(img_path) / 255 # load and normalize
            self.image_data.append(img)

            # get Y
            label_path = os.path.join(self.labels_path, image.replace(".jpg", "") + ".txt")
            label = open(label_path).read()

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
            self.label_data.append([cls_id, bbox])

        print("Loading data done")

    def __len__(self):
        return len(self.label_data)

    def __getitem__(self, idx):
        pass


if __name__ == "__main__":
    train_dataset = TrafficLightDataset("train")
    test_dataset = TrafficLightDataset("test")