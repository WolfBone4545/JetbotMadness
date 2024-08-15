import torch
from torchvision.models.resnet import resnet18, ResNet18_Weights
from torchinfo import summary

# CONFIGURATION
N_EPOCH = 50
LOG_INDENT = 10

class ResNet_model():
    pass

class ResNet_trainer():
    def __init__(self):
        self.model = resnet18(weights=ResNet18_Weights.IMAGENET1K_V1)
        summary(self.model, (1, 3, 416, 416))

    def train(self):
        pass

    def test(self):
        pass

    def fit(self):
        for epoch in N_EPOCH:
            self.train()
            self.test()


class ResNet_infer():
    pass


if __name__ == "__main__":
    ResNet_trainer()
