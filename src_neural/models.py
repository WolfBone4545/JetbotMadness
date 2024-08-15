import torch
from torchvision.models.resnet import resnet18, ResNet18_Weights
from torchinfo import summary

# CONFIGURATION
N_EPOCH = 50
LOG_INDENT = 10


class ResNet_model:
    def __init__(self):
        self.backbone = resnet18(weights=ResNet18_Weights.IMAGENET1K_V1)

        # TODO: modify functional layer

    def load_checkpoint(self, checkpoint_path):
        pass


class ResNet_trainer:
    def __init__(self):
        self.model = ResNet_model()
        summary(self.model, (1, 3, 416, 416))

    def train(self):
        pass

    def test(self):
        pass

    def fit(self):
        for epoch in N_EPOCH:
            self.train()
            self.test()


class ResNet_infer:
    def __init__(self):
        self.model = ResNet_model()
        self.model.load_checkpoint()

    def infer(self, image):
        input_tensor = torch.from_numpy(image)
        output = self.model(input_tensor)

        bbox = output[:3]
        cls_id = output[3:]

        print(bbox)
        print(cls_id)


if __name__ == "__main__":
    ResNet_trainer()