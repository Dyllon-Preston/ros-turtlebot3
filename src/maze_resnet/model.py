"""

MazeSignResNet: A ResNet-based model for sign language recognition in a maze environment.
This model is designed to classify images of signs into one of several categories.

Group Members:
- Dyllon Preston
- Richi Dubey

"""

import torch
import torch.nn as nn
import torch.nn.functional as F

class BasicBlock(nn.Module):
    """A simple ResNet basic block: two 3x3 convs + skip connection."""
    expansion = 1

    def __init__(self, in_planes, planes, stride=1):
        super(BasicBlock, self).__init__()
        # First conv
        self.conv1 = nn.Conv2d(in_planes, planes, kernel_size=3,
                               stride=stride, padding=1, bias=False)
        self.bn1 = nn.BatchNorm2d(planes)
        # Second conv
        self.conv2 = nn.Conv2d(planes, planes, kernel_size=3,
                               stride=1, padding=1, bias=False)
        self.bn2 = nn.BatchNorm2d(planes)

        # Skip connection if shape changes
        self.downsample = None
        if stride != 1 or in_planes != planes * self.expansion:
            self.downsample = nn.Sequential(
                nn.Conv2d(in_planes, planes * self.expansion,
                          kernel_size=1, stride=stride, bias=False),
                nn.BatchNorm2d(planes * self.expansion)
            )

    def forward(self, x):
        identity = x

        out = self.bn1(self.conv1(x))
        out = F.relu(out, inplace=True)

        out = self.bn2(self.conv2(out))

        if self.downsample is not None:
            identity = self.downsample(x)

        out += identity
        return F.relu(out, inplace=True)


class MazeSignResNet(nn.Module):
    def __init__(self, num_classes=6, layers=[2,2,2,2], in_channels=3, dropout_prob=0.5):
        super(MazeSignResNet, self).__init__()
        self.in_planes = 64

        # Initial conv + BN + ReLU + maxpool
        self.conv1 = nn.Conv2d(in_channels, 64, kernel_size=7, stride=2, padding=3, bias=False)
        self.bn1   = nn.BatchNorm2d(64)
        self.relu  = nn.ReLU(inplace=True)
        self.maxpool = nn.MaxPool2d(kernel_size=3, stride=2, padding=1)

        # ResNet stages
        self.layer1 = self._make_layer(64,  layers[0], stride=1)
        self.layer2 = self._make_layer(128, layers[1], stride=2)
        self.layer3 = self._make_layer(256, layers[2], stride=2)
        self.layer4 = self._make_layer(512, layers[3], stride=2)

        # Spatial dropout on feature maps after layer4
        self.feature_dropout = nn.Dropout2d(p=dropout_prob * 0.5)

        # Global pooling + small classifier head with dropout
        self.avgpool = nn.AdaptiveAvgPool2d((1,1))
        self.classifier = nn.Sequential(
            nn.Flatten(),
            nn.Linear(512 * BasicBlock.expansion, 256),
            nn.ReLU(inplace=True),
            nn.Dropout(p=dropout_prob),
            nn.Linear(256, num_classes)
        )

        # Initialize weights
        for m in self.modules():
            if isinstance(m, nn.Conv2d):
                nn.init.kaiming_normal_(m.weight, mode='fan_out', nonlinearity='relu')
            elif isinstance(m, nn.BatchNorm2d):
                nn.init.constant_(m.weight, 1)
                nn.init.constant_(m.bias, 0)

    def _make_layer(self, planes, blocks, stride):
        layers = []
        layers.append(BasicBlock(self.in_planes, planes, stride))
        self.in_planes = planes * BasicBlock.expansion
        for _ in range(1, blocks):
            layers.append(BasicBlock(self.in_planes, planes))
        return nn.Sequential(*layers)

    def forward(self, x):

        # Stem
        x = self.relu(self.bn1(self.conv1(x)))
        x = self.maxpool(x)

        # Stages
        x = self.layer1(x)
        x = self.layer2(x)
        x = self.layer3(x)
        x = self.layer4(x)

        # Feature‐map dropout
        x = self.feature_dropout(x)

        # Pool + classifier
        x = self.avgpool(x)        # [B, 512, 1, 1]
        x = self.classifier(x)     # [B, num_classes]
        return x


if __name__ == '__main__':
    # Test the model
    model = MazeSignResNet(num_classes=6, dropout_prob=0.5)
    dummy = torch.randn(1, 3, 240, 320)
    out = model(dummy)
    print("Output shape:", out.shape)  # [1, 6]
