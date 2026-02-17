#!/usr/bin/env python3
"""
つぶれ検出モデル定義

対応モデル:
- CrushDetectorCNN: カスタム軽量CNN
- CrushDetectorVGG16: VGG16ベース
- CrushDetectorResNet18: ResNet18ベース
- CrushDetectorEfficientNet: EfficientNet-B0ベース
"""

import torch
import torch.nn as nn
from torchvision import models
from torchvision.models import (
    VGG16_Weights,
    ResNet18_Weights,
    EfficientNet_B0_Weights,
)


ALL_MODELS = ["cnn", "vgg16", "resnet18", "efficientnet"]


class CrushDetectorCNN(nn.Module):
    """カスタムCNN（2値分類）"""

    def __init__(self, in_channels: int = 6, dropout: float = 0.3):
        super().__init__()

        self.features = nn.Sequential(
            nn.Conv2d(in_channels, 32, 3, stride=2, padding=1),
            nn.BatchNorm2d(32),
            nn.ReLU(inplace=True),

            nn.Conv2d(32, 64, 3, stride=2, padding=1),
            nn.BatchNorm2d(64),
            nn.ReLU(inplace=True),

            nn.Conv2d(64, 128, 3, stride=2, padding=1),
            nn.BatchNorm2d(128),
            nn.ReLU(inplace=True),

            nn.Conv2d(128, 256, 3, stride=2, padding=1),
            nn.BatchNorm2d(256),
            nn.ReLU(inplace=True),

            nn.AdaptiveAvgPool2d(1),
        )

        self.classifier = nn.Sequential(
            nn.Flatten(),
            nn.Linear(256, 64),
            nn.ReLU(inplace=True),
            nn.Dropout(dropout),
            nn.Linear(64, 1),
        )

    def forward(self, x):
        x = self.features(x)
        x = self.classifier(x)
        return x


class CrushDetectorVGG16(nn.Module):
    """VGG16ベース（2値分類）"""

    def __init__(
        self,
        in_channels: int = 6,
        dropout: float = 0.5,
        pretrained: bool = True,
        freeze_backbone: bool = False,
    ):
        super().__init__()

        weights = VGG16_Weights.IMAGENET1K_V1 if pretrained else None
        base_model = models.vgg16(weights=weights)

        if in_channels != 3:
            old_conv = base_model.features[0]
            new_conv = nn.Conv2d(in_channels, 64, kernel_size=3, stride=1, padding=1)
            if pretrained:
                with torch.no_grad():
                    if in_channels == 6:
                        new_conv.weight[:, :3] = old_conv.weight
                        new_conv.weight[:, 3:] = old_conv.weight
                    new_conv.bias = old_conv.bias
            base_model.features[0] = new_conv

        self.features = base_model.features
        self.avgpool = base_model.avgpool

        self.classifier = nn.Sequential(
            nn.Linear(512 * 7 * 7, 1024),
            nn.ReLU(inplace=True),
            nn.Dropout(dropout),
            nn.Linear(1024, 256),
            nn.ReLU(inplace=True),
            nn.Dropout(dropout),
            nn.Linear(256, 1),
        )

        if freeze_backbone:
            for param in self.features.parameters():
                param.requires_grad = False

    def forward(self, x):
        x = self.features(x)
        x = self.avgpool(x)
        x = torch.flatten(x, 1)
        x = self.classifier(x)
        return x


class CrushDetectorResNet18(nn.Module):
    """ResNet18ベース（2値分類）"""

    def __init__(
        self,
        in_channels: int = 6,
        dropout: float = 0.3,
        pretrained: bool = True,
        freeze_backbone: bool = False,
    ):
        super().__init__()

        weights = ResNet18_Weights.IMAGENET1K_V1 if pretrained else None
        base_model = models.resnet18(weights=weights)

        if in_channels != 3:
            old_conv = base_model.conv1
            new_conv = nn.Conv2d(in_channels, 64, kernel_size=7, stride=2, padding=3, bias=False)
            if pretrained:
                with torch.no_grad():
                    if in_channels == 6:
                        new_conv.weight[:, :3] = old_conv.weight
                        new_conv.weight[:, 3:] = old_conv.weight
            base_model.conv1 = new_conv

        self.conv1 = base_model.conv1
        self.bn1 = base_model.bn1
        self.relu = base_model.relu
        self.maxpool = base_model.maxpool
        self.layer1 = base_model.layer1
        self.layer2 = base_model.layer2
        self.layer3 = base_model.layer3
        self.layer4 = base_model.layer4
        self.avgpool = base_model.avgpool

        self.classifier = nn.Sequential(
            nn.Linear(512, 128),
            nn.ReLU(inplace=True),
            nn.Dropout(dropout),
            nn.Linear(128, 1),
        )

        if freeze_backbone:
            for name, param in self.named_parameters():
                if not name.startswith('classifier'):
                    param.requires_grad = False

    def forward(self, x):
        x = self.conv1(x)
        x = self.bn1(x)
        x = self.relu(x)
        x = self.maxpool(x)
        x = self.layer1(x)
        x = self.layer2(x)
        x = self.layer3(x)
        x = self.layer4(x)
        x = self.avgpool(x)
        x = torch.flatten(x, 1)
        x = self.classifier(x)
        return x


class CrushDetectorEfficientNet(nn.Module):
    """EfficientNet-B0ベース（2値分類）"""

    def __init__(
        self,
        in_channels: int = 6,
        dropout: float = 0.3,
        pretrained: bool = True,
        freeze_backbone: bool = False,
    ):
        super().__init__()

        weights = EfficientNet_B0_Weights.IMAGENET1K_V1 if pretrained else None
        base_model = models.efficientnet_b0(weights=weights)

        if in_channels != 3:
            old_conv = base_model.features[0][0]
            new_conv = nn.Conv2d(in_channels, 32, kernel_size=3, stride=2, padding=1, bias=False)
            if pretrained:
                with torch.no_grad():
                    if in_channels == 6:
                        new_conv.weight[:, :3] = old_conv.weight
                        new_conv.weight[:, 3:] = old_conv.weight
            base_model.features[0][0] = new_conv

        self.features = base_model.features
        self.avgpool = base_model.avgpool

        self.classifier = nn.Sequential(
            nn.Dropout(dropout),
            nn.Linear(1280, 256),
            nn.ReLU(inplace=True),
            nn.Dropout(dropout),
            nn.Linear(256, 1),
        )

        if freeze_backbone:
            for param in self.features.parameters():
                param.requires_grad = False

    def forward(self, x):
        x = self.features(x)
        x = self.avgpool(x)
        x = torch.flatten(x, 1)
        x = self.classifier(x)
        return x


def create_crush_model(
    model_name: str,
    in_channels: int = 6,
    dropout: float = 0.3,
    pretrained: bool = True,
    freeze_backbone: bool = False,
    device: str = "cuda",
) -> nn.Module:
    """モデル作成"""
    model_name = model_name.lower()

    if model_name == "cnn":
        model = CrushDetectorCNN(in_channels=in_channels, dropout=dropout)
    elif model_name == "vgg16":
        model = CrushDetectorVGG16(
            in_channels=in_channels,
            dropout=dropout,
            pretrained=pretrained,
            freeze_backbone=freeze_backbone,
        )
    elif model_name == "resnet18":
        model = CrushDetectorResNet18(
            in_channels=in_channels,
            dropout=dropout,
            pretrained=pretrained,
            freeze_backbone=freeze_backbone,
        )
    elif model_name == "efficientnet":
        model = CrushDetectorEfficientNet(
            in_channels=in_channels,
            dropout=dropout,
            pretrained=pretrained,
            freeze_backbone=freeze_backbone,
        )
    else:
        raise ValueError(f"Unknown model: {model_name}. Available: {ALL_MODELS}")

    return model.to(device)


def count_parameters(model: nn.Module, trainable_only: bool = True) -> int:
    """パラメータ数をカウント"""
    if trainable_only:
        return sum(p.numel() for p in model.parameters() if p.requires_grad)
    return sum(p.numel() for p in model.parameters())


if __name__ == "__main__":
    # テスト
    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"Device: {device}")

    for model_name in ALL_MODELS:
        print(f"\n{model_name}:")
        model = create_crush_model(model_name, in_channels=6, device=device)
        print(f"  Params: {count_parameters(model):,}")

        x = torch.randn(2, 6, 224, 224).to(device)
        with torch.no_grad():
            out = model(x)
        print(f"  Output: {out.shape}")
