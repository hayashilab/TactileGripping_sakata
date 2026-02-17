#!/usr/bin/env python3
"""
事前学習済みモデル（触覚分類用）

VGG16, ResNet18, ResNet34, EfficientNet-B0 をベースにした分類モデル
ImageNetで事前学習した重みを使用し、最終層のみファインチューニング
"""

import torch
import torch.nn as nn
from torchvision import models
from torchvision.models import (
    VGG16_Weights,
    ResNet18_Weights,
    ResNet34_Weights,
    EfficientNet_B0_Weights,
)


class TactileVGG16(nn.Module):
    """
    VGG16ベースの触覚分類モデル

    - 入力: 3ch または 6ch
    - 6chの場合、最初のconv層を置き換え
    """

    def __init__(
        self,
        in_channels: int = 6,
        num_classes: int = 3,
        dropout: float = 0.5,
        pretrained: bool = True,
        freeze_backbone: bool = False,
    ):
        super().__init__()

        self.in_channels = in_channels
        self.num_classes = num_classes

        # VGG16をロード
        weights = VGG16_Weights.IMAGENET1K_V1 if pretrained else None
        base_model = models.vgg16(weights=weights)

        # 最初のconv層を置き換え（6ch対応）
        if in_channels != 3:
            old_conv = base_model.features[0]
            new_conv = nn.Conv2d(
                in_channels, 64,
                kernel_size=3, stride=1, padding=1
            )
            # 3chの重みを複製して初期化
            if pretrained:
                with torch.no_grad():
                    if in_channels == 6:
                        new_conv.weight[:, :3] = old_conv.weight
                        new_conv.weight[:, 3:] = old_conv.weight
                    new_conv.bias = old_conv.bias
            base_model.features[0] = new_conv

        self.features = base_model.features
        self.avgpool = base_model.avgpool

        # 分類器を置き換え
        self.classifier = nn.Sequential(
            nn.Linear(512 * 7 * 7, 4096),
            nn.ReLU(inplace=True),
            nn.Dropout(dropout),
            nn.Linear(4096, 1024),
            nn.ReLU(inplace=True),
            nn.Dropout(dropout),
            nn.Linear(1024, num_classes),
        )

        # バックボーンをフリーズ
        if freeze_backbone:
            for param in self.features.parameters():
                param.requires_grad = False

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = self.features(x)
        x = self.avgpool(x)
        x = torch.flatten(x, 1)
        x = self.classifier(x)
        return x


class TactileResNet(nn.Module):
    """
    ResNet18/34ベースの触覚分類モデル
    """

    def __init__(
        self,
        in_channels: int = 6,
        num_classes: int = 3,
        dropout: float = 0.3,
        pretrained: bool = True,
        freeze_backbone: bool = False,
        variant: str = "resnet18",  # "resnet18" or "resnet34"
    ):
        super().__init__()

        self.in_channels = in_channels
        self.num_classes = num_classes
        self.variant = variant

        # ResNetをロード
        if variant == "resnet18":
            weights = ResNet18_Weights.IMAGENET1K_V1 if pretrained else None
            base_model = models.resnet18(weights=weights)
            fc_in_features = 512
        elif variant == "resnet34":
            weights = ResNet34_Weights.IMAGENET1K_V1 if pretrained else None
            base_model = models.resnet34(weights=weights)
            fc_in_features = 512
        else:
            raise ValueError(f"Unknown variant: {variant}")

        # 最初のconv層を置き換え（6ch対応）
        if in_channels != 3:
            old_conv = base_model.conv1
            new_conv = nn.Conv2d(
                in_channels, 64,
                kernel_size=7, stride=2, padding=3, bias=False
            )
            if pretrained:
                with torch.no_grad():
                    if in_channels == 6:
                        new_conv.weight[:, :3] = old_conv.weight
                        new_conv.weight[:, 3:] = old_conv.weight
            base_model.conv1 = new_conv

        # 特徴抽出部分
        self.conv1 = base_model.conv1
        self.bn1 = base_model.bn1
        self.relu = base_model.relu
        self.maxpool = base_model.maxpool
        self.layer1 = base_model.layer1
        self.layer2 = base_model.layer2
        self.layer3 = base_model.layer3
        self.layer4 = base_model.layer4
        self.avgpool = base_model.avgpool

        # 分類器
        self.classifier = nn.Sequential(
            nn.Linear(fc_in_features, 256),
            nn.ReLU(inplace=True),
            nn.Dropout(dropout),
            nn.Linear(256, num_classes),
        )

        # バックボーンをフリーズ
        if freeze_backbone:
            for name, param in self.named_parameters():
                if not name.startswith('classifier'):
                    param.requires_grad = False

    def forward(self, x: torch.Tensor) -> torch.Tensor:
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


class TactileEfficientNet(nn.Module):
    """
    EfficientNet-B0ベースの触覚分類モデル
    """

    def __init__(
        self,
        in_channels: int = 6,
        num_classes: int = 3,
        dropout: float = 0.3,
        pretrained: bool = True,
        freeze_backbone: bool = False,
    ):
        super().__init__()

        self.in_channels = in_channels
        self.num_classes = num_classes

        # EfficientNet-B0をロード
        weights = EfficientNet_B0_Weights.IMAGENET1K_V1 if pretrained else None
        base_model = models.efficientnet_b0(weights=weights)

        # 最初のconv層を置き換え（6ch対応）
        if in_channels != 3:
            old_conv = base_model.features[0][0]
            new_conv = nn.Conv2d(
                in_channels, 32,
                kernel_size=3, stride=2, padding=1, bias=False
            )
            if pretrained:
                with torch.no_grad():
                    if in_channels == 6:
                        new_conv.weight[:, :3] = old_conv.weight
                        new_conv.weight[:, 3:] = old_conv.weight
            base_model.features[0][0] = new_conv

        self.features = base_model.features
        self.avgpool = base_model.avgpool

        # 分類器（EfficientNet-B0の出力は1280次元）
        self.classifier = nn.Sequential(
            nn.Dropout(dropout),
            nn.Linear(1280, 256),
            nn.ReLU(inplace=True),
            nn.Dropout(dropout),
            nn.Linear(256, num_classes),
        )

        # バックボーンをフリーズ
        if freeze_backbone:
            for param in self.features.parameters():
                param.requires_grad = False

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = self.features(x)
        x = self.avgpool(x)
        x = torch.flatten(x, 1)
        x = self.classifier(x)
        return x


def create_pretrained_model(
    model_name: str,
    in_channels: int = 6,
    num_classes: int = 3,
    dropout: float = 0.3,
    pretrained: bool = True,
    freeze_backbone: bool = False,
    device: str = "cuda",
) -> nn.Module:
    """
    事前学習済みモデルを作成

    Args:
        model_name: "vgg16", "resnet18", "resnet34", "efficientnet"
        in_channels: 入力チャンネル数
        num_classes: 出力クラス数
        dropout: Dropout率
        pretrained: 事前学習済み重みを使用
        freeze_backbone: バックボーンをフリーズ
        device: デバイス

    Returns:
        モデル
    """
    model_name = model_name.lower()

    if model_name == "vgg16":
        model = TactileVGG16(
            in_channels=in_channels,
            num_classes=num_classes,
            dropout=dropout,
            pretrained=pretrained,
            freeze_backbone=freeze_backbone,
        )
    elif model_name == "resnet18":
        model = TactileResNet(
            in_channels=in_channels,
            num_classes=num_classes,
            dropout=dropout,
            pretrained=pretrained,
            freeze_backbone=freeze_backbone,
            variant="resnet18",
        )
    elif model_name == "resnet34":
        model = TactileResNet(
            in_channels=in_channels,
            num_classes=num_classes,
            dropout=dropout,
            pretrained=pretrained,
            freeze_backbone=freeze_backbone,
            variant="resnet34",
        )
    elif model_name == "efficientnet":
        model = TactileEfficientNet(
            in_channels=in_channels,
            num_classes=num_classes,
            dropout=dropout,
            pretrained=pretrained,
            freeze_backbone=freeze_backbone,
        )
    else:
        raise ValueError(f"Unknown model: {model_name}")

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

    models_to_test = ["vgg16", "resnet18", "resnet34", "efficientnet"]

    for model_name in models_to_test:
        print(f"\n{'='*50}")
        print(f"Testing {model_name}")
        print(f"{'='*50}")

        model = create_pretrained_model(
            model_name=model_name,
            in_channels=6,
            num_classes=3,
            pretrained=True,
            freeze_backbone=False,
            device=device,
        )

        total_params = count_parameters(model, trainable_only=False)
        trainable_params = count_parameters(model, trainable_only=True)

        print(f"Total parameters: {total_params:,}")
        print(f"Trainable parameters: {trainable_params:,}")

        # Forward test
        x = torch.randn(2, 6, 224, 224).to(device)
        with torch.no_grad():
            out = model(x)
        print(f"Input shape: {x.shape}")
        print(f"Output shape: {out.shape}")

    print("\nAll tests passed!")
