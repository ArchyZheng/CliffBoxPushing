import torch.nn as nn


class MLPNet(nn.Module):
    """
    this is just a small simple.

    MLP(num_observer, 64) -> MLP(64, 64) -> MLP(64, 64) -> MLP(64, num_action)

    each MLP has a ReLU activation function.
    """
    def __init__(self, num_observer, num_action) -> None:
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(num_observer, 64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64, num_action),
            nn.ReLU()
        )

    def forward(self, x):
        return self.net(x) * 20 - 10
