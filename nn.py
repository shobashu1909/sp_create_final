import torch.nn as nn

# 5 layers
class KinematicMLP(nn.Module):
    def __init__(self, input_size=9, output_size=9):
        super(KinematicMLP, self).__init__()
        self.net = nn.Sequential(
            nn.Linear(input_size, 256),
            nn.ReLU(),
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, 32),
            nn.ReLU(),
            nn.Linear(32, output_size)
        )

    def forward(self, x):
        return self.net(x)

# input : actuation commands (9 values)
# output : position of the 3 sections (9 values) not the base
# model = KinematicMLP(input_size=9, output_size=9)


class InverseKinematicMLP(nn.Module):
    def __init__(self, input_size=3, output_size=9):
        super(InverseKinematicMLP, self).__init__()
        self.net = nn.Sequential(
            nn.Linear(input_size, 256),
            nn.ReLU(),
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, 32),
            nn.ReLU(),
            nn.Linear(32, output_size)
        )

    def forward(self, x):
        return self.net(x)