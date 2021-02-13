
import torch.nn as nn
import torch.nn.functional as F
from components.parameters import N_INPUTS,FIRST_LAYER, SECOND_LAYER, THIRD_LAYER, N_ACTIONS

class DQN(nn.Module):

    def __init__(self):
        super(DQN, self).__init__()
        self.fc1 = nn.Linear(int(N_INPUTS), FIRST_LAYER)
        self.fc2 = nn.Linear(FIRST_LAYER, SECOND_LAYER)
        self.fc3 = nn.Linear(SECOND_LAYER, THIRD_LAYER)
        self.fc4 = nn.Linear(THIRD_LAYER, N_ACTIONS)

    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = F.relu(self.fc3(x))

        x = self.fc4(x)
        
        return x