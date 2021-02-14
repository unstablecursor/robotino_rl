
import torch.nn as nn
import torch.nn.functional as F

class DQN(nn.Module):

    def __init__(self, n_inputs, first_layer, second_layer, third_layer, n_actions):
        super(DQN, self).__init__()
        self.fc1 = nn.Linear(int(n_inputs), first_layer)
        self.fc2 = nn.Linear(first_layer, second_layer)
        self.fc3 = nn.Linear(second_layer, third_layer)
        self.fc4 = nn.Linear(third_layer, n_actions)

    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = F.relu(self.fc3(x))

        x = self.fc4(x)
        
        return x