#!/usr/bin/env python
# coding: utf-8

# In[3]:


import torch; torch.manual_seed(0)
import torch.nn as nn
import torch.nn.functional as F
import torch.utils
import torch.distributions
import numpy as np
import matplotlib.pyplot as plt; plt.rcParams['figure.dpi'] = 200


# In[8]:


class PD_network(nn.Module):
    def __init__(self, input_dim, output_dim):
        super(PD_network, self).__init__()
        self.lstm1 = nn.LSTM(input_dim, 128)
        self.lstm2 = nn.LSTM(128, 128)
        self.linear = nn.Linear(128, output_dim)
    







