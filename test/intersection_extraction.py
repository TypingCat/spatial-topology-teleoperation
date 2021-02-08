#!/usr/bin/env python3

import pickle

with open('data.pkl', 'rb') as f:
    data = pickle.load(f)
    
print(data)