import numpy as np
import matplotlib.pyplot as plt

with open("groundtruth.txt") as f:
    data = f.readlines()

x = [line.split()[0] for line in data]
y = [line.split()[1] for line in data]
z = [line.split()[2] for line in data]

ax = plt.axes(projection='3d')
ax.plot(x, y, z, 'gray')
plt.show()