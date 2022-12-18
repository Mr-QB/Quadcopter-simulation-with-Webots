import numpy as np


# a = (np.arctan2(0.06,-0.09))
# print(a)

import numpy as np
import matplotlib.pyplot as plt

# theta goes from 0 to 2pi
theta = np.linspace(0, 2*np.pi, 100)
# the radius of the circle
r = np.sqrt(0.05)
# compute x1 and x2
x1 = r*np.cos(theta)
x2 = r*np.sin(theta)
print((x1[:10]))
# create the figure
# fig, ax = plt.subplots(1)
# ax.plot(x1, x2)
# ax.set_aspect(1)
# plt.show()
