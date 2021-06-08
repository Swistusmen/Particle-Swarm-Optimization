import matplotlib.pyplot as plt
import numpy as np

from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D

def fun(x,y):
    return pow(x * x + y - 11, 2) + pow(x + y * y - 7, 2)

x=np.linspace(-5.12, 5.12,30)
y=np.linspace(-5.12,5.12,30)

X, Y= np.meshgrid(x,y)
Z= fun(X,Y)

fig = plt.figure(figsize=(4,4))

ax= plt.axes(projection='3d')

ax.contour3D(X,Y,Z,50,cmap="viridis")

ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')

plt.show()
