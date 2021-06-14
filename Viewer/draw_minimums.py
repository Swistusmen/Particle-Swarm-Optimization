import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D

data=pd.read_csv("data1.txt",sep=" ",names=["x","y","z"])
a=data.values.tolist()

def fun(x,y):
    #return pow(x * x + y - 11, 2) + pow(x + y * y - 7, 2)
    return (
        np.sin(3 * np.pi * x) ** 2
        + ((x - 1) ** 2) * (1 + np.sin(3 * np.pi * y) ** 2)
        + ((y - 1) ** 2) * (1 + np.sin(2 * np.pi * y) ** 2)
    )
    '''
    return (
        20 + x ** 2 - 10 * np.cos(2 * 3.14159 * x) + y ** 2 - 10 * np.cos(2 * 3.14159 * y)
    )'''

x=np.linspace(-5.12, 5.12,30)
y=np.linspace(-5.12,5.12,30)

X, Y= np.meshgrid(x,y)
Z= fun(X,Y)

fig = plt.figure(figsize=(7,7))

ax= plt.axes(projection='3d')

for i in a:
    ax.scatter(i[0],i[1],i[2])
#ax.scatter(2,3,2)


ax.contour3D(X,Y,Z,50,cmap="viridis")

ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')


plt.show()