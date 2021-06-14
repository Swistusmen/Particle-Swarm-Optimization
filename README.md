# Particle-Swarm-Optimization
C++ implementation of algorithm

Particle Optimization Swarm Implementation:

- artificaial inteligence algorithm for solving searching problems. 

<h2>Build:</h2>

```
mkdir build

cd build

cmake -S .. -B $(pwd)
```

Algorithm was developed twice: fristly in OOP way- to ru only for one thread, secondly for multimethreading (C++20)


<h2>About an algorithm:</h2>

- there is a few agents which are looking for best position (in my implementation minimum). Every of agents have current best location (calculated in current iteration),
best local position and best global position. In every turn, information about newly detected minimum may influence to speed of other agents (global minimum influences, and 
in every turn local minimum may become such a minimum). 

-In every turn: 

a) new velocity is calculated

b) new parameters are calculated

<h2>Sample results get by draw_minimums.py</h2>

![alt text](https://github.com/Swistusmen/Particle-Swarm-Optimization/blob/master/screens/Levi%20function.jpg)
