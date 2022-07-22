# Robot-Formation-Control
Formation control for a multi-agent system in a bidimensional space. Various types of formations available.
The implemented control laws are described in this [paper](https://arxiv.org/pdf/1506.05636.pdf). The approach is a **bearing-based** one.
This project was developed as course project for the course *Distributed Autonomous Systems* at University of Bologna. 

The main files of the project are the *launch file*, which launchs all the agents as ROS nodes, and the *agent_i.py*, which is is executed by each agent.
The agents are divided into leaders and followers, the system is distributed because each agent uses only information from its neighbors. The communication is described by a fixed graph.

To define the **formation type**, modify the following line in the launch file:
```
formation_type = 'square' # <---- modify this to change formation type
```

Formation types:
- `square`               <--- square formation, fixed
- `moving_square`        <--- square formation, leaders move with constant velocity (piecewise constant)
- `letters`              <--- the agents forms, one, after the other, the letters of the word BALA
- `moving_square_circle` <--- square formation, leaders move following circular pattern (TIME-VARYING LEADERS VELOCITY CASE)

**Visualization**: each simulation will generate a *csv file* for each agent, by executing the *plot_cvs.py* file these files are read and the trajectories are plotted using matplotlib.
For alle the formation types except letters, also Rviz visualizaion is available during the simulation execution.

Some **videos**:

https://user-images.githubusercontent.com/100198704/180421279-d233fd80-b753-4803-a8ed-bf9a0f110884.mp4




https://user-images.githubusercontent.com/100198704/180422646-84dbcf8c-f15a-4a90-98b6-c3eb75822721.mp4



https://user-images.githubusercontent.com/100198704/180422681-af51afea-3818-4b78-9bce-656de6f01a3e.mp4

**Further improvements**:
- each agent compute the bearings relative to neighbors only
- the uniqueness of the formation is verified in a reliable way also in the letters case
- control with integral action
