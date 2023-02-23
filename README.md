# Scientific Working in Computational Engineering
In this repository, I maintain the code for the Scientific Working in Computational Engineering Course at the [Institute of Digital and Autonomous Construction](https://www.tuhh.de/idac/research.html). The task for this project was to create a digital twin of a robotic arm and moving platform system that coordinates the movement in the context of a cement printing. As a simulation environment, I chose the [WeBots](https://www.cyberbotics.com/) simulator and implemented a simple controler that maximizes the potential printing area.

## How to run

First, clone the repository.

```
git clone https://github.com/jonathan-hellwig/scientific_working.git
```
Install python <= 3.9.
Then, using a python package manager install all required packages.t

```
pip install -r requirements.txt
```

Download and install the latest version of [Webots](https://cyberbotics.com/#download).

Open up Webots and select the the world file in the worlds subdirectory.
![Open world file](doc/open_world_file_1.png)
![Open world file](doc/open_world_file_2.png)

Then, the simulation should automatically start running. However, the first run always fails because some values are not initialized correctly. Press ctrl+shift+T to reset the simulation.
