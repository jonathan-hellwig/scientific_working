# Scientific Working in Computational Engineering 
## How to run

First, clone the repository.

```
git clone https://github.com/jonathan-hellwig/scientific_working.git
```
Install python <= 3.9.
Then, using a python package manager install all required packages.

```
pip install -r requirements.txt
```

Download and install the latest version of [Webots](https://cyberbotics.com/#download).

Open up Webots and select the the world file in the worlds subdirectory.
![Open world file](doc/open_world_file_1.png)
![Open world file](doc/open_world_file_2.png)

Then, the simulation should automatically start running. However, the first run always fails because some values are not initialized correctly. Press ctrl+shift+T to reset the simulation.
