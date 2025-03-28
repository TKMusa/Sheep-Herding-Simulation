# Sheep Herding Simulation
## Demostrations

| **Baseline** | **Strong Seperation** |
|------------|------------|
| ![Baseline](https://github.com/user-attachments/assets/6b7c7012-5bd3-4f39-a492-c2c06530d4d8) | ![Seperation](https://github.com/user-attachments/assets/bad4afaf-2996-4fa7-bab1-0df1bf0fbb90) |

| **Weak cohesion with large range** | **Strong cohesion with small range** |
|------------|------------|
| ![coh_dist](https://github.com/user-attachments/assets/cfe4c131-a0bd-4b8f-9884-ca8f638a2296) | ![coh_strength](https://github.com/user-attachments/assets/5b23ef93-3053-4349-8465-6414418fdd77) |

## Setup

build package
```js
colcon build
```

source installation
```js
source install/local_setup.bash
```

run desired launch file
```js
ros2 launch sheep_simulation launch.py // runs simulation + rviz2 with preloaded config

// development
// run launch files on separate terminals
// simulation terminal can be interrupted and re-launched after building, to run simulation again

ros2 launch sheep_simulation launch_rviz.py // runs rviz2 with preloaded config
ros2 launch sheep_simulation launch_sim.py // runs simulation only
```
## Configurations
Herding properties can be changed within sheep_node.py found in:
```js
src/sheep_simulation/sheep_simulation/PsuedoSheepSeperationPort.py
```
#### Regular baseline
-	Separation Range = 1.0 ; Cohesion Range = 8.0 ; Separation Weight = 1.0 ; Cohesion Weight = 0.3
#### Large separation range
-	Separation Range = 2.5 ; Cohesion Range = 8.0 ; Separation Weight = 1.0 ; Cohesion Weight = 0.3
#### Weak cohesion strength with large range
-	Separation Range = 1.0 ; Cohesion Range = 16.0 ; Separation Weight = 1.0 ; Cohesion Weight = 0.1
#### Strong cohesion strength with a small range
-	Separation Range = 1.0 ; Cohesion Range = 3.0 ; Separation Weight = 1.0 ; Cohesion Weight = 0.6
