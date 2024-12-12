# Setup

Clone the repository on both turtlebot and remote PC
```
git clone https://github.com/Alynie/turtlebot-packages.git
```

## Turtlebot

On turtlebot terminal 1:
```
export TURTLEBOT3_MODEL=waffle_pi
```
```
ros2 launch turtlebot3_bringup robot.launch.py
```

Open another turtlebot terminal 2:
```
cd turtlebot-packages
```
```
colcon build
```
```
cd src/final
```
```
ros2 run final turtlebot
```

## Remote PC

On remote PC:
```
cd turtlebot-packages
```
```
colcon build
```
```
cd src/final
```
```
mkdir images
```
```
ros2 run final navigation
```
