# default_franka_controllers
Default controllers for franka fr3
## Usage
### Build
```bash
cd ros2_ws # your ros2 workspace
colcon build
source install/setup.bash
```
### Run
- Launch example.launch.py with controller_name argument set
```bash
ros2 launch dyros_fr3_controllers example.launch.py controller_name:=default_controller
```
```
    'robot_config_file',
    default_value=PathJoinSubstitution([FindPackageShare('dyros_fr3_controllers'), 'config', 'franka.config.yaml']),
    description='Path to the robot configuration file to load',

    'controller_name',
    description='Name of the controller to spawn (required, no default)',
```
### Generate new controllers
- run add_controllers.py with --controller_name, --control_mode argument set
```bash
cd ~/ros2_ws/src/default_franka_controllers
python3 add_controller.py --controller_name TestEffortController --control_mode effort
```
```
"--controller_name", type=str, 
                     required=True, 
                     help="Name of Controller"

"--control_mode", type=str, 
                  required=True, 
                  help="Control Mode [position, velocity, effort]"
```
