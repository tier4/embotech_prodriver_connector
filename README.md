# Integration of Embotech's PRODRIVER® with Autoware

Videos of Autoware running with PRODRIVER®.
- https://youtu.be/3jUk-1i_URQ
- https://youtu.be/p3hp5tAY7hU

## Initial setup

Setup the Autoware workspace using branch `2022.12`.
```bash
git clone -b 2022.12 git@github.com:autowarefoundation/autoware.git
cd autoware
mkdir src
vcs import src < autoware.repos
```

Clone the `embotech_prodriver_connector`.
```bash
git clone git@github.com:tier4/embotech_prodriver_connector.git src/universe/external/embotech_prodriver_connector
```

Install dependencies
```bash
source /opt/ros/humble/setup.bash
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

(optional) Copy your version of PRODRIVER® to the `autoware` folder.
If you skip this step, you need to manually configure the path to PRODRIVER® in some of the other steps.
```bash
mkdir embotech
ln -s /path/to/your/prodriver embotech
```

### Map setup

The map used by PRODRIVER® and the connector also need to be configured to match the map used by Autoware.
- In `config/scenario.json`: parameter `ptclAreaMapFile` must be updated to the correct map path.
- In `src/universe/external/embotech_prodriver_connector/param/embotech_prodriver_connector.default.param.yaml`: parameters for the `map_origin` must be updated for the corresponding map.

## Building

If you did not copy your version of PRODRIVER® to the `autoware/embotech` folder, you need to set an environment variable `PTCL_PATH` with the path to your PRODRIVER® folder containing the `ptcl` library.
```bash
export PTCL_PATH=path/to/prodriver/
```
Build Autoware and the connector.
```bash
colcon build --symlink-install
```

## Running Autoware with PRODRIVER®.

Source the setup script.
```bash
source install/setup.sh
```

Set the path to the map (`$MAP`), the vehicle (`$VEHICLE`) and sensor (`$SENSOR`) to use, and run Autoware _without its planning module_.
- Planning simulator:
```bash
ros2 launch autoware_launch planning_simulator.launch.xml map_path:="$MAP" vehicle_model:="$VEHICLE" sensor_model:="$SENSOR" launch_planning:=false
```
- Real vehicle:
```bash
ros2 launch autoware_launch autoware.launch.xml map_path:="$MAP" vehicle_model:="$VEHICLE" sensor_model:="$SENSOR" launch_planning:=false
```
Run PRODRIVER® and the connector.
- Set the path to PRODRIVER (`$PRODRIVER_PATH`).
- If your version of PRODRIVER® is installed in the `autoware` workspace you can skip setting the `prodriver_path`.
```bash
ros2 launch embotech_prodriver_connector planning.launch.xml prodriver_path:="$PRODRIVER_PATH" vehicle_model:="$VEHICLE"
```
