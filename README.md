# Mobile System Control
This is the repository for **Mobile System Control** lecture

   ![mpc](https://github.com/rise-lab-skku/Mobile_System_Control/assets/80592399/8638328b-c6c6-4007-86b4-e025b3875c0c)

## Launching CARLA
1. Download CARLA <br/>
   Link: <br/>
2. Launch CARLA
    ```sh
    ./CarlaUE4.sh
    ```

## Build Example Packages
1. Setup **catkin build**
* Change **catkin_make** workspace to **catkin build**
    ```sh
    pip3 install pip --upgrade
    pip3 install -U catkin_tools
    cd ~/catkin_ws/src
    rm CMakeLists.txt
    cd ..
    rm -rf build devel
    mkdir build devel install logs
    catkin init
    catkin build
    ```
2. Build Packages
* Install external library for QP solver
    ```
    cd ~/catkin_ws/src
    git clone https://github.com/rise-lab-skku/Mobile_System_Control.git
    cd Mobile_System_Control/MPC_control_ex
    sudo ./install.sh
    ```
* Build packages
    ```sh
    catkin build
    ```
## Launch Control Examples
1. Launch **carla_ros_bridge**
    ```sh
    roslaunch carla_ros_bridge carla_ros_bridge.launch
    ```
2. Launch **mobile_system_control**
    ```sh
    roslaunch mobile_system_control mobile_system.launch
    ```
    ```sh
    roslaunch mobile_system_control_vis mobile_system_control_vis.launch
    ```
3. Spawn vehicle and control window
    ```sh
    roslaunch carla_spawn_objects carla_spawn_objects.launch objects_file_name:=ego
    ```
    ```
    roslaunch carla_manual_control carla_manual_control.launch role_name:=ego_vehicle
    ```
4. Launch control examples
    * PID controller
        ```
        roslaunch PID_control_ex PID.launch
        ```
    * Pure Pursuit controller
        ```
        roslaunch PurePursuit_control_ex PurePursuit.launch
        ```
    * Kanayama controller
        ```
        roslaunch Kanayama_control_ex Kanayama.launch
        ```
    * MPC controller
        ```
        roslaunch MPC_control_ex MPC.launch
        ```
