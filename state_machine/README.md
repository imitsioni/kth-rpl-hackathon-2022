# kth-rpl-hackathon-2022
Repository hosting the code for the 2022 Robotics, Perception and Learning hackathon

## installation
### Step 1: Install ROS
For ubuntu 20.04, it is recommended to install ROS Noetic. follow the website [http://wiki.ros.org/noetic/Installation](http://wiki.ros.org/noetic/Installation)

1. Configure your Ubuntu repositories
Configure your Ubuntu repositories to allow "restricted," "universe," and "multiverse." You can follow the Ubuntu guide for instructions on doing this.

2. Setup your sources.list
Setup your computer to accept software from packages.ros.org. 
    ```
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    ```


3. Set up your keys
    ```
    sudo apt install curl # if you haven't already installed curl
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    ```

4. 
    ```
    sudo apt update
    ```

5. Now pick how much of ROS you would like to install.

    Desktop-Full Install: (Recommended) : Everything in Desktop plus 2D/3D simulators and 2D/3D perception packages

    ```
    sudo apt install ros-noetic-desktop-full
    ```

6. 
    ```
    source /opt/ros/noetic/setup.bash
    ```
7. Dependencies for building packages
    ```
    $ sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
    $ sudo apt install python3-rosdep
    $ sudo rosdep init
    $ rosdep update
    ```
8. Create a ROS Workspace
    ```
    $ mkdir -p ~/ros_ws/src
    $ cd ~/ros_ws/
    $ catkin_make # if fail try, catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 instead
    $ source devel/setup.bash
    ```

### Step 2: Install Baxter 
Partly refer to the repo [https://github.com/skumra/baxter-pnp](https://github.com/skumra/baxter-pnp)
1. Install SDK Dependencies
    ```
    sudo apt-get install python3-wstool python3-rosdep
    ```
2. Install Baxter Research Robot SDK
    ```
    $ cd ~/ros_ws/src
    $ wstool init .
    $ wstool merge https://raw.githubusercontent.com/RethinkRobotics/baxter/master/baxter_sdk.rosinstall
    $ wstool update
    $ source /opt/ros/noetic/setup.bash
    ```
3. Install Baxter Gazebo simulation
    ```
    $ cd ~/ros_ws/src
    $ git clone https://github.com/RethinkRobotics/baxter_simulator.git
    ```

5. 
    ```
    source /opt/ros/noetic/setup.bash
    ```
6. Build and Install
    
    ```
    $ cd ~/ros_ws
    $ catkin_make # if fail try, catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 instead
    $ catkin_make install
    ```
    **!!! Trouble shooting !!!**
    
    - if error with **effort-controllers**, refer to [https://github.com/skumra/baxter-pnp/issues/4](https://github.com/skumra/baxter-pnp/issues/4), do 
        ```
        sudo apt-get install ros-noetic-effort-controllers
        ```
    - if error with **baxter_simulator**, change the baxter_simulator code accroding to [https://github.com/RethinkRobotics/baxter_simulator/pull/132](https://github.com/RethinkRobotics/baxter_simulator/pull/132). (For melodic, refer the changes to [https://github.com/RethinkRobotics/baxter_simulator/pull/130/files](https://github.com/RethinkRobotics/baxter_simulator/pull/130/files))

    - if error with **baxter_interface**, change the baxter_interface code accroding to [https://github.com/RethinkRobotics/baxter_interface/pull/87](
    https://github.com/RethinkRobotics/baxter_interface/pull/87)

    - if error with **libapr-1.so.0**, `/usr/bin/ld: /lib/x86_64-linux-gnu/libapr-1.so.0: undefined reference to uuid_generate@UUID_1.0`, refer to [](https://chowdera.com/2020/12/20201217093415078s.html)


### (Optional) try the [baxter-pnp repo](https://github.com/skumra/baxter-pnp). 

    git clone https://github.com/skumra/baxter-pnp.git
    
### Run simulation
1. Use the baxter.sh script for proper environment setup
    ```
    cd ~/ros_ws
    cp src/baxter/baxter.sh .
    # modify the baxter.sh, change the Line 30 to specify the ros version,
    ros_version = xxx (eg, neotic)
    ```