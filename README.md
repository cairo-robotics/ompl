The Open Motion Planning Library (OMPL)
=======================================

Linux / OS X [![Build Status](https://travis-ci.org/ompl/ompl.svg?branch=master)](https://travis-ci.org/ompl/ompl)
Windows [![Build status](https://ci.appveyor.com/api/projects/status/valuv9sabye1y35n/branch/master?svg=true)](https://ci.appveyor.com/project/mamoll/ompl/branch/master)

Visit the [OMPL installation page](http://ompl.kavrakilab.org/core/installation.html) for
detailed installation instructions.

OMPL has the following required dependencies:

* [Boost](http://www.boost.org) (version 1.54 or higher)
* [CMake](http://www.cmake.org) (version 2.8.7 or higher)

The following dependencies are optional:

* [ODE](http://ode.org) (needed to compile support for planning using the Open Dynamics Engine)
* [Py++](https://bitbucket.org/ompl/ompl/src/tip/doc/markdown/installPyPlusPlus.md) (needed to generate Python bindings)
* [Doxygen](http://www.doxygen.org) (needed to create a local copy of the documentation at
  http://ompl.kavrakilab.org/core)
* [Eigen](http://eigen.tuxfamily.org) (needed for an informed sampling technique to improve the optimization of path length and for the Vector Field RRT planner)

Once dependencies are installed, you can build OMPL on Linux, OS X,
and MS Windows. Go to the top-level directory of OMPL and type the
following commands:

    mkdir -p build/Release
    cd build/Release
    cmake ../..
    # next step is optional
    make -j 4 update_bindings # if you want Python bindings
    make -j 4 # replace "4" with the number of cores on your machine
    
    
##  Notes for CAIRO:

In order to set this up with a custom cost function first you'll need to setup an OMPL development environment in either Ubuntu 16.04 Desktop or Docker using this tutorial: https://jack-kawell.com/2019/06/24/installing-ompl/

After that you'll want to clone [this repo](https://github.com/cairo-robotics/custom_planning.git) into the `ws_moveit/src` directory:

```
cd ws_moveit/src
git clone https://github.com/cairo-robotics/custom_planning.git
```

You'll also need to checkout the branch of the CAIRO fork of OMPL that contains the code for custom costs through ROS.

```
cd ompl
git checkout custom-cost
```

Once that's done you should be able to rebuild and re-source the workspace:

```
sudo catkin build
source ./devel/setup.bash
```

### Testing setup

An easy way of testing the environment is with the Panda Rviz robot demo provided by MoveIt!. This demo is spelled out more fully [here](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/quickstart_in_rviz/quickstart_in_rviz_tutorial.html), but we'll need to make a slight modification to the OMPL config to test out our setup.

All we need to do is add a line to the `ompl_planning.yaml` file within the `panda_moveit_config` ROS package. To do this run the following commands:

```
roscd panda_moveit_config
sudo nano config/ompl_planning.yaml
```

And then add `optimization_objective: CustomObjective` inside the heading "RRTstarkConfigDefault" right under the "type" key. The final section should look like this:

```
  RRTstarkConfigDefault:
    type: geometric::RRTstar
    optimization_objective: CustomObjective
    range: 0.0  # Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on setup()
    goal_bias: 0.05  # When close to goal select goal, with this probability? default: 0.05
    delay_collision_checking: 1  # Stop collision checking as soon as C-free parent found. default
```

By making this change you're requesting OMPL to use our CustomObjective that will call out to a ROS service that is ready to be called as `/custom_cost` in order to find the cost for each state in its motion planning. In order for this to work though, you'll need that service to be running. To do that you can simply `rosrun` any of the custom cost services within the `custom_planning` package you cloned into the workspace earlier.

In other words, once you've used `roslaunch` to begin the Panda demo (`roslaunch panda_moveit_config demo.launch rviz_tutorial:=true`), you can simply run the below command to start a ROS cost service that will give out random costs when OMPL requests them:

```
rosrun custom_planning random_cost.py
```

Once that's done you should be able to follow the tutorial [here](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/quickstart_in_rviz/quickstart_in_rviz_tutorial.html) to test out that your custom installation of MoveIt! and OMPL are working with your new custom objective.

You'll notice that there are at least two custom cost ROS services for you to test within the `custom_planning` package. Note that you can run one, kill it, and then run another without stopping the MoveIt!/OMPL/Rviz Panda demo as long as no planning is currently being run. This shows the awesome modularity of this custom cost function solution.
