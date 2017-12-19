# Installing opentracing instrumentation
We can add opentracing instrumentation to a ROS project by overlaying a fork of ros_comm.
In the future, the same should be achievable (in Python, at least) by monkey-patching.

## Clone the Figure fork of ros_comm
```bash
cd src/
git clone git@github.com:FigureWorks/ros_comm.git
git checkout opentracing
cd ..
```

## Configure rosdep to use a local rosdep.yaml file
```bash
sudo echo "yaml file:///home/${USER}/workspaces/rosdep.yaml" > /etc/ros/rosdep/sources.list.d/10-local.list
```

## Download rosdep.yaml from Figure
Download https://raw.githubusercontent.com/FigureWorks/elliot-ros/master/workspaces/rosdep.yaml into `~/workspaces`

## Install rosdeps
```bash
rosdep install --from-paths src --ignore-src -r -y
```

## Build
```bash
catkin_make
source devel/setup.bash
```

## Confirm you are successfully overlaying rospy
```bash
rospack find rospy
# should return a directory in ~/workspaces
```

## Run the Jaeger collector, server, and UI
```bash
docker run -d -e COLLECTOR_ZIPKIN_HTTP_PORT=9411 -p5775:5775/udp -p6831:6831/udp -p6832:6832/udp \
  -p5778:5778 -p16686:16686 -p14268:14268 -p9411:9411 jaegertracing/all-in-one:latest
```

## Launch your application
```bash
roslaunch my_application main
```

## Instrument local function calls
Add the `@traced_function` decorator to any local functions you'd like to trace
```python
from opentracing_instrumentation import traced_function

@traced_function
def fn():
    return 0
```

## Visit Jaeger UI
http://localhost:16686/