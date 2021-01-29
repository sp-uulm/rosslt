FROM ros:foxy

RUN apt-get update
RUN DEBIAN_FRONTEND=noninteractive apt-get install --yes --force-yes -qq qtwebengine5-dev libboost-dev
RUN DEBIAN_FRONTEND=noninteractive apt-get install --yes --force-yes -qq ros-foxy-rviz2 ros-foxy-rqt python3-colcon-common-extensions ros-foxy-tf2-geometry-msgs ros-foxy-interactive-markers

WORKDIR /colcon_ws

RUN git clone https://github.com/sp-uulm/rosslt.git /colcon_ws/src/rosslt/
RUN git clone -b ros2_rosslt --recurse-submodules https://github.com/sp-uulm/interactive_script.git /colcon_ws/src/interactive_script/

RUN bash -c "source /opt/ros/foxy/setup.bash; colcon build --packages-up-to rosslt_demo"
RUN bash -c "source /opt/ros/foxy/setup.bash; colcon build --packages-up-to interactive_script_blockly"
RUN bash -c "source /opt/ros/foxy/setup.bash; colcon build --packages-up-to interactive_script"

# run with docker run -it --env="DISPLAY" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --entrypoint=/bin/bash <CONTAINER>
# - source /opt/ros/foxy/setup.bash and install/setup.bash
# - run demos in rosslt_demo/launch
# - run script editor in rqt with topics/interactive_script plugin
