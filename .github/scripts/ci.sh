#!/usr/bin/env bash
# Run from the GitHub Actions workspace; dependencies are checked out by ci.yml.
set -eo pipefail
source /opt/ros/lyrical/setup.bash

for package in rpp_common rpp_plugin_registrator rpp_orchestrator rpp_py rpp_cli rpp_testing; do
  python -m pip install --no-deps -e "rpp_dev/src/$package"
done


# Resolve the ROS dependencies declared by this library and the C++ runtime.
apt-get update
apt-get install --no-install-recommends -y python3-rosdep
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  rosdep init
fi
rosdep update --rosdistro lyrical
rosdep install --from-paths libraries/rpp_control rpp_dev/src/rpp_cpp libraries/marble_control_msgs --ignore-src --rosdistro lyrical -r -y

colcon build --base-paths rpp_dev/src/rpp_cpp libraries/marble_control_msgs rpp_dev/src/rpp_ros2 \
  --symlink-install --cmake-args -DBUILD_TESTING=OFF \
  -DPython3_EXECUTABLE=/opt/rpp-test-venv/bin/python
source install/setup.bash
rpp registry config set RPP_CPP_CORE_PATH "$PWD/build/rpp_cpp"
rpp init-home

# Generate interfaces before building the ROS adapter. Register implementations
# after its headers and shared libraries have been installed.
rpp library create rpp_control
rpp library rpp_control register "$PWD/libraries/rpp_control/plugin_types/controllers.capnp" --type
colcon build --base-paths libraries/rpp_control --symlink-install --cmake-args \
  -DBUILD_TESTING=ON -DPython3_EXECUTABLE=/opt/rpp-test-venv/bin/python
source install/setup.bash
rpp library unregister rpp_control
rpp library register "$PWD/libraries/rpp_control" --link

colcon test --packages-select rpp_control --event-handlers console_direct+ \
  --return-code-on-test-failure
colcon test-result --verbose
