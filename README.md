# RC car interface

ROS 2 Jazzy driver for [RC1 Pico firmware](https://github.com/pbeuchat43/rc_car_firmware). The Pico decides actuator authority. This driver provides exclusive serial discovery, the typed ROS API and a direct bench transport for deployment procedures.

~~~sh
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rc_car_interface
source install/setup.bash
ros2 launch rc_car_interface vehicle.launch.py namespace:=car device_id:=YOUR_DEVICE_ID profile_id:=YOUR_PROFILE_ID build_id:=YOUR_BUILD_ID
~~~

Leave device_id unset only when exactly one compatible Pico is connected. Set profile_id and build_id from the deployment-selected manifest to enforce both pins. Use a namespaced node per car. The launch uses system time. Drive, pan and mode clients must use the same system time and include the session from fresh status.

| Private name | Type | Purpose |
| --- | --- | --- |
| ~/command | DriveCommand | Fresh normalized drive/steering pair for the current session and generation |
| ~/pan_command | PanCommand | Independent normalized pan target for the current session |
| ~/set_mode | SetMode | Request Manual, Autonomy or Hold and optionally select a source for each axis |
| ~/status | VehicleStatus | MCU status and output selection, with freshness and connection state |
| ~/diagnostics | DiagnosticArray | Low-rate link and frame diagnostics |

The vehicle interface never resumes motion after reconnect or an RC edge. To run autonomy, submit a stamped SetMode request using the current session and generation, wait for fresh AUTO status, then send freshly stamped commands with the new generation. Both axes default to HOST when requesting AUTO. RC/RC requests Manual. Pan PWM status is the programmed output. Status with fresh=false is historical or unobserved.

The installed lib/rc_car_interface/rc_car_bench_link executable is a direct USB transport for deployment maintenance. It requires the exact device ID and profile ID, with an optional build pin. It confirms HOLD, ENTER, PULSE <us>, EXIT and ABORT using fresh firmware status. Follow the selected deployment procedure and isolate propulsion before using it.

The canonical wire header and fixtures are maintained in rc_car_firmware. When changing the wire, synchronize include/rc_car/wire.hpp and tests/fixtures before running both suites.

~~~sh
cmake -S . -B build/native -DRC_CAR_STANDALONE=ON
cmake --build build/native
ctest --test-dir build/native --output-on-failure
~~~

The Linux suite uses a pseudo-terminal rather than a physical Pico.
