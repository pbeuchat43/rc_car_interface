# Repository guidance

The interface owns exclusive Linux serial I/O, ROS 2 Jazzy messages, freshness checks and launch. The Pico decides authority. Keep the vendored codec and fixtures synchronized with rc_car_firmware. Keep ESC model programming in robot_deployment.

Develop on a feature branch from dev. Keep main for agreed baselines. Run the Linux PTY tests and the Jazzy fake-Pico tests for relevant changes. Tests must not open a real serial device or publish into an operating robot's ROS graph.
