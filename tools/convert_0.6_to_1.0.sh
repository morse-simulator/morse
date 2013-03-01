#! /bin/sh

sed \
    -e s/Actuator\(.kuka_lwr.\)/KukaLWR\(\)/g                         \
    -e s/Actuator\(.ptu.\)/PTU\(\)/g                                  \
    -e s/Actuator\(.v_omega.\)/MotionVW\(\)/g                         \
    -e s/Actuator\(.xy_omega.\)/MotionXYW\(\)/g                       \
    -e s/Actuator\(.destination.\)/Destination\(\)/g                  \
    -e s/Actuator\(.orientation.\)/Orientation\(\)/g                  \
    -e s/Actuator\(.keyboard.\)/Keyboard\(\)/g                        \
    -e s/Actuator\(.rotorcraft_waypoint.\)/RotorcraftWaypoint\(\)/g   \
    -e s/Actuator\(.rotorcraft_attitude.\)/RotorcraftAttitude\(\)/g   \
    -e s/Actuator\(.gripper.\)/Gripper\(\)/g                          \
    -e s/Actuator\(.v_omega_diff_drive.\)/MotionVWDiff\(\)/g          \
    -e s/Actuator\(.waypoint.\)/Waypoint\(\)/g                        \
                                                                      \
    -e s/Robot\(.atrv.\)/ATRV\(\)/g                                   \
    -e s/Robot\(.jido.\)/Jido\(\)/g                                   \
    -e s/Robot\(.rmax.\)/RMax\(\)/g                                   \
    -e s/Robot\(.quadrotor_dynamic.\)/Quadrotor\(\)/g                 \
    -e s/Robot\(.victim.\)/Victim\(\)/g                               \
    -e s/WheeledRobot\(.segwayrmp400.\)/SegwayRMP400\(\)/g            \
    -e s/WheeledRobot\(.pioneer3dx.\)/Pioneer3DX\(\)/g                \
                                                                      \
    -e s/Sensor\(.depth_camera.\)/DepthCamera\(\)/g                   \
    -e s/Sensor\(.stereo_unit.\)/StereoUnit\(\)/g                     \
    -e s/Sensor\(.video_camera.\)/VideoCamera\(\)/g                   \
    -e s/Sensor\(.odometry.\)/Odometry\(\)/g                          \
    -e s/Sensor\(.pose.\)/Pose\(\)/g                                  \
    -e s/Sensor\(.sick.\)/Sick\(\)/g                                  \
    -e s/Sensor\(.sick-ld-mrs.\)/SickLDMRS\(\)/g                      \
    -e s/Sensor\(.imu.\)/IMU\(\)/g                                    \
    -e s/Sensor\(.semantic_camera.\)/SemanticCamera\(\)/g             \
    -e s/Sensor\(.camera.\)/VideoCamera\(\)/g                         \
    -e s/Sensor\(.kuka_lwr.\)/KukaLWR\(\)/g                           \
    -e s/Sensor\(.battery.\)/Battery\(\)/g                            \
    -e s/Sensor\(.gps.\)/GPS\(\)/g                                    \
    -e s/Sensor\(.gyroscope.\)/Gyroscope\(\)/g                        \
    -e s/Sensor\(.proximity.\)/Proximity\(\)/g                        \
    -e s/Sensor\(.rosace.\)/SearchAndRescue\(\)/g                     \
    -e s/Sensor\(.thermometer.\)/Thermometer\(\)/g                    \
                                                                      \
    -e s/configure_mw/add_stream/g                                    \
    -e s/configure_service/add_service/g                              \
    -e s/configure_overlay/add_overlay/g                              \
    -e s/configure_modifier/alter/g                                   \
    ${1}

