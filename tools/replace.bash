#!/bin/sh

replace() {
    old=$1
    new=$2

    files=$(grep -rl "$old" . --exclude="*.pyc" --exclude="*.bash" \
            --exclude-dir="build" --exclude-dir=".git")

    for f in $files; do
        sed -i "s/${old}/${new}/g" $f
    done
}

replace "Actuator(.kuka_lwr.)" "KukaLWR()"
replace "Actuator(.ptu.)" "PTU()"
replace "Actuator(.v_omega.)" "MotionVW()"
replace "Actuator(.xy_omega.)" "MotionXYW()"
replace "Actuator(.destination.)" "Destination()"
replace "Actuator(.keyboard.)" "Keyboard()"
replace "Actuator(.rotorcraft_waypoint.)" "RotorcraftWaypoint()"
replace "Actuator(.rotorcraft_attitude.)" "RotorcraftAttitude()"
replace "Actuator(.gripper.)" "Gripper()"
replace "Actuator(.v_omega_diff_drive.)" "MotionVWDiff()"
replace "Actuator(.waypoint.)" "Waypoint()"

replace "Robot(.atrv.)" "ATRV()"
replace "Robot(.jido.)" "Jido()"
replace "Robot(.rmax.)" "RMax()"
replace "Robot(.quadrotor_dynamic.)" "Quadrotor()"
replace "Robot(.victim.)" "Victim()"
replace "WheeledRobot(.segwayrmp400.)" "SegwayRMP400()"
replace "WheeledRobot(.pioneer3dx.)" "Pioneer3DX()"

replace "Sensor(.depth_camera.)" "DepthCamera()"
replace "Sensor(.stereo_unit.)" "StereoUnit()"
replace "Sensor(.video_camera.)" "VideoCamera()"
replace "Sensor(.odometry.)" "Odometry()"
replace "Sensor(.pose.)" "Pose()"
replace "Sensor(.sick.)" "Sick()"
replace "Sensor(.imu.)" "IMU()"
replace "Sensor(.semantic_camera.)" "SemanticCamera()"
replace "Sensor(.camera.)" "VideoCamera()"
replace "Sensor(.kuka_lwr.)" "KukaLWR()"
replace "Sensor(.battery.)" "Battery()"
replace "Sensor(.gps.)" "GPS()"
replace "Sensor(.gyroscope.)" "Gyroscope()"
replace "Sensor(.proximity.)" "Proximity()"
replace "Sensor(.rosace.)" "SearchAndRescue()"
replace "Sensor(.thermometer.)" "Thermometer()"

################################################################################
