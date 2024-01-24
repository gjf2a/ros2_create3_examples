import sys, subprocess

if len(sys.argv) < 2:
    print("Usage: reset_pose robot_name")
else:
    bot = sys.argv[1]
    call = f'ros2 service call /{bot}/reset_pose irobot_create_msgs/srv/ResetPose '
    call += '"{pose:{position:{x: 0.0, y: 0.0, z: 0.0}, orientation:{x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"'
    #result = subprocess.run(call, shell=True, capture_output=True)
    result = subprocess.call(call, shell=True)
    print(f"Call complete with result {result}")
