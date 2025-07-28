import math
import random
import subprocess
import time
import carla

from simulation.sensors import (
    GnssSensor,
    IMUSensor,
    LidarSensor,
    RgbCamera,
)
import threading
from transforms3d.euler import euler2mat, quat2euler, euler2quat

CONTAINER_NAME = "autoware"
BRIDGE_CONTAINER_NAME = "autoware_bridge"
ENV_SCRIPT_PATH = "env.sh"
ROS_SCRIPT_PATH = "/ros_entrypoint.sh"


# source {ROS_SCRIPT_PATH} 2>/dev/null &&
def _stream_output(stream):
    for line in iter(stream.readline, b""):
        print(f"  [background] > {line.decode('utf-8').strip()}")
    stream.close()


def _execute_command(command: str, container_name: str, background_run: bool):
    full_command_inside_container = f"cd autoware_carla_launch && source {ENV_SCRIPT_PATH} && source ./install/setup.bash && source /ros_entrypoint.sh && {command}"

    command_to_run_on_host = [
        "docker",
        "exec",
        container_name,
        "bash",
        "-ic",
        full_command_inside_container,
    ]

    print(f"Running CMD: {command_to_run_on_host}")

    if background_run:
        try:
            process = subprocess.Popen(
                command_to_run_on_host,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
            )
            thread = threading.Thread(target=_stream_output, args=(process.stdout,))
            thread.daemon = True
            thread.start()
            print(f"✅ Process started in background with PID: {process.pid}")
            return process
        except FileNotFoundError:
            print("❌ Error: 'docker' command not found.")
            return False
        except Exception as e:
            print(f"❌ Error starting background process: {e}")
            return False
    else:
        try:
            result = subprocess.run(
                command_to_run_on_host, check=True, capture_output=True, text=True
            )
            print("✅ Command executed successfully!")
            print("--- Container Output ---")
            print(result.stdout.strip())
            if result.stderr:
                print("--- Container Stderr ---")
                print(result.stderr.strip())
            return True
        except subprocess.CalledProcessError as e:
            print(f"❌ Error executing command: {e}")
            print("--- Error Details (stdout) ---")
            print(e.stdout)
            print("--- Error Details (stderr) ---")
            print(e.stderr)
            return False
        except FileNotFoundError:
            print("❌ Error: 'docker' command not found.")
            return False


def carla_transform_2_pose(tf):
    p_x, p_y, p_z = None, None, None
    ox, oy, oz, ow = None, None, None, None
    #####
    p_x, p_y, p_z = tf.location.x, -tf.location.y, tf.location.z
    carla_rotation = tf.rotation
    roll = math.radians(carla_rotation.roll)
    pitch = -math.radians(carla_rotation.pitch)
    yaw = -math.radians(carla_rotation.yaw)
    quat = euler2quat(roll, pitch, yaw)
    ow, ox, oy, oz = quat[0], quat[1], quat[2], quat[3]
    return p_x, p_y, p_z, ox, oy, oz, ow


def send_goal_pose(dp):
    print("\n----- Sending Goal Pose -----")
    print(f"Transform gp {dp}")
    p_x, p_y, p_z, ox, oy, oz, ow = carla_transform_2_pose(tf=dp)
    message_payload = f"""
{{
    header: {{
        stamp: {{sec: 0, nanosec: 0}},
        frame_id: 'map'
    }},
    pose: {{
        position: {{x: {p_x}, y: {p_y}, z: {p_z}}},
        orientation: {{x: {ox}, y: {oy}, z: {oz}, w: {ow}}}
    }}
}}
"""

    ros_command = (
        f"ros2 topic pub --once "
        f"/planning/mission_planning/goal "
        f"geometry_msgs/msg/PoseStamped "
        f"'{message_payload.strip()}'"
    )

    _execute_command(ros_command, CONTAINER_NAME, False)


def change_to_autonomous_mode():
    print("\n----- Changing to Autonomous Mode -----")
    ros_command = (
        f"ros2 service call "
        f"/api/operation_mode/change_to_autonomous "
        f"autoware_adapi_v1_msgs/srv/ChangeOperationMode {{}}"
    )

    _execute_command(ros_command, CONTAINER_NAME, False)


def create_autoware_vehicle():
    client = carla.Client("127.0.0.1", 2000)
    world = client.load_world("Town01")
    settings = world.get_settings()
    print(f"Current settings: {settings}")
    blueprint = world.get_blueprint_library().filter("vehicle.tesla.model3")[0]
    blueprint.set_attribute("role_name", "autoware_v1")
    if blueprint.has_attribute("color"):
        color = random.choice(blueprint.get_attribute("color").recommended_values)
        blueprint.set_attribute("color", color)
    if blueprint.has_attribute("driver_id"):
        driver_id = random.choice(
            blueprint.get_attribute("driver_id").recommended_values
        )
        blueprint.set_attribute("driver_id", driver_id)
    if blueprint.has_attribute("is_invincible"):
        blueprint.set_attribute("is_invincible", "true")
    print(blueprint)
    sps = world.get_map().get_spawn_points()
    sp = random.choice(sps)
    dp = random.choice(sps)
    dp = sps[0]
    actor = world.try_spawn_actor(
        blueprint,
        sp,
    )
    try:
        physics_control = actor.get_physics_control()
        physics_control.use_sweep_wheel_collision = True
        actor.apply_physics_control(physics_control)
    except Exception as e:
        print(e)

    _gnss_sensor = GnssSensor(actor, sensor_name="ublox")
    _imu_sensor = IMUSensor(actor, sensor_name="tamagawa")
    _lidar_sensor = LidarSensor(actor, sensor_name="top")
    _rgb_camera = RgbCamera(actor, sensor_name="traffic_light")
    world.wait_for_tick()

    for _ in range(20):
        blueprint = world.get_blueprint_library().filter("vehicle.tesla.model3")[0]
        npc = world.try_spawn_actor(blueprint, random.choice(sps))
        if npc:
            npc.set_autopilot()
            print(f"Set {npc}")

    time.sleep(5)  # wait for carla ready
    print("Create Vehicle")
    start_bridge()  # start bridge in backgroun
    time.sleep(10)  # wait for bridge load
    start_autoware()  # open autoware
    time.sleep(60)
    send_goal_pose(dp)  # send goal point
    time.sleep(5)  # wait traj calculate
    change_to_autonomous_mode()
    while True:
        time.sleep(1000)


def start_bridge():
    start_bridge_cmd = "pkill -f zenoh_carla_bridge && sleep 1 && ./script/bridge_ros2dds/run-bridge.sh"
    _execute_command(start_bridge_cmd, BRIDGE_CONTAINER_NAME, True)


def start_autoware():
    start_autoware_cmd = "pkill -f ros2 && pkill -f zenoh-bridge-ros2dds && sleep 1 && ./script/autoware_ros2dds/run-autoware.sh"
    _execute_command(start_autoware_cmd, CONTAINER_NAME, True)


if __name__ == "__main__":
    create_autoware_vehicle()
