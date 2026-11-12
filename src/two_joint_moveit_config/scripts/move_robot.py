#!/usr/bin/env python3

"""
Move the two-joint robot to an XYZ position using pymoveit2.

This script sends a position-only goal to MoveIt 2. MoveIt plans a
collision-aware joint trajectory; the trajectory is then executed through
the configured arm_controller.

Coordinates are in metres, relative to base_link.
"""

import math
import sys
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from pymoveit2 import MoveIt2


# These values match the current two_joint_robot.urdf.xacro.
BASE_TO_SHOULDER_Z = 0.16 + 0.40
UPWARD_MOUNT_ANGLE = math.radians(15.0)
UPPER_ARM_LENGTH = 0.40
JOINT2_MIN = -1.57
JOINT2_MAX = 1.57

JOINT_NAMES = ["joint1", "joint2"]
BASE_LINK = "base_link"
END_EFFECTOR_LINK = "tool0"
PLANNING_GROUP = "arm"


def read_coordinate(label: str) -> float:
    """Read one finite coordinate from the terminal."""
    while True:
        try:
            value = float(input(f"{label} (metres): ").strip())
            if math.isfinite(value):
                return value
            print("Enter a finite number.")
        except ValueError:
            print("Invalid number. Example: 0.3864")


def check_two_joint_reachability(x: float, y: float, z: float):
    """
    Check the ideal geometry of this 2-DOF robot.

    Joint 1 rotates around Z. Joint 2 pitches the 0.40 m arm.
    The shoulder is at z=0.56 m. Because there is no telescopic/prismatic
    joint, the tool point must be 0.40 m from the shoulder.
    """
    horizontal_radius = math.hypot(x, y)
    dz = z - BASE_TO_SHOULDER_Z
    shoulder_to_tool_distance = math.sqrt(
        horizontal_radius**2 + dz**2
    )

    distance_error = abs(shoulder_to_tool_distance - UPPER_ARM_LENGTH)
    if distance_error > 0.005:
        raise ValueError(
            "\nTarget is outside the robot's reachable surface.\n"
            f"Distance from shoulder: {shoulder_to_tool_distance:.4f} m\n"
            f"Required distance:      {UPPER_ARM_LENGTH:.4f} m\n"
            "For this model, the target must lie approximately on a "
            "0.40 m radius sphere centred at (0, 0, 0.56) in base_link."
        )

    # Approximate joint values for this robot's geometry.
    joint1 = math.atan2(y, x) if horizontal_radius > 1e-9 else 0.0
    elevation = math.atan2(dz, horizontal_radius)
    # The joint frame is mounted 15 degrees upward (negative Y pitch).
    # Positive joint2 rotation lowers the arm, hence q2 = 15deg - elevation.
    joint2 = UPWARD_MOUNT_ANGLE - elevation

    if not (JOINT2_MIN <= joint2 <= JOINT2_MAX):
        raise ValueError(
            f"Target requires joint2={math.degrees(joint2):.1f} degrees, "
            "outside the configured [-90, +90] degree limits."
        )

    return joint1, joint2, shoulder_to_tool_distance


def main():
    print("\n=== Two-joint robot: XYZ motion using MoveIt 2 ===")
    print("Reference frame: base_link")
    print("Units: metres\n")

    try:
        x = read_coordinate("X")
        y = read_coordinate("Y")
        z = read_coordinate("Z")
        joint1_est, joint2_est, distance = check_two_joint_reachability(
            x, y, z
        )
    except (ValueError, KeyboardInterrupt) as exc:
        print(f"\n{exc}")
        return 1

    print("\nTarget:")
    print(f"  X = {x:.4f} m")
    print(f"  Y = {y:.4f} m")
    print(f"  Z = {z:.4f} m")
    print(f"  Shoulder-to-tool distance = {distance:.4f} m")
    print("\nGeometric joint estimate (for reference only):")
    print(f"  joint1 = {math.degrees(joint1_est):.2f} deg")
    print(f"  joint2 = {math.degrees(joint2_est):.2f} deg")

    answer = input("\nPlan this target? [y/N]: ").strip().lower()
    if answer not in ("y", "yes"):
        print("Cancelled.")
        return 0

    rclpy.init()
    node = Node(
        "xyz_moveit_client",
        parameter_overrides=[Parameter("use_sim_time", value=True)],
    )

    moveit = None
    executor = MultiThreadedExecutor(num_threads=2)
    spin_thread = None

    try:
        moveit = MoveIt2(
            node=node,
            joint_names=JOINT_NAMES,
            base_link_name=BASE_LINK,
            end_effector_name=END_EFFECTOR_LINK,
            group_name=PLANNING_GROUP,
        )

        # pymoveit2 does not spin the node internally. The executor must
        # process joint-state, planning-service and action callbacks.
        executor.add_node(node)
        spin_thread = threading.Thread(
            target=executor.spin, daemon=True
        )
        spin_thread.start()

        node.get_logger().info(
            f"Planning XYZ target ({x:.3f}, {y:.3f}, {z:.3f}) m"
        )


        # Convert the XYZ target into joint-space targets.
        # The geometric calculation above provides the joint estimates.
        node.get_logger().info(
            f"Planning joint target: "
            f"joint1={math.degrees(joint1_est):.2f} deg, "
            f"joint2={math.degrees(joint2_est):.2f} deg"
        )

        trajectory = moveit.plan(
            joint_positions=[joint1_est, joint2_est],
            joint_names=JOINT_NAMES,
            timeout_sec=30.0,
        )


        if trajectory is None:
            print(
                "\nMoveIt could not find a valid trajectory. "
                "Check the target, joint states, IK configuration, "
                "planning scene and controller setup."
            )
            return 2

        print("\nMoveIt found a trajectory.")
        answer = input("Execute the trajectory? [y/N]: ").strip().lower()
        if answer not in ("y", "yes"):
            print("Trajectory was planned but not executed.")
            return 0

        if not moveit.execute(trajectory):
            print("\nMoveIt rejected the trajectory for execution.")
            return 3

        print("Trajectory sent. Waiting for execution result...")
        success = moveit.wait_until_executed(
            timeout_sec=60.0,
            cancel_on_timeout=True,
        )

        if success:
            print("Motion completed successfully.")
            return 0

        print("Motion did not complete successfully.")
        failure = moveit.last_failure()
        if failure:
            print(f"Details: {failure}")
        return 4

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
        if moveit is not None:
            moveit.cancel_execution()
        return 130
    except Exception as exc:
        print(f"\nError: {exc}")
        return 5
    finally:
        if moveit is not None:
            moveit.destroy()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if spin_thread is not None:
            spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    sys.exit(main())
