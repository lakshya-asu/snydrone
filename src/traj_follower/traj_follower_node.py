from typing import Optional, List
import threading

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path

# Try to import MAVSDK, but don't hard-fail if it's not installed.
try:
    from mavsdk import System
    from mavsdk.offboard import OffboardError, PositionNedYaw
    MAVSDK_AVAILABLE = True
except ImportError:
    MAVSDK_AVAILABLE = False


class TrajFollowerNode(Node):
    """Trajectory follower.

    - Always subscribes to 'trajectory' and logs the path.
    - If enable_mavsdk == true AND MAVSDK is installed, it starts a background
      MAVSDK task that:
        * connects to PX4
        * starts Offboard mode
        * streams position setpoints along the received Path.
    """

    def __init__(self) -> None:
        super().__init__("traj_follower")

        # Parameters
        self.declare_parameter("enable_mavsdk", False)
        self.declare_parameter("mavsdk_url", "udp://:14540")
        self.declare_parameter("offboard_rate_hz", 20.0)
        self.declare_parameter("default_altitude_ned", -3.0)

        self.enable_mavsdk = (
            self.get_parameter("enable_mavsdk").get_parameter_value().bool_value
        )
        self.mavsdk_url = (
            self.get_parameter("mavsdk_url").get_parameter_value().string_value
        )
        self.offboard_rate_hz = (
            self.get_parameter("offboard_rate_hz").get_parameter_value().double_value
        )
        self.default_alt_ned = (
            self.get_parameter("default_altitude_ned").get_parameter_value().double_value
        )

        # Latest path (shared with MAVSDK worker)
        self._path_lock = threading.Lock()
        self._current_path: Optional[Path] = None

        # Subscribe to trajectory
        self.path_sub = self.create_subscription(
            Path, "trajectory", self.path_callback, 10
        )

        self.get_logger().info("Trajectory follower node started.")
        if self.enable_mavsdk:
            if not MAVSDK_AVAILABLE:
                self.get_logger().error(
                    "enable_mavsdk is true but mavsdk python package is not installed."
                )
            else:
                self.get_logger().info(
                    f"MAVSDK enabled, connecting to PX4 at {self.mavsdk_url}"
                )
                # Start MAVSDK worker in a background thread
                self._stop_event = threading.Event()
                self._worker_thread = threading.Thread(
                    target=self._mavsdk_worker, daemon=True
                )
                self._worker_thread.start()
        else:
            self.get_logger().info("MAVSDK disabled; will only log trajectories.")

    # ---------------- ROS Callbacks ----------------

    def path_callback(self, msg: Path) -> None:
        with self._path_lock:
            self._current_path = msg
        self.get_logger().info(
            f"Received trajectory with {len(msg.poses)} poses."
            + (" (MAVSDK disabled)" if not (self.enable_mavsdk and MAVSDK_AVAILABLE) else "")
        )

    # ---------------- MAVSDK Worker ----------------

    def _mavsdk_worker(self) -> None:
        """Run MAVSDK Offboard control in its own asyncio loop."""
        import asyncio

        async def run():
            drone = System()
            await drone.connect(system_address=self.mavsdk_url)

            self.get_logger().info("MAVSDK: Waiting for PX4 connection...")
            async for state in drone.core.connection_state():
                if state.is_connected:
                    self.get_logger().info("MAVSDK: Connected to PX4.")
                    break

            self.get_logger().info("MAVSDK: Waiting for local position...")
            async for health in drone.telemetry.health():
                if health.is_local_position_ok:
                    self.get_logger().info("MAVSDK: Local position OK.")
                    break

            # Arm
            self.get_logger().info("MAVSDK: Arming...")
            await drone.action.arm()

            # Prime Offboard with an initial setpoint
            self.get_logger().info("MAVSDK: Setting initial setpoint...")
            await drone.offboard.set_position_ned(
                PositionNedYaw(0.0, 0.0, self.default_alt_ned, 0.0)
            )

            try:
                await drone.offboard.start()
                self.get_logger().info("MAVSDK: Offboard mode started.")
            except OffboardError as e:
                self.get_logger().error(
                    f"MAVSDK: Offboard start failed: {e._result.result}"
                )
                await drone.action.disarm()
                return

            # Main loop: follow the latest path if available.
            rate = self.offboard_rate_hz if self.offboard_rate_hz > 0.0 else 20.0
            dt = 1.0 / rate

            while not self._stop_event.is_set():
                # Snapshot the current path under lock
                with self._path_lock:
                    path_copy = self._current_path

                if path_copy is None or len(path_copy.poses) == 0:
                    # No path yet; just keep hovering
                    await asyncio.sleep(0.2)
                    continue

                self.get_logger().info(
                    f"MAVSDK: Following path with {len(path_copy.poses)} poses."
                )

                # Iterate through poses and send setpoints
                for pose_stamped in path_copy.poses:
                    if self._stop_event.is_set():
                        break

                    # Very simple ENU->NED mapping:
                    # - assume path.header.frame_id ~ local ENU, x=north, y=east
                    north = pose_stamped.pose.position.x
                    east = pose_stamped.pose.position.y

                    # Use path z if non-zero; otherwise default altitude.
                    z = pose_stamped.pose.position.z
                    down = -z if z != 0.0 else self.default_alt_ned

                    yaw_deg = 0.0  # TODO: derive from orientation if desired

                    try:
                        await drone.offboard.set_position_ned(
                            PositionNedYaw(north, east, down, yaw_deg)
                        )
                    except OffboardError as e:
                        self.get_logger().error(
                            f"MAVSDK: Failed to set position setpoint: {e._result.result}"
                        )
                        break

                    await asyncio.sleep(dt)

                # After finishing the path, we could hover at last point.
                await asyncio.sleep(0.1)

            # Stop offboard and land on shutdown
            self.get_logger().info("MAVSDK: Stopping offboard and landing...")
            try:
                await drone.offboard.stop()
            except OffboardError as e:
                self.get_logger().error(
                    f"MAVSDK: Offboard stop failed: {e._result.result}"
                )
            await drone.action.land()
            self.get_logger().info("MAVSDK: Land command sent, worker exiting.")

        # Run the async function in this thread
        import asyncio

        try:
            asyncio.run(run())
        except Exception as e:
            self.get_logger().error(f"MAVSDK worker crashed: {e}")

    # ---------------- Lifecycle ----------------

    def destroy_node(self) -> bool:
        # Signal the MAVSDK worker to stop
        if hasattr(self, "_stop_event"):
            self._stop_event.set()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
