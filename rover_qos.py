"""QoS profile for /cmd_vel — must match micro-ROS ESP subscription."""
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

# Match micro-ROS — RELIABLE proved delivery; best_effort --once often drops
CMD_VEL_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)
