"""
Classes to handle Agent odometry (player and non-player)
"""

from geometry_msgs.msg import TransformStamped, Twist
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint

from carla.sensor import Transform as carla_Transform
from carla_ros_bridge.transforms import carla_transform_to_ros_transform, carla_transform_to_ros_pose, ros_transform_to_pose, carla_measurements_to_ros_twist

class AgentOdometryHandler(object):
    def __init__(self, name, process_msg_fun=None, world_link='map'):
        self.name = name
        self.world_link = world_link
        self.process_msg_fun = process_msg_fun
        self.lookup_table_marker_id = {}

    def process_msg(self, data, cur_time):
        """

        :param data: carla agent data
        :param cur_time: current ros simulation time
        :return:
        """
        raise NotImplemented


class PlayerOdometryHandler(AgentOdometryHandler):
    def __init__(self, name, **kwargs):
        super(PlayerOdometryHandler, self).__init__(name, **kwargs)

    def process_msg(self, data, cur_time):
        odo = Odometry()
        odo.header.stamp = cur_time
        odo.header.frame_id = self.world_link
        odo.child_frame_id = "base_link"
        odo.pose.pose = carla_transform_to_ros_pose(
                            data.transform)
        odo.twist.twist = carla_measurements_to_ros_twist(data.transform, data.forward_speed)
        self.process_msg_fun(self.name, odo)


class NonPlayerOdometryHandler(AgentOdometryHandler):
    def __init__(self, name, **kwargs):
        super(NonPlayerOdometryHandler, self).__init__(name, **kwargs)

    def process_msg(self, data, cur_time):
        vehicles_odometry = MultiDOFJointTrajectory()
        vehicles_odometry.header = Header(stamp=cur_time, frame_id=self.world_link)
        for agent in data:
            if agent.HasField('vehicle'):
                vehicles_odometry.joint_names.append(agent.id)
                transform = carla_transform_to_ros_transform(carla_Transform(agent.vehicle.transform))
                velocity = carla_measurements_to_ros_twist(agent.vehicle.transform, agent.vehicle.forward_speed)
                acceleration = Twist()
                point = MultiDOFJointTrajectoryPoint(transform, velocity, acceleration, 0)
                vehicles_odometry.points.append(point)

        self.process_msg_fun('vehicles_odometry', vehicles_odometry)
