import ros
import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np
from beacon_gazebo_sim.msg import ReceiverIn, BeaconSimPose
from typing import *
# from dataclasses import dataclass
# from collections import namedtuple


def _rssi_to_distance(rssi: float, m_power: float, a: float = 10, b: float = 1, c: float = 0) -> float:  # rssi, m_power -> distance
    return b * ((rssi / m_power) ** a) + c


def _solve_position_2d(b_positions: np.ndarray, b_distances: np.ndarray) -> np.ndarray:
    """
    simple 2d pose estimation
    :param b_positions: np.ndarray - 2d pose
    :param b_distances: np.ndarray - rssi vals
    :return: np.ndarray - 2d pose
    """
    b_p = b_positions[1:, :]
    b_p_1 = b_positions[0]
    # print(b_p_1, b_p)
    A = (b_p_1 - b_p)
    # print("A", A)
    # print("b_p**2", np.expand_dims(np.sum(-b_p**2, axis=1), axis=1), np.sum(b_p_1**2), b_distances[0]**2, b_distances[1:]**2, np.expand_dims(b_distances[1:]**2, axis=1))
    b = np.expand_dims(np.sum(-b_p**2, axis=1), axis=1) + np.sum(b_p_1**2) - b_distances[0]**2 + np.expand_dims(b_distances[1:]**2, axis=1)
    b = b*0.5
    # print("b", b)
    # print("------", A.T @ A)
    m = np.linalg.inv(A.T @ A) @ (A.T @ b)
    # print(m)
    return np.reshape(m, (2))


class TestEst:
    class RSSIMsg(NamedTuple):
        time_stamp: float = 0
        rssi: float = 0
        m_rssi: float = 0

    class BeaconPose(NamedTuple):
        time_stamp: float = 0
        pose: np.ndarray = np.ones((3))

    def __init__(self, receiver_topic: str, beacons_poses_topic: str, self_beacon: str, pose_pub_topic: str, update_rate: float = 20):
        self.beacons_pose: Dict[str, TestEst.BeaconPose]  = dict()
        self.beacons_rssi: Dict[str, TestEst.RSSIMsg] = dict()
        self.self_beacon = self_beacon
        self.receiver_topic = receiver_topic
        self.beacons_poses_topic = beacons_poses_topic
        self.update_rate = update_rate
        self.pose_pub_topic = pose_pub_topic

        self._pose_pub = rospy.Publisher(self.pose_pub_topic, PoseStamped)
        self._receiver_sub = rospy.Subscriber(self.receiver_topic, ReceiverIn, self._receiver_clb)
        self._beacon_poses_sub = rospy.Subscriber(self.beacons_poses_topic, BeaconSimPose, self._beacon_poses_clb)

    def _receiver_clb(self, msg: ReceiverIn):
        if msg.id != self.self_beacon:
            r = msg.rssi
            self.beacons_rssi[msg.id] = TestEst.RSSIMsg(time_stamp=msg.time_stamp, rssi=r, m_rssi=msg.m_rssi)

    def _beacon_poses_clb(self, msg):
        if msg.id != self.self_beacon:
            self.beacons_pose[msg.id] = TestEst.BeaconPose(msg.time_stamp,
                                                           np.array([msg.position.x, msg.position.y, msg.position.z]))

    def predict_pose_stamped(self, time) -> PoseStamped:
        # 2d pose est
        beacons_ids = list(set(self.beacons_pose.keys()).intersection(set(self.beacons_rssi.keys())))
        print(beacons_ids)
        if len(beacons_ids) >= 3:
            b_positions = np.array([el.pose
                                    for k, el in map(lambda k: (k, self.beacons_pose[k]), beacons_ids)])
            b_distances = np.array([_rssi_to_distance(el.rssi, el.m_rssi)
                                    for k, el in map(lambda k: (k, self.beacons_rssi[k]), beacons_ids)])
            time_stamp = min([el.time_stamp.to_sec() for k, el in map(lambda k: (k, self.beacons_rssi[k]), beacons_ids)])
            print("b_distances: ", b_distances)
            xy = _solve_position_2d(b_positions[:, :2], b_distances)
            msg = PoseStamped()
            msg.pose.position.x = xy[0]
            msg.pose.position.y = xy[1]
            msg.pose.position.z = b_positions[:, 2].mean()
            msg.header.stamp = time
            msg.header.frame_id = "world"

            return msg
        else:
            return None

    def spin(self):
        r = rospy.Rate(self.update_rate)
        while not rospy.is_shutdown():
            msg = self.predict_pose_stamped(rospy.Time.now())
            if not msg is None:
                self._pose_pub.publish(msg)
            r.sleep()

rospy.init_node("test_pose_est")

te = TestEst(receiver_topic="/Robot1/receiver__Robot1/receiver_in_msgs",
             beacons_poses_topic="/beacon_gazebo_sim/beacons",
             self_beacon="beacon__Robot1",
             pose_pub_topic="/Robot1/test_pose")
te.spin()


