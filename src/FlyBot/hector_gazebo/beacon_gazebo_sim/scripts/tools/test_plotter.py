import time

import numpy as np
import matplotlib.pyplot as plt
import rospy
from beacon_gazebo_sim.msg import ReceiverIn, BeaconSimPose

a = np.array([])
d = np.array([])
hist = []

def rssi_to_distance(rssi, m_power: float, a: float, b: float, c: float):  # rssi, m_power -> distance
    return b * ((rssi / m_power) ** a) + c



def receiver_clb(msg: ReceiverIn):
    global hist, a, d
    if msg.id == "beacon__beacon_1":
        print(f"msg: {msg.id} {round(msg.rssi)}")
        hist = [msg.rssi] + hist[:1000]
        a = np.array(hist)
        # print("\t", a.std(), a.mean(), np.median(a))
        d = rssi_to_distance(a, -60, 10, 1, 0)
        # print("\t", d.std(), d.mean(), np.median(d), d[0])
        # plt.hist(a)
        # plt.pause(0.05)
        # plt.show()



receiver_sub = rospy.Subscriber(
    "/Robot1/receiver__Robot1/receiver_in_msgs", ReceiverIn, receiver_clb)

rospy.init_node('test____', anonymous=True)
rospy.spin()

# while True:
#     if (len(hist) >= 500):
#         plt.hist(a, bins=50)
#         plt.savefig("test_a.jpeg")
#         plt.clf()
#         plt.hist(d, bins=50)
#         plt.savefig("test_d.jpeg")
#         exit()
#     time.sleep(0.1)
