# pose_utils.py
from geometry_msgs.msg import Pose

def create_pose(x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose
