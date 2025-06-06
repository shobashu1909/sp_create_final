# Comment:
# Handles Optitrack motion tracking with ROS

import sys

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped


def skew(vec):
    ''' Compute the skew-symmetric matrix of a vector
    Args:
        vec: 3x1 numpy array
    Returns:
        matrix_sk: 3x3 numpy array
    '''
    vec = vec.reshape([3])
    x = vec[0]
    y = vec[1]
    z = vec[2]

    # Compute the rotation matrix elements
    matrix_sk = np.array([
        [0, -z, y],
        [z, 0, -x],
        [-y, x, 0]
    ])
    return matrix_sk


def qua2ori(q):
    ''' Convert quaternion to rotation matrix
    Args:
        q: 4x1 numpy array
    Returns:
        R: 3x3 numpy array
    '''
    qw = q[0]
    qv = q[1:4].reshape([3, 1])

    R = (qw * qw - qv.T @ qv) * np.eye(3) + 2 * qv @ qv.T + 2 * qw * skew(qv)
    # ori_vec = R @ np.array([0, 0, 1]).T

    return R


def init_opti_sensing(num_seg):
    ''' Initialize the optitrack sensing
    Args:
        num_seg: number of segments
    Returns:
        seg_pos: 3x(num_seg+1) numpy array
        seg_ori: 3x3x(num_seg+1) numpy array
    '''
    # rospy.init_node('listener')
    seg_pos = np.zeros((num_seg + 1, 3))
    seg_qua = np.zeros((num_seg + 1, 4))
    seg_ori = np.zeros((num_seg + 1, 3, 3))

    print("here")

    # rigit bodies names = [base, secbottom, secmiddle, sectop]
    seg_name = ["base", "secbottom", "secmiddle", "sectop"]
    for i in range(num_seg + 1):

        markers_msg = rospy.wait_for_message("/optitrack/" + seg_name[i] + "/pose", PoseStamped)
        # seg_name = "czx_" + str(i)
        # print("seg_name: ", seg_name)
        # markers_msg = rospy.wait_for_message("/optitrack/" + seg_name + "/pose", PoseStamped)
        print("markers_msg: ", markers_msg)
        seg_pos[i] = np.array([markers_msg.pose.position.x, markers_msg.pose.position.y, markers_msg.pose.position.z])
        seg_qua[i] = np.array([markers_msg.pose.orientation.w, markers_msg.pose.orientation.x,
                               markers_msg.pose.orientation.y, markers_msg.pose.orientation.z])
        seg_ori[i] = qua2ori(seg_qua[i])

    pos_label = np.lexsort([seg_pos[:, 1], seg_pos[:, 2]])

    seg_pos = seg_pos[pos_label]
    seg_ori = seg_ori[pos_label]

    # seg_pos: 3
    # seg_vec: 3
    # seg_qua: 4

    return seg_pos, seg_ori


def opti_sensing(num_seg, old_pos, old_ori):
    ''' Initialize the optitrack sensing
    Args:
        num_seg: number of segments
    Returns:
        seg_pos: 3x(num_seg+1) numpy array
        seg_ori: 3x3x(num_seg+1) numpy array
    '''
    # rospy.init_node('listener')
    seg_pos = np.zeros((num_seg + 1, 3))
    seg_qua = np.zeros((num_seg + 1, 4))
    seg_ori = np.zeros((num_seg + 1, 3, 3))

    seg_name = ["base", "secbottom", "secmiddle", "sectop"]
    
    for i in range(num_seg + 1):
        markers_msg = rospy.wait_for_message("/optitrack/" + seg_name[i] + "/pose", PoseStamped)

        # seg_name = "czx_" + str(i)
        # markers_msg = rospy.wait_for_message("/optitrack/" + seg_name + "/pose", PoseStamped)
        seg_pos[i] = np.array([markers_msg.pose.position.x, markers_msg.pose.position.y, markers_msg.pose.position.z])
        seg_qua[i] = np.array([markers_msg.pose.orientation.w, markers_msg.pose.orientation.x,
                               markers_msg.pose.orientation.y, markers_msg.pose.orientation.z])
        seg_ori[i] = qua2ori(seg_qua[i])

    ri_seg_pos = np.copy(seg_pos)
    ri_seg_ori = np.copy(seg_ori)

    for i in range(num_seg + 1):
        dis_list = np.zeros([4])
        for j in range(4):
            dis_list[j] = np.linalg.norm(seg_pos[j] - old_pos[i])

        # print(seg_ori)
        # print(old_pos)
        #
        # print(dis_list)
        # sys.exit()

        idx = np.argmin(dis_list)
        ri_seg_pos[i] = np.copy(seg_pos[idx])
        # seg_pos[idx] *= 100
        ri_seg_ori[i] = seg_ori[idx]

        # if np.linalg.norm(seg_ori[idx] - old_ori[idx]) < 1:
        #     ri_seg_ori[i] = seg_ori[idx]
        # else:
        #     ri_seg_ori[i] = -seg_ori[idx]

    # seg_pos: 3
    # seg_vec: 3
    # seg_qua: 4

    return ri_seg_pos, ri_seg_ori

print("run")
