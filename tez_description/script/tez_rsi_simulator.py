#!/usr/bin/env python3

import sys
import socket
import numpy as np
import time
import xml.etree.ElementTree as ET

import errno
import rospy

from sensor_msgs.msg import JointState

MAX_UDP_MSG_LENGTH = 1024
cycle_time = 0.004
timeout_count = 0
node_name = 'tez_rsi_simulation'
joints_name = ['kr210_1_joint_1',
               'kr210_1_joint_2',
               'kr210_1_joint_3',
               'kr210_1_joint_4',
               'kr210_1_joint_5',
               'kr210_1_joint_6',
               'kr210_1_linear_axis_joint']


def create_rsi_xml_rob(joints_pos: np.ndarray, setpoint_joints: np.ndarray, timeout_count: int, ipoc: int) -> bytes:
    q = joints_pos
    qd = setpoint_joints
    root = ET.Element('Rob', {'TYPE': 'KUKA'})
    ET.SubElement(root, 'RIst', {'X': '0.0', 'Y': '0.0', 'Z': '0.0',
                                 'A': '0.0', 'B': '0.0', 'C': '0.0'})
    ET.SubElement(root, 'RSol', {'X': '0.0', 'Y': '0.0', 'Z': '0.0',
                                 'A': '0.0', 'B': '0.0', 'C': '0.0'})
    ET.SubElement(root, 'AIPos', {'A1': str(q[0]), 'A2': str(q[1]), 'A3': str(q[2]),
                                  'A4': str(q[3]), 'A5': str(q[4]), 'A6': str(q[5])})
    ET.SubElement(root, 'ASPos', {'A1': str(qd[0]), 'A2': str(qd[1]), 'A3': str(qd[2]),
                                  'A4': str(qd[3]), 'A5': str(qd[4]), 'A6': str(qd[5])})
    ET.SubElement(root, 'EIPos', {'E0': str(q[6])})
    ET.SubElement(root, 'ESPos', {'E0': str(qd[6])})
    ET.SubElement(root, 'Delay', {'D': str(timeout_count)})
    ET.SubElement(root, 'IPOC').text = str(ipoc)
    return ET.tostring(root)


def parse_rsi_xml_sen(data):
    root = ET.fromstring(data)
    AK = root.find('AK').attrib
    desired_joint_correction = np.array([AK['A1'], AK['A2'], AK['A3'],
                                         AK['A4'], AK['A5'], AK['A6']]).astype(np.float64)
    EK = root.find('EK').attrib
    desired_external_axis_correction = np.array([EK['E0']]).astype(np.float64)
    IPOC = root.find('IPOC').text
    return np.concatenate([desired_joint_correction, desired_external_axis_correction], axis=0), int(IPOC)


def shutdown_hook():
    rospy.loginfo('{}: Shutting down'.format(node_name))
    s.close()


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='KUKA RSI Simulation')
    parser.add_argument('rsi_hw_iface_ip', help='The ip address of the RSI control interface')
    parser.add_argument('rsi_hw_iface_port', help='The port of the RSI control interface')
    parser.add_argument('--sen', default='ImFree', help='Type attribute in RSI XML doc. E.g. <Sen Type:"ImFree">')
    # Only parse known arguments
    args, _ = parser.parse_known_args()

    print(args.rsi_hw_iface_ip)
    host = args.rsi_hw_iface_ip
    port = int(args.rsi_hw_iface_port)
    sen_type = args.sen

    rospy.init_node('tez_rsi_simulation')
    rospy.loginfo('{}: Started'.format(node_name))

    rsi_act_pub = rospy.Publisher('rsi/joint_states', JointState, queue_size=1)
    rsi_cmd_pub = rospy.Publisher('/rsi/command', JointState, queue_size=1)

    joint_state_msg: JointState = JointState()
    cmd_msg: JointState = JointState()

    act_joint_pos = np.array([0, 0, 0, 0, 0, 0, 0]).astype(np.float64)
    cmd_joint_pos = act_joint_pos.copy()
    des_joint_correction_absolute = np.zeros(6)

    joint_state_msg.name = joints_name
    joint_state_msg.position = act_joint_pos
    joint_state_msg.header.stamp = rospy.Time.now()

    cmd_msg.name = joints_name
    cmd_msg.position = cmd_joint_pos
    cmd_msg.header.stamp = rospy.Time.now()

    ipoc = 0

    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rospy.loginfo('{}, Successfully created socket'.format(node_name))
        s.settimeout(1)
    except socket.error as e:
        rospy.logfatal('{}Could not create socket'.format(node_name))
        sys.exit()

    rospy.on_shutdown(shutdown_hook)

    while not rospy.is_shutdown():
        try:
            msg = create_rsi_xml_rob(act_joint_pos, cmd_joint_pos, timeout_count, ipoc)

            s.sendto(msg, (host, port))
            recv_msg, addr = s.recvfrom(MAX_UDP_MSG_LENGTH)
            des_joint_correction_absolute, ipoc_recv = parse_rsi_xml_sen(recv_msg.decode('utf-8'))

            if ipoc != ipoc_recv:
                rospy.logerr('Mismatch between IPOC values, sent: %s received: %s ; most likely Twincat lost some msgs', ipoc, ipoc_recv)
                break

            # Let's assume the robot execute perfectly the given command
            print(des_joint_correction_absolute)

            cmd_joint_pos += des_joint_correction_absolute
            act_joint_pos = cmd_joint_pos.copy()

            joint_state_msg.position = act_joint_pos
            joint_state_msg.header.stamp = rospy.Time.now()
            rsi_act_pub.publish(joint_state_msg)

            ipoc += 1
            time.sleep(cycle_time / 2)

        except socket.timeout as msg:
            rospy.logwarn('{}:Socket timed out'.format(node_name))
            timeout_count += 1
        except socket.error as e:
            if e.errno != errno.EINTR:
                raise
