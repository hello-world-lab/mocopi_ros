#!/usr/bin/python
# -*- coding: utf-8 -*-

import socket
import struct
import math

import rospy

import tf2_ros
import geometry_msgs.msg


joint_map = [
    "root", #0
    "torso_1", #1
    "torso_2", #2
    "torso_3", #3
    "torso_4", #4
    "torso_5", #5
    "torso_6", #6
    "torso_7", #7
    "neck_1", #8
    "neck_2", #9
    "head", #10
    "l_shoulder", #11
    "l_up_arm", #12
    "l_low_arm", #13
    "l_hand", #14
    "r_shoulder", #15
    "r_up_arm", #16
    "r_low_arm", #17
    "r_hand", #18
    "l_up_leg", #19
    "l_low_leg", #20
    "l_foot", #21
    "l_toes", #22
    "r_up_leg", #23
    "r_low_leg", #24
    "r_foot", #25
    "r_toes" #26
]


def is_field(name):
    """
    The is_field function quoted from:
    Project: mcp-receiver
    URL: https://github.com/seagetch/mcp-receiver
    LICENSE: MIT
    Copyright (c) 2022 seagetch
    """
    return name.isalpha()

def _deserialize(data, index, length, is_list = False):
    """
    The is_field function quoted from:
    Project: mcp-receiver
    URL: https://github.com/seagetch/mcp-receiver
    LICENSE: MIT
    Copyright (c) 2022 seagetch
    """
    result = [] if is_list else {}
    end_pos = index + length
    while end_pos - index > 8 and is_field(data[index+4:index+8]):
        size = struct.unpack("@i", data[index: index+4])[0]
        index += 4
        field = data[index:index+4]
        index += 4
        value, index2 = _deserialize(data, index, size, field in [b"btrs", b"bons"])
        index = index2
        if is_list:
            result.append(value)
        else:
            result[field.decode()] = value
    if len(result) == 0:
        body  = data[index:index+length]
        return body, index + len(body)
    else:
        return result, index

def _process_packet(message):
    """
    The is_field function quoted from:
    Project: mcp-receiver
    URL: https://github.com/seagetch/mcp-receiver
    LICENSE: MIT
    Copyright (c) 2022 seagetch
    """
    data = _deserialize(message, 0, len(message), False)[0]
    data["head"]["ftyp"] = data["head"]["ftyp"].decode()
    data["head"]["vrsn"] = ord(data["head"]["vrsn"])
    data["sndf"]["ipad"] = struct.unpack("@BBBBBBBB", data["sndf"]["ipad"])
    data["sndf"]["rcvp"] = struct.unpack("@H", data["sndf"]["rcvp"])[0]
    if "skdf" in data:
        for item in data["skdf"]["bons"]:
            item["bnid"] = struct.unpack("@H", item["bnid"])[0]
            item["pbid"] = struct.unpack("@H", item["pbid"])[0]
            item["tran"] = struct.unpack("@fffffff", item["tran"])
    elif "fram" in data:
        data["fram"]["fnum"] = struct.unpack("@I", data["fram"]["fnum"])[0]
        data["fram"]["time"] = struct.unpack("@I", data["fram"]["time"])[0]
        for item in data["fram"]["btrs"]:
            item["bnid"] = struct.unpack("@H", item["bnid"])[0]
            item["tran"] = struct.unpack("@fffffff", item["tran"])
    return data

def make_tf(pframe_id, cframe_id, data):

    t = geometry_msgs.msg.TransformStamped()

    if pframe_id >= 0 and pframe_id <= 26:
        t.header.frame_id = joint_map[pframe_id]
    else:
        t.header.frame_id = "map"


    for btdt in data["fram"]["btrs"]:
        if btdt["bnid"] == cframe_id:
            t.child_frame_id = joint_map[cframe_id]
            trans = btdt["tran"]

            t.header.stamp = rospy.Time.now()
            t.transform.translation.x = trans[6]
            t.transform.translation.y = trans[4]
            t.transform.translation.z = trans[5]
            t.transform.rotation.x = trans[2]
            t.transform.rotation.y = trans[0]
            t.transform.rotation.z = trans[1]
            t.transform.rotation.w = trans[3]

            break

    return t

def mocopi_receiver():
    rospy.init_node('mocopi_receiver', anonymous=True)
    br = tf2_ros.TransformBroadcaster()

    transforms = []

    addr=""
    port=12351

    mocopisocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    mocopisocket.bind((addr, port))

    while not rospy.is_shutdown():
        try:
            message, client_addr = mocopisocket.recvfrom(2048)
            data = _process_packet(message)
        except KeyError as e:
            rospy.logerr("socket err: %s", e)


        if "fram" in data: 
            #send transform 0-10 head
            trans = make_tf(99, 0, data)
            if trans.child_frame_id != "":
                transforms.append(trans)
            for i in range(0, 10):
                trans = make_tf(i, i+1, data)
                if trans.child_frame_id != "":
                    transforms.append(trans)
                    

            #send transform  11-14 lefthand
            trans = make_tf(7, 11, data)
            if trans.child_frame_id != "":
                transforms.append(trans)
            for i in range(11, 14):
                trans = make_tf(i, i+1, data)
                if trans.child_frame_id != "":
                    transforms.append(trans)

            #send transform  15-18 righthand
            trans = make_tf(7, 15, data)
            if trans.child_frame_id != "":
                transforms.append(trans)
            for i in range(15, 18):
                trans = make_tf(i, i+1, data)
                if trans.child_frame_id != "":
                    transforms.append(trans)

            #send transform  19-21 leftleg
            trans = make_tf(0, 19, data)
            if trans.child_frame_id != "":
                transforms.append(trans)
            for i in range(19, 22):
                trans = make_tf(i, i+1, data)
                if trans.child_frame_id != "":
                    transforms.append(trans)
            
            #send transform  23-26 leftleg
            trans = make_tf(0, 23, data)
            if trans.child_frame_id != "":
                transforms.append(trans)
            for i in range(23, 26):
                trans = make_tf(i, i+1, data)
                if trans.child_frame_id != "":
                    transforms.append(trans)

        br.sendTransform(transforms)
        transforms.clear()







if __name__ == '__main__':
    try:
        mocopi_receiver()

    except rospy.ROSInterruptException: pass