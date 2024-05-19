import argparse
import os
from os.path import join

import cv2
import rospy
from cv_bridge import CvBridge
from rosbag import Bag

try:
    from tqdm import tqdm
except ImportError:
    tqdm = None

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion, PoseStamped

cv_bridge = CvBridge()


def parse_data_csv(root):
    with open(join(root, "data.csv"), "r") as data_csv_f:
        lines = data_csv_f.readlines()
    return [[e for e in line.strip().split(",")] for line in lines[1:]]


def parse_timestamp(timestamp):
    onesec_in_ns = 1000000000
    sec = timestamp // onesec_in_ns
    nsec = timestamp % onesec_in_ns
    return rospy.Time(sec, nsec)


def parse_image(timestamp, filepath):
    image = cv_bridge.cv2_to_imgmsg(
        cv2.imread(filepath, cv2.IMREAD_UNCHANGED), encoding="mono8"
    )
    image.header.stamp = parse_timestamp(timestamp)
    return image


def parse_imu(timestamp, rotation, acceleration):
    msg = Imu()
    msg.header.stamp = parse_timestamp(timestamp)
    msg.angular_velocity = Vector3(*rotation)
    msg.linear_acceleration = Vector3(*acceleration)
    return msg


def parse_posestamped(frame, timestamp, position, orientation):
    msg = PoseStamped()
    msg.header.stamp = parse_timestamp(timestamp)
    msg.header.frame_id = frame
    msg.pose.position = Vector3(*position)

    w, x, y, z = orientation
    msg.pose.orientation = Quaternion(x, y, z, w)
    return msg


def parse_vector(data, col_start, col_end):
    return [tuple(float(_) for _ in line[col_start:col_end]) for line in data]


def read_start_time(root):
    data = parse_data_csv(root)
    return int(data[0][0])


def read_camera(root, time_offset):
    print("reading camera data: {}".format(root))

    data = parse_data_csv(root)

    def make_result(progress=None):
        result = [None] * len(data)
        for (i, (timestamp, filename)) in enumerate(data):
            result[i] = parse_image(
                int(timestamp) - time_offset, join(root, "data", filename)
            )
            if progress is not None:
                progress.update(1)
        return result

    if tqdm is None:
        return make_result()

    with tqdm(total=len(data), unit="file") as progress:
        return make_result(progress)


def read_imu(root, time_offset):
    print("reading IMU data: {}".format(root))

    data = parse_data_csv(root)
    return [
        parse_imu(*entry)
        for entry in zip(
            [int(line[0]) - time_offset for line in data],
            parse_vector(data, 1, 4),
            parse_vector(data, 4, 7),
        )
    ]


def read_groundtruth(root, time_offset, frame):
    print("reading ground truth data: {}".format(root))

    data = parse_data_csv(root)
    return [
        parse_posestamped(frame, *entry)
        for entry in zip(
            [int(line[0]) - time_offset for line in data],
            parse_vector(data, 1, 4),
            parse_vector(data, 4, 8),
        )
    ]


def process_dataset(data_root, timestamp_base):
    start_time = read_start_time(join(data_root, "imu0"))
    time_offset = start_time - timestamp_base

    imu_data = read_imu(join(data_root, "imu0"), time_offset)
    truth_estim_data = read_groundtruth(
        join(data_root, "state_groundtruth_estimate0"), time_offset, "map"
    )
    camera_data = read_camera(join(data_root, "cam0"), time_offset)

    return {
        "camera": camera_data,
        "imu": imu_data,
        "truth_estim": truth_estim_data,
    }


def write_output(data, output):
    if tqdm is None:
        for (topic, msgs) in data.items():
            for msg in msgs:
                output.write(topic, msg, msg.header.stamp)
    else:
        for (topic, msgs) in tqdm(data.items(), unit="topic"):
            for msg in tqdm(msgs, unit="message"):
                output.write(topic, msg, msg.header.stamp)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output_bag",
        "-o",
        type=str,
        help="The output bag file to write to",
    )
    parser.add_argument(
        "--data-dir",
        "-d",
        type=str,
        help="Path to the data directory",
    )
    parser.add_argument(
        "--timestamp-base",
        "-t",
        type=int,
        default=1403636579763555584,
    )

    args = parser.parse_args()
    return args


def main():
    args = parse_args()
    data = process_dataset(args.data_dir, args.timestamp_base)

    print("writing output")
    with Bag(args.output_bag, "w") as output:
        write_output(data, output)


if __name__ == "__main__":
    main()
