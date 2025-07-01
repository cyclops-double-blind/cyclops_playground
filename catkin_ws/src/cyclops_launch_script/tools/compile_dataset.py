import argparse
from os.path import join

import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge
from rosbag import Bag

try:
    from tqdm import tqdm  # type: ignore
except ImportError:
    tqdm = None

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Point, Quaternion, PoseStamped

cv_bridge = CvBridge()


def parseDataCSV(root):
    with open(join(root, "data.csv"), "r") as f:
        lines = f.readlines()
    return [[e for e in line.strip().split(",")] for line in lines[1:]]


def parseTimestamp(timestamp):
    onesec_in_ns = 1000000000
    sec = timestamp // onesec_in_ns
    nsec = timestamp % onesec_in_ns
    return rospy.Time(sec, nsec)


def parseImageTargetEncoding(cv_image):
    def handle_mono_image():
        if cv_image.dtype in (np.int8, np.uint8):
            return "mono8"
        if cv_image.dtype in (np.int16, np.uint16):
            return "mono16"
        raise ValueError("Unknown CV dtype {}".format(cv_image.dtype))

    if len(cv_image.shape) < 3:
        return handle_mono_image()

    _, _, channel = cv_image.shape
    if channel == 1:
        return handle_mono_image()

    if channel == 3:
        return "rgb8"

    raise ValueError("Intractable image channel width: {}".format(channel))


def parseImage(timestamp, filepath):
    cv_image = cv2.imread(filepath, cv2.IMREAD_UNCHANGED)
    encoding = parseImageTargetEncoding(cv_image)

    image = cv_bridge.cv2_to_imgmsg(cv_image, encoding=encoding)
    image.header.stamp = parseTimestamp(timestamp)

    return image


def parseIMU(timestamp, rotation, acceleration):
    msg = Imu()
    msg.header.stamp = parseTimestamp(timestamp)
    msg.angular_velocity = Vector3(*rotation)
    msg.linear_acceleration = Vector3(*acceleration)
    return msg


def parsePosestamped(frame, timestamp, position, orientation):
    msg = PoseStamped()
    msg.header.stamp = parseTimestamp(timestamp)
    msg.header.frame_id = frame
    msg.pose.position = Point(*position)

    w, x, y, z = orientation
    msg.pose.orientation = Quaternion(x, y, z, w)
    return msg


def parseVector(data, col_start, col_end):
    return [tuple(float(_) for _ in line[col_start:col_end]) for line in data]


def readStartTime(root):
    data = parseDataCSV(root)
    return int(data[0][0])


def readCamera(root, time_offset):
    print("Reading camera data: {}".format(root))

    data = parseDataCSV(root)

    def make_result(progress=None):
        result = [None] * len(data)  # type: list
        for i, (timestamp, filename) in enumerate(data):
            result[i] = parseImage(
                int(timestamp) - time_offset, join(root, "data", filename)
            )
            if progress is not None:
                progress.update(1)
        return result

    if tqdm is None:
        return make_result()

    with tqdm(total=len(data), unit="file") as progress:
        return make_result(progress)


def readIMU(root, time_offset):
    print("Reading IMU data: {}".format(root))

    data = parseDataCSV(root)
    return [
        parseIMU(*entry)
        for entry in zip(
            [int(line[0]) - time_offset for line in data],
            parseVector(data, 1, 4),
            parseVector(data, 4, 7),
        )
    ]


def readGroundtruth(root, time_offset, frame):
    print("Reading ground truth data: {}".format(root))

    data = parseDataCSV(root)
    return [
        parsePosestamped(frame, *entry)
        for entry in zip(
            [int(line[0]) - time_offset for line in data],
            parseVector(data, 1, 4),
            parseVector(data, 4, 8),
        )
    ]


def processDataset(data_root, timestamp_base, groundtruth_stream):
    start_time = readStartTime(join(data_root, "imu0"))
    time_offset = start_time - timestamp_base

    imu_data = readIMU(join(data_root, "imu0"), time_offset)
    truth_estim_data = readGroundtruth(
        join(data_root, groundtruth_stream), time_offset, "map"
    )
    camera_data = readCamera(join(data_root, "cam0"), time_offset)

    return {
        "camera": camera_data,
        "imu": imu_data,
        "truth_estim": truth_estim_data,
    }


def writeOutput(data, output):
    if tqdm is None:
        for topic, msgs in data.items():
            for msg in msgs:
                output.write(topic, msg, msg.header.stamp)
    else:
        for topic, msgs in tqdm(data.items(), unit="topic"):
            for msg in tqdm(msgs, unit="message"):
                output.write(topic, msg, msg.header.stamp)


def parseArgs():
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
    parser.add_argument(
        "--groundtruth-stream-name",
        "-g",
        type=str,
        default="state_groundtruth_estimate0",
    )

    args = parser.parse_args()
    return args


def main():
    args = parseArgs()
    data = processDataset(
        args.data_dir, args.timestamp_base, args.groundtruth_stream_name
    )

    print("Writing output")
    with Bag(args.output_bag, "w") as output:
        writeOutput(data, output)


if __name__ == "__main__":
    main()
