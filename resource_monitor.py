#!/usr/bin/env python3

import rospy
import psutil
from std_msgs.msg import Float32, String

def get_system_resources(previous_net_io):
    # Memory usage
    mem = psutil.virtual_memory()
    total_mem_usage = mem.percent

    # Disk space usage
    disk = psutil.disk_usage('/')
    total_disk_space = disk.percent

    # CPU usage
    total_cpu_usage = psutil.cpu_percent(interval=1)

    # Network bandwidth usage rate in GB/s
    net_io = psutil.net_io_counters()
    bytes_sent_per_sec = (net_io.bytes_sent - previous_net_io.bytes_sent) / 1  # bytes per second
    bytes_recv_per_sec = (net_io.bytes_recv - previous_net_io.bytes_recv) / 1  # bytes per second
    total_network_bandwidth_usage = (bytes_sent_per_sec + bytes_recv_per_sec) / (1024 ** 3)  # Convert to GB/s

    return total_mem_usage, total_disk_space, total_cpu_usage, total_network_bandwidth_usage, net_io

def publisher():
    rospy.init_node('system_resources_publisher', anonymous=True)
    rate = rospy.Rate(1)  # 1 Hz

    mem_pub = rospy.Publisher('total_mem_usage', Float32, queue_size=10)
    disk_pub = rospy.Publisher('total_disk_space', Float32, queue_size=10)
    cpu_pub = rospy.Publisher('total_cpu_usage', Float32, queue_size=10)
    network_pub = rospy.Publisher('total_network_bandwidth_usage', Float32, queue_size=10)

    previous_net_io = psutil.net_io_counters()

    while not rospy.is_shutdown():
        total_mem_usage, total_disk_space, total_cpu_usage, total_network_bandwidth_usage, previous_net_io = get_system_resources(previous_net_io)

        rospy.loginfo(f"Memory Usage: {total_mem_usage}%")
        rospy.loginfo(f"Disk Space Usage: {total_disk_space}%")
        rospy.loginfo(f"CPU Usage: {total_cpu_usage}%")
        rospy.loginfo(f"Network Bandwidth Usage: {total_network_bandwidth_usage:.6f} GB/s")

        mem_pub.publish(total_mem_usage)
        disk_pub.publish(total_disk_space)
        cpu_pub.publish(total_cpu_usage)
        network_pub.publish(total_network_bandwidth_usage)

        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
