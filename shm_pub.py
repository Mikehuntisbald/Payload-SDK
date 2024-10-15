import rospy
import cv2
import numpy as np
import mmap
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
import fcntl

# 初始化 ROS 节点
rospy.init_node('shared_memory_reader', anonymous=True)

# 创建 ROS 发布者
image_publisher_1 = rospy.Publisher('/shared_memory_image_1', Image, queue_size=10)
image_publisher_2 = rospy.Publisher('/shared_memory_image_2', Image, queue_size=10)

# 创建 CvBridge 实例
bridge = CvBridge()

# 共享内存设置
shared_memory_name = "MySharedMemory"

try:
    # 获取共享内存的大小
    shared_memory_path = f"/dev/shm/{shared_memory_name}"
    shared_memory_size = os.path.getsize(shared_memory_path)

    # 打开共享内存
    shm_fd = open(shared_memory_path, "r+b")
    shared_mem = mmap.mmap(shm_fd.fileno(), shared_memory_size, access=mmap.ACCESS_READ)

    rate = rospy.Rate(10)  # 以 10Hz 的频率读取数据

    while not rospy.is_shutdown():
        # 加锁共享内存以确保读取过程的完整性
        fcntl.flock(shm_fd, fcntl.LOCK_SH)
        try:
            # 读取共享内存中的元数据（行数、列数、类型）
            shared_mem.seek(0)
            meta_data = shared_mem.read(12)  # 读取 3 个 int (4 bytes * 3)
            rows, cols, img_type = np.frombuffer(meta_data, dtype=np.int32)

            # 读取共享内存中的第一张图像数据
            img_data_1 = shared_mem.read(rows * cols * (3 if img_type == cv2.CV_8UC3 else 1))
            img_1 = np.frombuffer(img_data_1, dtype=np.uint8)
            if img_type == cv2.CV_8UC3:
                img_1 = img_1.reshape((rows, cols, 3))
            else:
                img_1 = img_1.reshape((rows, cols))

            # 读取共享内存中的第二张图像数据
            img_data_2 = shared_mem.read(rows * cols * (3 if img_type == cv2.CV_8UC3 else 1))
            img_2 = np.frombuffer(img_data_2, dtype=np.uint8)
            if img_type == cv2.CV_8UC3:
                img_2 = img_2.reshape((rows, cols, 3))
            else:
                img_2 = img_2.reshape((rows, cols))

            # 发布第一张图像
            header_1 = Header()
            header_1.stamp = rospy.Time.now()
            header_1.frame_id = "shared_memory_frame_1"
            encoding_1 = "bgr8" if img_type == cv2.CV_8UC3 else "mono8"
            image_msg_1 = bridge.cv2_to_imgmsg(img_1, encoding=encoding_1)
            image_msg_1.header = header_1
            image_publisher_1.publish(image_msg_1)

            # 发布第二张图像
            header_2 = Header()
            header_2.stamp = rospy.Time.now()
            header_2.frame_id = "shared_memory_frame_2"
            encoding_2 = "bgr8" if img_type == cv2.CV_8UC3 else "mono8"
            image_msg_2 = bridge.cv2_to_imgmsg(img_2, encoding=encoding_2)
            image_msg_2.header = header_2
            image_publisher_2.publish(image_msg_2)

            # 使用 OpenCV 显示图像
            cv2.imshow("Shared Memory Image 1", img_1)
            cv2.imshow("Shared Memory Image 2", img_2)
            cv2.waitKey(1)
        finally:
            # 释放共享内存的锁
            fcntl.flock(shm_fd, fcntl.LOCK_UN)

        rate.sleep()

except Exception as e:
    rospy.logerr(f"Error reading from shared memory: {e}")

finally:
    # 清理资源
    shared_mem.close()
    shm_fd.close()
    cv2.destroyAllWindows()