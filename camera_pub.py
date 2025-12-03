#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    rospy.init_node('gst_camera_publisher', anonymous=True)
    image_pub = rospy.Publisher('/my_camera/image_raw', Image, queue_size=10)
    bridge = CvBridge()

    # [핵심] 여기에 gst-launch 명령어를 'appsink' 버전으로 넣으세요.
    # 예시: 웹캠(/dev/video0)인 경우
    # gst_str = "v4l2src device=/dev/video0 ! videoconvert ! video/x-raw,format=BGR ! appsink drop=1"
    
    # 예시: CSI 카메라나 RTSP 스트림인 경우 (본인이 쓰던 명령어 변형)
    # gst_str = "rtspsrc location=rtsp://192.168.1.50:8554/test ! decodebin ! videoconvert ! video/x-raw,format=BGR ! appsink drop=1"
    
    # 지금 사용 중인 gst-launch 명령어에서 끝부분만 appsink로 바꾸면 됩니다.
    gst_str = "v4l2src device=/dev/video0 ! videoconvert ! video/x-raw,format=BGR ! appsink drop=1"

    # OpenCV로 GStreamer 파이프라인 열기
    cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        rospy.logerr("카메라를 열 수 없습니다! GStreamer 파이프라인을 확인하세요.")
        return

    rate = rospy.Rate(30) # 30hz
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            # OpenCV(BGR) 이미지를 ROS 메시지로 변환
            msg = bridge.cv2_to_imgmsg(frame, "bgr8")
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "camera_frame" # TF 연결할 때 쓸 이름
            image_pub.publish(msg)
        else:
            rospy.logwarn("프레임을 읽을 수 없습니다.")
        
        rate.sleep()

    cap.release()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
