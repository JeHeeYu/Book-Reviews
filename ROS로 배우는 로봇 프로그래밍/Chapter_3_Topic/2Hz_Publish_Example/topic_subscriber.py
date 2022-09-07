#!/usr/bin/env python

# 2Hz 속도로 counter 토픽 정수를 반복적으로 발행하는 예제

import rospy

# ROS 표준 Message Package인 std_msgs에 정의된 32비트 정수 사용
from std_msgs.msg import Int32

# Message가 들어올 때 처리하는 Callback 함수
# 들어오는 Message 출력
def callback(msg):
    print(msg.data)

# topic_subscriber node 초기화
rospy.init_node('topic_subscriber')

# Int32 자료형인 'counter' Topic을 구독하고, callback 함수 지정
sub = rospy.Subscriber('counter', Int32, callback)

# ROS에게 제어권을 넘김
rospy.spin()
