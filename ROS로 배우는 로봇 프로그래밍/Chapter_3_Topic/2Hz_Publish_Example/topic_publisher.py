#!/usr/bin/env python

# 2Hz 속도로 counter 토픽 정수를 반복적으로 발행하는 예제

import rospy

# ROS 표준 Message Package인 std_msgs에 정의된 32비트 정수 사용
from std_msgs.msg import Int32

# 'topic_publish' 라는 node 초기화
rospy.init_node('topic_publisher')

# 'count'라는 Topic에 Int32 자료형
pub = rospy.Publisher('counter', Int32)

# 발행 속도(2초) 설정
rate = rospy.Rate(2)

# 카운트 횟수
count = 0

# is_shutdown() 함수는 node가 종료될 상황이면 True 반환
# node가 종료되지 않으면 반복하면서 count 횟수 Publishing
while not rospy.is_shutdown():
    pub.publish(count)
    count += 1
    rate.sleep()
