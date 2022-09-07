#!/usr/bin/env python

# Action 목표가 5초 대기인 예제

import rospy

# ROS actionlib Package
import actionlib

# 새롭게 정의한 Action
from basics.msg import TimerAction, TimerGoal, TimerResult

# node 초기화
rospy.init_node('timer_action_client')

# SimpleActionClient 생성
# Action Client의 이름인 'topic'은 Client가 Server와 통신할 때 사용되는 Topic
# TimerAction 자료향
client = actionlib.SimpleActionClient('timer', TimerAction)

# Server가 동작할 때까지 대기
client.wait_for_server()

# TimerGoal 자료형 객체 생성
goal = TimerGoal()

# 대기해야 할 타이머 정의(5초)
goal.time_to_wait = rospy.Duration.from_sec(5.0)

# Server로 타이머 전송
client.send_goal(goal)

# Server로 부터 결과 수신 시 까지 대기
client.wait_for_result()

# Server에서 수신한 time 출력
print('Time elapsed: %f'%(client.get_result().time_elapsed.to_sec()))
