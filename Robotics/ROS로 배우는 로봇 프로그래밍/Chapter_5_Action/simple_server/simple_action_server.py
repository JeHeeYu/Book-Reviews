#!/usr/bin/env python

# Action 목표가 5초 대기인 예제

import rospy
import time

# ROS actionlib Package
import actionlib

# 새롭게 정의한 Action
from basics.msg import TimerAction, TimerGoal, TimerResult

# 새로운 목푤르 수신했을 때 호출되는 함수
# 목표에서 요청한 시간 동안 일시 정지
def do_timer(goal):
    # 현재 시각
    start_time = time.time()

    # ROS duration 필드로, 초 단위로 변경하여 목표에서 요청한 시간 동안 일시 정지
    time.sleep(goal.time_to_wait.to_sec())

    # TimerResult 자료형인 결과 메시지 result
    result = TimerResult()

    # ROS duration 자료형으로 변환하여 현재 시각에서 시작 시간 차감
    result.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)


    result.updates_sent = 0

    # 서버에 결과 전송
    server.set_succeeded(result)

# node 초기화
rospy.init_node('timer_action_server')

# SimepleActionServer 생성
# Action을 구성하고 토픽이 광고될 'timer' 토픽, TimerAction 자료형, do_timer 콜백 함수, 서버의 자동 시작 False
server = actionlib.SimpleActionServer('timer', TimerAction, do_timer, False)

# 서버 시작
server.start()

rospy.spin()
