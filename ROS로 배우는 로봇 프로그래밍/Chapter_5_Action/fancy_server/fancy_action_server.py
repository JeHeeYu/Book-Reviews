#!/usr/bin/env python

# 5초에 한 번씩 

import rospy
import time

import actionlib

# 새롭게 정의한 Action
from basics.msg import TimerAction, TimerGoal, TimerResult, TimerFeedback

def do_timer(goal):
    # 시작 시각
    start_time = time.time()

    # 피드백 발행 횟수
    update_count = 0

    # 요청된 타이머가 60초 이상일 경우 set_aborted()를 호출하여 목표를 중지함
    if goal.time_to_wait.to_sec() > 60.0:

        # TimerResult 자료형인 결과 메시지 result
        result = TimerResult()

        # ROS duration 자료형으로 변환하여 현재 시각에서 시작 시간 차감
        result.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)
        result.updates_sent = update_count

        # 호출 중단
        server.set_aborted(result, "Timer aborted due to too-long wait")

        return

    # 목표 시간이 60초 이하일 때 반복
    while (time.time() - start_time) < goal.time_to_wait.to_sec():

        if server.is_preempt_requested():
            result = TimerResult()
            result.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)

            result.updates_sent = update_count
            server.set_preempted(result, "Timer preempted")
            return
        
        # 피드백 자료형 객체 생성
        feedback = TimerFeedback()
        feedback.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)
        feedback.time_remaining = goal.time_to_wait - feedback.time_elapsed

        # 클라이언트로 피드백 전송
        server.publish_feedback(feedback)

        # 피드백 전송 횟수 증가
        update_count += 1

        time.sleep(1.0)

    # 요청한 기간 동안 성공적으로 일시 정지했을 경우 일이 끝남을 클라이언트에게 알림
    retult = TimerResult()
    result.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)
    result.updates_sent = update_count
    server.set_succeeded(result, "Timer completed successfully")

rospy.init_node('timer_action_server')
server = actionlib.SimpleActionServer('timer', TimerAction, do_timer, False)
server.start()
rospy.spin()
