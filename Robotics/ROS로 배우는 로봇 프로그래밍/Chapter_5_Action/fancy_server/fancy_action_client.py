#!/usr/bin/env python

import rospy

import time
import actionlib

# 새롭게 정의한 Action
from basics.msg import TimerAction, TimerGoal, TimerResult, TimerFeedback

# 피드백 메시지를 받을 때 호출될 콜백 함수로 피드백의 내용 출력
def feedback_cb(feedback):
    print('[Feedback] Time elapsed: %f'%(feedback.time_elapsed.to_sec()))
    print('[Feedback] Time remaining: %f'%(feedback.time_remaining.to_sec()))

rospy.init_node('timer_action_client')
client = actionlib.SimpleActionClient('timer', TimerAction)

# 서버가 시작될 때까지 대기
client.wait_for_server()

# 서버 중단 테스트
# goal.time_to_wait = rospy.Duration.from_sec(5.0)

goal = TimerGoal()
goal.time_to_wait = rospy.Duration.from_sec(5.0)


# 목표 선점 테스트
#time.sleep(3.0)
#client.cancel_goal()


# send_goal() 호출 시 feedback_cb 키워드 인자로 피드백 콜백을 전달하여 등록
client.send_goal(goal, feedback_cb = feedback_cb)

# 결과물 출력
client.wait_for_result()
print('[Result] State : %d'%(client.get_state()))
print('[Result] Status : %s'%(client.get_goal_status_text()))
print('[Result] Time elapsed : %f'%(client.get_result().time_elapsed.to_sec()))
print('[Result] Updates sent : %d'%(client.get_result().updates_sent))
