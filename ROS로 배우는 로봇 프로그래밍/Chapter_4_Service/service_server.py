#!/usr/bin/env python

# 단어 개수 검출 예제

import rospy

# 새롭게 정의한 Service
from basics.srv import WordCount, WordCountResponse

def count_words(request):
    # 실패한 Service는 NULL 반환
    return WordCountResponse(len(request.words.split()))

rospy.init_node('service_server')

# Service 이름과 Service 자료형, Service Callback 함수 정의
service = rospy.Service('word_count', WordCount, count_words)

rospy.spin()
