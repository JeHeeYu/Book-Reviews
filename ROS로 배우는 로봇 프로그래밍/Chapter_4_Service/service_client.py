#!/usr/bin/env python

# 단어 개수 검출 예제

import rospy

# 새롭게 정의한 Service
from basics.srv import WordCount

import sys

rospy.init_node('service_client')

# Server에서 'word_count' Service가 광고될 때까지 대기
rospy.wait_for_service('word_count')

# Service가 광고되기 전에 사용하려 하면 오류가 발생함 (토픽과 큰 차이점)
# Service가 광고되고 나면 지역 Proxy 설정 가능
word_counter = rospy.ServiceProxy('word_count', WordCount)

words = ' '.join(sys.argv[1:])

# Service 이름(word_count)과 자료형(WordCount) 명시 필요
word_count = word_counter(words)

print(words, '->', word_count.count)
