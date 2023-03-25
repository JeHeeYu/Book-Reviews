# Action
## Action 이란
　ROS에서 Action은 <b>긴 시간이 소요되는 목표 지향적인 행위의 Interface를 구현하는 데 가장 좋은 방법</b>이다.
 <br>
　Service가 동기적이라면, Action은 비동기적 기법이다.
<br>
<br>
　Action은 행위를 시작하기 전 목적지와 같은 목표를 사용하며, 행위가 완료되면 결과를 보낸다.
<br>
　더 나아가 목표에 대한 행위의 진행 정보 갱신을 제공하기 위해 피드백을 사용하고, 목표의 취소도 허용한다.
<br>
<br>
　Action은 Topic을 사용하여 구현한다. 또한 많은 권한과 유연성을 제공한다.
## Action 정의
　새로운 Action을 생성하는 첫 번째 단계는 정의 파일에 목표, 결과, 피드백 3가지를 정의하는 것이다.
　관례적으로 .action의 확장자를 가지며, action Directory 내에 위치해야 한다.
<pre>
Timer.action

# 1 영역. 목표, Client로 부터 수신

# 대기하기 원하는 시간
duration time_to_wait
---

# 2 영역. 결과, 종료 시 Server로 부터 수신

# 대기 시간
duration time_elapsed

# 진행되는 동안 제공한 갱신 횟수
uint32 updates_sent
---

# 3 영역. 피드백, 실행되는 동안 서버로부터 주기적으로 수신

# 시작부터 경과된 시간
duration time_elapsed

# 종료될 때까지 남은 시간
duration time_remaining
</pre>
　Action은 총 3개의 영역<b>(목표, 결과, 피드백)</b>을 가지며, 각 영역을 대시(---)로 구분한다.
　Action 파일을 새로 생성하면 catkin build 작업이 필요하다.
<br>
　추가로, CMakeLists.txt 파일을 수정해야 한다.
<pre>
CMakeLists.txt

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
)

add_action_files(
   DIRECTORY action    // Action 정의 파일이 있는 Directory
   FILES Timer.action  // Action 정의 파일 이름
)

generate_messages(
   DEPENDENCIES
   actionlib_msgs     // 의존성 명시
   std_msgs
)

catkin_package(
   CATKIN_DEPENDS message_runtime
   CATKIN_DEPENDS
   actionlib_msgs     // 의존성 추가
)
</pre>

## Topic, Service, Action 사용 분야
　Topic : 단방향 통신, 특히 다수의 node가 청취하고 있을 때(Ex. 센서 데이터 스트림)
<br>
　Service : node의 현재 상태 질의와 같은 간단한 요청/응답 상호 작용
<br>
　Action : 요청 처리가 순간적이지 않은(Ex. 목표 위치까지 주행) 대부분의 요청/응답 상호 작용
<br>
