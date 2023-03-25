# Topic
## Topic이란
　Topic이란 node들 간의 통신을 할 수 있는 채널을 의미한다. 정의된 자료형을 가지는 Message Stream의 의미이다.
<br>
　Topic은 데이터를 교환하는 방법 중 발행(Publish)/구독(Subsribe) 통신 기법을 사용한다.
<br>
　node는 사용할 Topic 이름과 Message 자료형을 roscore에게 먼저 광고(Advertise)한다.
<br>
<br>
　광고하게 되면, 실제 node들은 Topic으로 Message를 발행할 수 있는 상태가 된다.
<br>
　node들이 어떤 Topic의 Message를 수신할 수 있도록 roscore에게 해당 Topic을 구독할 수 있다.
<br>
<br>
　ROS에서 동일한 Topic의 모든 Message들은 동일한 자료형을 가진다.
<br>
　또 하나의 관례로, Topic 이름은 Topic으로 전송되는 Message의 종류를 나타낸다. (video, image 등)
<br>
## Latched Topic
　Latched Topic이란 Topic이 광고될 때 전송된 제일 마지막 Message를 자동적으로 얻을 수 있는 Topic이다.
<br>
　latched 인자를 사용하여 latch 여부를 결정한다.
<pre>
pub = rospy.Publisher('map', nav_msgs_OccupancyGrid, latched=True)
</pre>
## ROS Message Type
　ROS는 std_msgs Package에 기본 자료형을 정의하며, 이러한 자료형의 배열은 고정 길이 또는 가변 길이 관계없이
<br>
　파이썬에서 튜플이나 리스트로 사용할 수 있다.
<pre>
ROS 자료형         직렬화         C++자료형       Python 자료형                     비고
  bool        무부호 8비트 정수   uint8_t           bool
  int8        부호 8비트 정수     int8_t            int  
  uint8       무부호 8비트 정수   uint8_t            int               uint8[]은 파이썬에서 문자열로 취급
  int16       부호 16비트 정수    int16_t           int
  uint16      무부호 16비트 정수   uint16_t          int
  int32       부호 32비트 정수    int32_t            int
  uint32      무부호 32비트 정수   uint32_t          int
  int64       부호 64비트 정수     int64_t           long
  uint64      무부호 64비트 정수   uint64_t          long
  float32     32비트 IEEE 실수    float             float
  float64     64비트 IEEE 실수    double            float
  string      ASCII 문자열       std::string       string             ROS는 Unicode 문자열을 지원하지 않아 UTF-8 인코딩 사용
  time        32비트 정수         ros::Time         rospy-time
</pre>
### New Message 정의 방법
　ROS는 특수한 Message를 정의할 수 있ㅇ며, Package 내 msg 폴더에 정의한다.
<br>
　새롭게 정의한 파일은 언어별로 다르게 컴파일된다. 즉 언어를 정의하고 catkin build 해야함을 뜻한다.
<pre>
Complex.msg

float32 real
float32 imaginary
</pre>
　새롭게 정의할 Message들을 .msg 파일 내 등록한다.
<br>
　real 이라는 float32 자료형, imaginary라는 float32 자료형이 새로 정의된다.
<br>
<br>
　ROS가 언어별 Message Code를 생성할 수 있도록 Build System에 알려주어야 한다.
<br>
　package.xml 파일 하단에 하기 태그를 추가한다.
<pre>
<build_depend>message_generation</build_depend>
<run_depend>message_runtime</run_depend>
</pre>
　catkin이 message_generation Package를 찾는 것을 알 수 있도록 알려주어야 한다.
<br>
　CMakeLists.txt 파일에 하기 내용을 수정한다.
<pre>
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation // 다른 Package 다음 message_generation을 여기에 추가
)

catkin_package(
   CATKIN_DEPENDS message_runtime
   ....
)

add_message_files(
    FILES
    Complex.msg // 새롭게 정의할 .msg 파일
)

generate_messages(
   DEPENDENCIES
   std_msgs 
)
</pre>
## Topic 명령어
<pre>
// 현재 사용 가능한 Topic List 조회
rostopic list

// rostopic 명령어에서 사용할 수 있는 인자 
rostopic -h

// Publish Message 확인
rostopic hz [Topic]

// 광고되는 Topic 정보 확인
rostopic info [Topic]

// 해당 자료형을 발행하는 Topic List 조회
rostopic find [Message Type]

// 특정 Topic으로 Message 발행

</pre>
