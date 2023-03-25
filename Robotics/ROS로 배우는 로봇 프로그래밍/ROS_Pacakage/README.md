# ROS Package 모음
## roscore
　roscore란 node들의 연결 정보를 제공하여 서로 메시지를 전송할 수 있도록 하는 서비스
<br>
　node들은 시작 시 roscore와 연결되며, Publish or Subscribe 할 Message Stream에 세부 사항을 등록한다.
<br>
　roscore 없이 각 node들 끼리 연결될 수 없으며, 모든 ROS System에서 roscore의 실행을 필요로 함
<br>
<ul>
<li>ROS에서 node들 간의 메시지는 P2P로 전송됨</li>
<li>roscore의 역할은 node들 간의 node의 위치를 알려주는 역할만을 위해 사용함</li>
<li>node들은 roscore에게 원하는 Publish Message와 Subscribe List를 전달함</li>
<li>ROS node는 시작 시 ROS_MASTER_URI라는 환경 변수룰 찾는데, 이 변수는 http://hostname:11311와 같은 형식의 문자열을 갖으며,
<br>해당의 경우 hostname이라는 host에서 11311 port로 접근 가능함을 뜻함</li>
</ul>

## catkin
　catkin이란 ROS가 실행할 Program, Library, Script 및 기타 Code가 사용할 Interface를 생성하는
<br>
　도구 집합인 ROS 빌드 시스템이다.
 catkin은 CMake에서 추가정긴 기능을 제공하기 위한 Python Script로 구성되어 있다.
### catkin_ws
　catkin_ws는 ROS를 실행할 수 있는 작업 공간으로, 관련된 ROS Directory들의 집합이다.
<br>
　즉, 한 번에 하나의 작업 공간에서만 작업할 수 있으며, 현재 작업 공간에 있는 코드만 볼 수 있다는 의미
<br>
<br>
　catkin 작업 공간은 <b>catkin_ws/src</b>로 구성되어 있다.
<pre>
source /opt/ros/melodic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace

cd ~/catkin_ws
catkin build
</pre>
　catkin build는 src 폴더가 아니 부모 폴더 catkin_ws에서 실행해야 한다.
<br>
　catkin build는 catkin 작업 공간에 있는 파일들을 빌드하는 것으로,
<br>
　build 및 devel Directory를 생성한다.
<br>
　　※ cd ~/catkin_ws/devel/setup.bash && /opt/ros/melodic/setup.bash 를 실행해야 ROS가 정상적으로 동작함
<br>　　　　alias를 등록해두거나 ~/.bashrc 파일에 source ~/catkin_ws/devel/setup.bash와 같이 추가하면 편리함
## ROS Package
　ROS Package들은 catkin_ws 내부 src에 둔다. 각 Package Directory들은 CMakeLists.txt 파일과
<br>　Package 내용간 catkin이 어떻게 연동되어야 하는지 설명하는 <b>package.xml 파일을 포함해야 한다</b> 
### New Create Package
<pre>
cd ~/catkin_ws/src
catkin_create_pkg basics rospy
cd ~/catkin_ws/src/basics
mkdir scripts && cd scripts
vi topic_publisher.py
vi topic_subscriber.py
cd ~/catkin_ws && catkin build
rosrun basics topic_pubilsher.py
rosrun basics topic_subscriber.py
</pre>
## rosrun
　rosrun package는 ROS의 기본적인 실행 명령어이다. ROS에 요청된 프로그램에 대한 Package를 검색하고, 검색된 Package들을
<br>
　자동으로 실행해주는 유틸리티 개념이다. 
<pre>
rosrun rospy_tutorials talker
rosrun rospy_tutorials listener
</pre>
 위와 같이 실행할 프로그램 앞에 rosrun을 붙여 사용한다.
## roslaunch
 roslaunch란 여러 개의 ROS node 시작을 자동화 할 수 있도록 설계된 명령행 도구이다.
<br>
 roslaunch는 노드를 사용하여 동작하는 것이 아닌, 시작 파일(launch file)을 사용하여 동작한다.
<br>
 시작 파일은 여러 node들의 대한 topic과 매개변수가 기록된 XML 파일이다.
<br>
 관례적으로 이 파일은 <b>.launch 확장자</b>를 가진다.
<br>
 roslaunch는 ssh를 통하여 네트워크상 다른 컴퓨터에 프로그램을 시작하거나 종료된 노드를 재시작 등 중요한 특징이 있다.
