# 스터디 진행하면서 발생했던 오류 모음
## raise DistributionNotFound(req, requirers) pkg_resources.DistributionNotFound: The 'catkin-pkg==0.5.2' distribution was not found and is required by the application
　rospkg의 version이 맞지 않아 발생한 것으로 보임
<pre>
pip3 install rospkg==1.2.8
</pre>
## catkin_create_pkg: error: the following arguments are required: --rosdistro
　rospkg create 시 arguments 오류로 발생
<pre>
// 명령어 뒤 --rosdistro [ROS Version] 입력
catkin_create_pkg test_code rospy --rosdistro melodic
</pre>
## The build space at '/home/jhy/catkin_ws/build' was previously built by 'catkin build'. Please remove the build space or pick a different build space.
　catkin_ws make 시 오류로, catkin workspace가 비어있지 않아 발생
<pre>
//catkin_workspace 비우기 (build파일, devel파일 삭제)
catkin clean -y 

//다시 build하기 - catkin_make 보다 catkin build가 troubleshooting에 용이
catkin_make 
</pre>
## RLException: roscore cannot run as another roscore/master is already running. 
Please kill other roscore/master processes before relaunching.
　실행 중인 roscore Process가 있어서 발생
 <pre>
 killall -9 rosmaster
 </pre>
## [rospack] Error: package 'count' not found
　ros package를 찾지 못할 때
 <pre>
 ~/catkin_ws/devel/setup.bash
 rospack profile
 rospack find [packge_name]
 </pre>
## import-im6.q16: not authorized error 'os' @ error/constitue.c/WriteImage/1037 for a Python web scraper
　Python 쉬뱅이 첫 줄에 없어서 발생
<pre>
#!/usr/bin/env python3
</pre>
## The manifest of package "basics" (with format version 2) must not contain the following tags: run_depend
Please replace <run_depend> tags with <exec_depend> tags.
　package.xml 내 <run_depend>가 더 이상 사용되지 않고 <exec_depend>로 대체되어 발생
<pre>
package.xml

<package format="2">
<exec_depend></exec_depend>
</pre>
## catkin_package() the catkin package 'std_msgs' has been find_package()-ed but is not listed as a build dependency in the package.xml
　Action 정의 시 package.xml 파일 내 의존성 누락으로 인해 발생
<pre>
package.xml

....
<buildtool_depend>catkin</buildtool_depend>
<build_depend>actionlib_msgs</build_depend>
<build_depend>rospy</build_depend>
<build_depend>std_msgs</build_depend>
<build_export_depend>actionlib_msgs</build_export_depend>
<build_export_depend>rospy</build_export_depend>
<build_export_depend>std_msgs</build_export_depend>
<exec_depend>actionlib_msgs</exec_depend>
<exec_depend>rospy</exec_depend>
<exec_depend>std_msgs</exec_depend>
</pre>
## ImportError: "from catkin_pkg.topological_order import topological_order" failed: No module named 'catkin_pkg'
Make sure that you have installed "catkin_pkg", it is up to date and on the PYTHONPATH.
　catkin_pkg 미설치로 인해 catkin_make 시 발생 
<pre>
sudo pip install rospkg catkin_pkg
</pre>
##ModuleNotFoundError: No module named 'netifaces'
　ros package 실행 시 python 버전으로 인해 발생
<pre>
update-alternatives --config python
python 2.7 선택 (1
</pre>
RLException: Invalid <arg> tag: environment variable 'TURTLEBOT3_MODEL' is not set. 
　터틀봇 모델 환경변수 
<pre>
export TURTLEBOT3_MODEL=burger
</pre>
