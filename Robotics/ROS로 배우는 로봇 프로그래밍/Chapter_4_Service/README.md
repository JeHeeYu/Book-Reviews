# Service
## Service란
　Service는 ROS에서 node들 간 데이터를 전송하는 또 다른 방법이다.
<br>
　Service는 동기적인 원격 프로시저 호출을 뜻한다. 이는 한 node가 다른 node에서 실행되는 함수 호출을 가능하게 한다.
<br>
<br>
　Service를 제공하는 Server는 Service 요청을 다루는 콜백을 정의하고, Service를 광고한다.
<br>
　그 후 Service를 호출하는 Client는 지역 Proxy를 통하여 이 서비스를 사용한다.
<br>
<br>
　Service 호출은 <b>가끔 호출되거나, 제한된 시간이 필요한 일</b>에 적합한 기법이다.
<br>
　다른 컴퓨터로 배포하기 원할 때 사용하기 좋은 방법이다.
## Service 정의 파일(Service-Definition File)
　Service를 새로 생성할 때 첫 번째 단계는 Service 호출의 I/O를 정의하는 것이다.
<br>
　Service 파일은 .srv 파일로 이루어지며, 전통적으로 주 Package Directory 아래 srv Directory에 위치한다.
<br>
　Service 파일을 새로 생성하면 catkin build 작업이 필요하다.
<pre>
WordCount.srv

string words
---           // --- 기호는 입력의 끝과 출력 정의의 시작을 나타냄
uint32 count
</pre>
　Service 파일을 생성했으면, CMakeLists.txt와 package.xml에 Package 등록을 해주어야 한다.
<pre>
CMakeLists.txt

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
)

add_service_files(
   FILES
   WordCount.srv    // 새로 정의한 Service 파일
)

generate_messages(
   DEPENDENCIES
   std_msgs
)
</pre>
<pre>
package.xml

<build_depend>rospy</build_depend>
<exec_depend>rospy</exec_depend>
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>

</pre>
　Service를 새로 생성했을 경우, 추가한 Service의 클래스가 생성된다.
<br>
　이 예제에선 WordCount Service를 생성하여 WordCount, WordCountRequest, WordCountResponse 같은 세 개의 클래스가 생성되었다.
<br>
　이 클래스들은 Service를 사용하여 상호 작용할 때 사용된다.

## Service의 값 반환
　Service에 대한 반환하는 방법은 여러 가지 방법이 존재한다.
<br>
　Service에 대한 한 개의 반환 값이 있을 경우
<pre>
def count_words(request):
    return len(request.words.split())
</pre>
　다수의 반환 값이 있을 경우 튜플이나 리스트로 반환한다.
<br>
　리스트에 있는 값은 Service 정의에서 순서대로 값을 할당할 수 있으며, 한 개의 반환 값이 있을 경우에도 적용할 수 있다.
<pre>
def count_words(request):
    return [len(request.words.split())]
</pre>
　문자열로 주어진 딕셔너리를 반환할 수 있다.
<pre>
def count_words(request):
    return {'count': len(request.words.split())}
</pre>
## Service 명령어
<pre>
// Service 호출 정의 확인
rossrv show [Service Name]

// Service를 제공하는 패키지 조회
rossrv package [Package Name]

// 실행 중인 Service 조회
rosservice list

// Service에 관한 상세 정보 조회
rosservice info [Service Name]

// Service 사용
rosservice call word_count 'one two three'

</pre>
