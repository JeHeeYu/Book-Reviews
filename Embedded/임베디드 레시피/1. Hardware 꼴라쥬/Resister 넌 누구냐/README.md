# Register 넌 누구냐 정리 내용

## 레지스터(Register)
레지스터는 플립 플롭(Flip Flop)의 집합이며 이 플립 플롭이라는 것은 각각 1bit의 정보를 저장할 수 있는 것들을 의미한다.
<br>
결국 n-bit Register라는 것은 n bit의 정보를 저장할 수 있는, 즉 n개의 플립 플롭으로 이루어진 플립 플롭 그룹을 말한다.
<br>
이런 의미에서 레지스터라는 것은 최소 1bit 단위로 정보를 저자 ㅇ또는 수정할 수 있는 것을 말한다.
<br>
<br>
가끔 회로에서 래치(Latch)라는 말을 하는데, 래치란 1bit 즉 1 또는 0인지를 기억할 수 있는 소자를 통칭하는 말이다.
<br>
그 중 가장 대표적인 예가 플립 플롭이며, 플립 플롭은 속도가 빨라서 레지스터의 구현에 가장 많이 사용된다.
<br>
<br>
그리고 레지스터는 굳이 CPU만 연산 중간이나 결과에 사용한다고 볼 수는 없다.
<br>
CPU 내부에는 메모리 대신에 메모리보다 더 빠른 레지스터가 쓰이는 정도라고 이해해야 한다.
<br>
그렇게 이해하고 있으면 CPU 외부에 레지스터는 없다고 생각을 가둘 수도 있으니 조금 위험한 생각이다.
<br>
CPU 내부 말고도 그 외부에 레지스터를 읽을 수가 있기 때문이다.
<br>
<br>
때에 따라서는 메모리도 이런 레지스터의 모음으로 구성할 때도 있다.
<br>
하지만 용도에 따라 다르기 때문에 CPU만 레지스터를 사용할 수 있다는 생각은 버려야 한다.
<br>
<br>
추가로 n bit Register는 n 개의 래치로 이루어 지며 이런 레지스터들의 Output에는 간단한 Data-Processing Operation을 할 수 있는 Gate Combination이 덧붙여 질 수도 있다.
<br>
<br>
이 의미는 어떤 특정한 레지스터는 특정한 Task를 수행하기 위해서 존재할 수도 있으며, 이 특정한 레지스터에 약속된 특정 값을 Write 함 으로써 특정한 일을 시킬 수도 있다는 것을 의미한다.
<br>
<br>
이런 특징 때문에 레지스터를 이용한 Memory Mapped I/O의 구현이 가능해진다.
<br>
<br>
레지스터의 동작을 설명하기 위해서는 플립 플롭이라는 논리 회로를 이해 해야 한다.
<br>
Flip의 뜻은 프라이팬을 뒤집다, Flop의 뜻은 벌렁 드러눕다 라는 의미이다.
<br>
뜻의 의미에 맞게 플립 플롭은 드러누웠다가 뒤집혔다가 다시 드러누웠다 하는 회로이다.
<br>
<br>
가장 간단한 형태의 플리 픕ㄹ롭이란느 것은 실은 NOR Gate 두 개의 아웃풋을 서로 인풋으로 다시 피드백하는 형태로 생겼다.
<br>
또는 R-S(Reset Set) F/F라고 부르기도 한다.
<br>
<br>
이것을 표현한 것이 아래 그림이다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/086365ac-7136-4fc7-b3e0-a9109c064be0)


<br>

위 그림을 보면 Data In은 Data Input이고 Write는 Write Enable이라고 해서 Write가 TRUE가 되어야 Data In을 위쪽 AND Gate나 아래쪽 And Gate로 넣을 수가 있다.
<br>
<br>
Write 신호가 항상 TRUE라는 가정 하에 R은 Reset이고 S는 Set이라는 이름을 살펴 본다면
<br>
R = 1 이고 S = 0 이면 R쪽의 NOR Gate의 Ouptut은 무조건 0이 되고,
<br>
R = 0 이고 S = 1 이면 R쪽의 NOR Gate Output은 무조건 1이 된다.
<br>
<br>
결국 R은 Data Out에 대한 Reset, S는 Data Output에 대한 Set이라고 볼 수 있으며 R과 S는 같은 값을 가져서는 안된다.
<br>
이런 식이라면 R과 S만 잘 컨트롤 하면 Data Out에 뭔가를 넣을 수 있다.
<br>
<br>
결국 Data In은 S와 R에 들어갈 데이터니까 S쪽은 데이터 유지를, R쪽에는 데이터를 Invert해서 넣으면 된다.
<br>
더 중요한 점은 이런 플립 플롭 회로는 다음 Input이 들어올 때 까지 Data Out을 유지하고 있다는 점이다.
<br>
결국 Memory 기능을 가지고 있다는 뜻이다.
<br>
<br>
이런 플립 플롭 회로를 RS 플립 플롭이라고 부른다.
<br>
<br>
다음 Input이라는 것은 Write 신호가 TRUE가 된 경우에만 Data In이 입력이 되고 D0에 기억하고 있는다.
<br>
그래서 이런 1bit를 저장할 수 있는 플립 플롭 회로를 다음과 같이 한 개의 기호로 나타내기 시작했다.
<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/49e09b20-ddb4-4608-b9ed-f15467dedcbd)

<br>

위 그림을 보면 DI는 Data In, DO는 Data Out, W는 Write Enable 신호이다.
<br>
이런 래치를 이용하여 뭔가를 잠시 저장하는데, 이때 래치들을 이용한다.
<br>
DO에 뭔가 Data Processing 회로를 달아서 DO가 TRUE가 되는 순간 뭔가 일을 할수 있도록 할 수도 있다.
<br>
<br>
이런걸 일컬어 Level Trigger Latch 라고 부른다.
<br>
W가 High를 유지할 때 Write가 가능하고 그 외에는 값을 기억하고 있다는 뜻이다.
<br>
<br>
굳이 Level Trigger라고 부르는 이유는 Edge Trigger D Type Latch라는 것도 있어서 그렇다.
<br>
이 Level Trigger Latch는 Write 신호가 High로 올라가 있는 동안 입력된 Data(DI)를 기억해 준다.
<br>
<br>
그리고 Edge Trigger는 RS F/F 두 개를 묶어서 만드는데 그 구조도 있다.
<br>
<br>
즉 정리하자면 Edge Trigger Latch는 CLK가 올라가는 순간 또는 내려오는 순간에만 Write가 가능하고 그것이 아닌 순간에는 그 전에 Write된 값을 기억한다.
<br>
<br>
이런 플립 플롭은 아래와 같이 표현한다.
<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/3b55818a-915c-4e6f-96f1-9558239e49d6)

<br>

흔히 Level Trigger 플립 플롭은 Latch, Edge Trigger 플립 플롭을 그냥 플립 플롭이라고 부르는 경향이 있다.
<br>
또 어떠할 때는 Latch는 클럭에 따른 동기화가 없는 부분, 플립 플롭은 클럭을 넣어서 동기화하는 부분 등 이렇게 나누기도 한다.
<br>
<br>
그리고 CLK 부분의 작은 삼각형은 Edge Trigger 방식이라는 뜻이다.
<br>
CLK이 0에서 1로 변할 때 값을 Write 하는 경우에는 Rising Edge Trigger라고 부르고, CLK이 1에서 0을 변할 때 Write 하는 경우에는 Falling Edge Trigger라고 부른다.
<br>
<br>
Rising Edge Trigger는 아래 처럼 클럭이 0에서 1로 올라갈 때의 아래 그림과 같이 Default Signal을 Input으로 받아 들여 데이터를 저장한다. (Capture 라고 함)
<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/21486332-49c8-4d3c-a482-7a76afa08d2f)

<br>
이런 1bit 래치를 엮어 n-bit 래치를 만들기도 하는데, 그런건 아래 그림과 같이 여러 개의 1bit 래치를 엮어서 만들어 낸다.
<br>
아래 그림과 같이 I0, I1, I2, I3은 모두 Input 데이터이며, A0, A1, A2, A3은 Output 데이터, Write 신호가 Edge일 경우에만 Input 데이터로 Output들이 업데이트 되며 그 내용을 유지하고 있는다.
<br>
<br>
레지스터란 이런 래치 여러 개 또는 플립 플롭 여러 개를 엮어서 n bit로 만든 것을 말한다.
<br>
그러니까 아래는 Write 신호에 대해 Edge Triggering 되는 4bit 래치 또는 4bit 레지스터라고 보면 된다.
<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/87ace260-705a-49f7-921a-4f54bfbec9de)

<br>

위와 같은 구조를 아래 그림처럼 하나의 래치로 표현하기도 한다.
<br>
역시 W에 삼각형 표시가 있으니 Edge Trigger 방식이다.
<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/bcf7d756-c6e7-419e-9a74-4a8c812e31fa)

<br>
이런 레지스터의 저장 특성을 이용하여 MCP/CPU 내부에서 여러 가지 용도로 이용한다.
<br>
I/O로 이용하기도 하고 CPU Core 내의 임시 저장 공간 또는 CPU의 레지스터 File로 이용하기도 한다.
<br>
<br>
이런 의미에서 레지스터는 크게 두 가지 종류로 나눌 수 있다.
<br>
General Purpose Register와 Special Purpose Register 이다.

### General Purpose Register
- Address Register : 외부 메모리에 쓰거나 읽을 때 데이터가 들어 있는 주소를 가리키는 값을 넣어두는 레지스터
- Data Register : 외부 메모리에서 읽어온 값을 임시 저장하는 레지스터로, 자세히는 Data Input Register, Data Output Register로 구성됨
- Instruction Pipline Register : 외부 메모리에서 읽어온 OP-Code(Operation Code)를 저장하는 레지스터

### Special Purpose Register
- Program Counter : 현재 실행되고 있는 주소를 가리키는 레지스터
- Stack Pointer : 현재 사용하고 있는 스택 영역에서 마지막에 데이터가 푸시된 곳을 가리키는 레지스터
- Linked Register : 방금까지 수행하다가 Jump 했을 경우 돌아갈 곳의 주소를 가리키는 레지스터
- Status Register : MCU의 현재 상태를 나타내는 레지스터로 현재 모드 또는 계산 결과 상태 등을 저장
