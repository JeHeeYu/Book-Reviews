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
