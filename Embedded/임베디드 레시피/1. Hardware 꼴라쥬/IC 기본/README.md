# IC 기본 정리 내용

## IC(Integrated Circuit)
IC는 우리 말로 직접 회로라고도 말한다.
<br>
IC는 Digital 논리회로들을 하나의 패키지로 만든 칩을 IC라고 한다.
<br>
<br>
우선 IC는 IC를 보고 다리 번호 정도는 볼 줄 알아야 한다.
<br>
IC를 보면 아래와 같이 마크가 되어 있고, 자리 표시가 되어 있는 부분을 왼쪽에 놓고 반시계 방향으로 읽어 나가면 된다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/1443a2b4-22c1-4cb0-b621-2b7d7797f535)

<br>

그리고 IC의 Specification을 보다 보면 아래 그림과 같이 설명되어 있을 때가 많이 있다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/847fbafb-a371-453c-b415-2c2d01e6ba71)

<br>

1번 부터 8번 핀까지 ㅏㄱ 핀에 대한 설명이 대충 나와 있다.
<br>
여기서 U301이라고 쓰여져 있는 부분은 대부분의 하드웨어 엔지니어들은 반도체 IC를 U라고 Naming해서 회로도에 표기한다.
<br>
<br>
그리고 각 핀에 대한 설명은 다음과 같다.
- NC(1, 2, 6) : Not Connection으로서 아무것도 연결하지 말라는 의미로 이런 핀들을 어떻게 처리해야 하는지 Device Spec을 잘 확인해야 함
- CLK(3) : Edge Trigger를 의미하며 Clock이 High 또는 Low일 때 동작하는 것을 의미
- GND(4) : Ground
- VCC(5) : 전원
- Data(7, 8) 데이터의 In/Out
