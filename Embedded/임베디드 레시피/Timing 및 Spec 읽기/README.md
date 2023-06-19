![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/ecaf188c-9b7f-44be-8d7b-81f48cd58fab)# Timing 및 Spec 읽기 정리 내용

## 타이밍(Timing)
타이밍을 제대로 확인하기 위해서는 스위치 특성을 이해해야 하는데, 게이트에서는 입력 신호가 들어온 후 출력 신호가 나오기까지 약간의 시간이 걸린다.
<br>
이러한 시간을 전달 지연 시간이라고 부른다.
<br>
그리고 전달 지연 시간에 관한 전기적 특성을 스위치 특성이라고 부른다.
<br>
<br>
예를 들어 펄스 하나를 게이트 IC에 입력하는 경우 아래 그림과 같이 펄스는 실제 타이밍 차트상에서 완전한 사각형 모양이 아닌 사다리꼴 모양이 된다.
<br>
디지털 신호가 이상적으로 동작하지 않으며, 신호가 스위칭될 때 약간의 시간이 걸리기 때문이다.
<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/f8e527de-4655-4cf8-922b-8d2385610d93)

<br>

위와 같이 완전한 신호 High일 때 전압을 기준으로 10%에서 90%로 올리기까지 필요한 시간을 상승시간 이라고 한다.
<br>
반대로 90%에서 10%로 내려가기까지 필요한 시간을 하강시간 이라고 한다. 
<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/a3e7f885-b8be-44eb-8bbe-8942ea89d339)

<br>

그리고 Timing에 대한 표기법은 다음과 같은 표기법이 일반적이다.
<br>
색깔이 어두운 부분은 Undetermistic 한 부분이며, 어떤 값이든 가질 수 있고 실제 동작에 있어서 아무런 영향을 미치지 않는다.
<br>
High 또는 Low를 가질 수 있는 영역이라고 볼 수 있다.
<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/8d3c4324-3272-4309-8a16-08d694c113a9)

<br>

클럭은 말 그대로 클럭이고 모양도 주기성을 타는 것 처럼 보인다.
<br>
<br>
<br>
예를 들어 아래와 같은 Device의 타이밍 다이어그램이 있다.
<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/76d61d47-5cd4-4719-a1f8-a861e8b359bf)

<br>
이런 타이밍 다이어그램은 중요한데, 예를 들어 tCE라는걸 해석해 본다면 CE/라는 신호가 Low로 떨어지고 나서 다시 Output Data가 나올 때 까지의 시간이며 이 시간은 지켜져야 한다는 것을  말하는 것이다.
<br>
<br>
이런 의미에서 타이밍 다이어그램에는 그 의미를 해석한 Tming Spec이 Data Sheet에 꼭 명시되어 있다.
<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/15c69cae-5d8f-4424-b5de-e050e1fa2541)

<br>

위 테이블이 바로 타이밍 다이어그램이며, tCE는 CE/ Access Time이고 최대 65ns를 넘어서면 안된다는 의미이다.
<br>
<br>
추가로 tRC는 Address Line에 흐르는 신호에 대한 타이밍 스펙으로서 그 의미는 Read Cycle Time이며 최소 65ns를 유지해야 제대로 된 데이터를 얻을 수 있다는 의미이다.
