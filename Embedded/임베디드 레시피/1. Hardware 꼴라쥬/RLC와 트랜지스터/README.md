# RLC와 Transistor 정리 내용

## Example C 회로
예를 들어 아래와 같은 회로가 있다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/cf8a9f94-ef5e-416d-9f98-5636f28fa3b4)


<br>

가장 간단하게, 칩에 3V DC 전원을 인가하기 위한 회로이다.
<br>
만약 여기서 C의 용도는 어떤 것일까?
<br>
<br>
C의 용도는 AC인 고주파만 통과시키고, 3V 전원이 인가될 때 AC 성분을 GND로 흘려 칩에 들어가는 것일 차단하기 위한 Capacitor이다.
<br>
<br>
여기서 AC는 전원 Noise 따위이다.
<br>
또는 전원이 켜질 때 Transition으로 생기는 Noise 등을 제거하기 위한 것이다.
<br>
이것들을 Ripple 제거라고도 하는데, Ripple이란 작은 양의 DC의 흔들림(AC성분)을 말한다.

<br>

## Example L, C 회로
예를 들어 아래와 같은 회로가 있다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/42b1c702-cfe4-42e5-af91-51e99ba31f0c)


<br>

이 회로에서는 +3V_VCC가 전원이며, 이 전원이 D_CHIP의 VCC_IN에 들어와 칩이 




