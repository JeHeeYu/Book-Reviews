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

이 회로에서는 +3V_VCC가 전원이며, 이 전원이 D_CHIP의 VCC_IN에 들어와 칩이 동작하도록 한다.
<br>
이 회로 중 L은 AC에게 큰 저항, DC에게는 저항 없음이며, C는 AC에게 저항 없음이며 DC에게는 큰 저항이다.
<br>
<br>
즉, +3_VCC의 DC 성분을 안정적으로 VCC_IN에 안정적으로 공급하기 위한 것이다.
<br>
<br>
L 대신 R 을 넣고 사용해도 되는 의문이 들 수도 있는데, L을 넣는 경우는 보통 그냥 코일이 아닌 Bead라고 부르는 Inductor의 한 종류를 삽입한다.
<br>
이 Bead의 특성이 특정 주파수에서의 저항을 역할을 하여 마치 공진기 처럼 특정 주파수를 없애 버리는 역할도 같이 한다.
<br>
<br>
결국 R을 넣으면 전체적으로 줄어 들긴 하나 특정 주파수를 제거하지는 못하지만 AC를 전체적으로 죽이면서도 특정 주파수를 죽여버릴 수 있는 장점이 있다.
<br>
<br>
이런 회로는 통신 회로에 많이 사용되는데, 통신 회로 중 통신을 하는 Carrier 주파수를 없앨 수 있는 장점이 있다.
<br>
이런 Bead를 삽입해서 그 주파수가 보드에 발생하지 못하도록 한다.
<br>
<br>
앞의 2개 예제에서 사용되는 Capacitor들을 Bypass Capacitor 또는 Decoupling Capacitor라고 부른다.
<br>
Bypass Capacitor는 Noise, Ripple 성분을 주파수 특성을 이용하여 GND로 통과시켜 버린다는 의미이고, 또 Decoupling Capacitor는 Noise 또는 Ripple 성분들 전원 공급에서 뗴어낸 다는 것을 의미한다.

<br>

## USB / TR / C Control

예를 들어 아래와 같은 회로가 있다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/a06aac63-3d62-4397-b432-46bcbc815732)

<br>
이 회로는 Chip 1에 Control 신호를 넣으면 USB_CONN을 구동하는 회로이다.
<br>
즉, 다른 어떤 칩으로부터 Control High Drive를 받으면 Chip 1은 5번 핀에 High Drice를,
<br>
이후 Transistor가 On이 되고, USB_CONN이 Low Drive로 Active 된다.

<br>
<br>

여기서 R, C 는 다음과 같은 의미로 사용된다.
- C : 컨트롤 신호가 Chip 1에 입력될 때 AC 성분이 들어가지 않도록 AC를 GND로 흘려 보내는 역할을 함
- R : USB_CONN이 low Active 이므로 평상시 Default 상태를 High로 만들기 위한 +3V_VCC와 함께 Pull up을 하기 위한 역할

<br>

## R, TR
예를 들어 아래와 같은 회로가 있다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/612e650f-ee02-4a49-bc5e-f0b5e55caf33)

<br>

이 회로는 NPN형 T1, PNP형 T2 트랜지스터가 맞물려 있다.
<br>
여기서 input으로는 +INPUT, CONTROL/가 있고, Output이 오른쪽 상단에 Out Port로 나와 있다.
<br>
<br>
일단 이 회로는 DC 회로이며, CONTROL/에 의해서 Control 된다고 볼 수 있다.
<br>
그리고 R1과 +INPUT 신호를 이용한 Pull up이다.
<br>
<br>
일단 CONTROL/가 High 일 때 T1이 ON되며, +INPUT은 3번 포트와 5번 퐅르ㅡㄹ 따라 전압을 소진하게 되면서 동시에 T2가 On이 된다.
<br>
<br>
5번 포트는 T1이 On됨에 의하여 1번 포트의 GND에 연결되니까 그런 것이다.
<br>
그러므로 OUT으로 연결되는 T2가 On 되었으니까 Out에 Input이 대부분 걸리게 되는 것이다.
<br>
<br>
이번엔 반대 케이스인 CONTROL/의 신호가 Low인 경우는 CONTROL/이 Low가 되면서 T1이 Off가 된다.
<br>
그리고 T2도 같이 off가 되면서 결국 +INPUT 신호는 오갈 곳 없이 OUT에는 아무 것도 걸리지 않는 것이다.
<br>
<br>
결국 CONTROL/ 신호가 Low 일 때 Output에는 아무것도 걸리지 않고 CONTROL/ 신호가 High 일 때는 +INPUT신호가 OUT으로 나가게 되는 회로가 된다.
