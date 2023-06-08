# 주파수와 신호 정리 내용

## 신호와 주파수 영역 Spectrum Analysis
사전에서 주파수란 진동 운동에서 단위 시간당 같ㅌ은 것이 일어난 회수 또는 자주 일어남이라고 표현되어 있다.
<br>
영어로 Frequency 라고 한다.
<br>
<br>
아래 그림은 예를 들어 f=20KHz 일 때의 주파수이다.

<br>


![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/75b9c5cc-e568-4f98-96fd-bc57c619247a)

<br>

이 그림에서 cos(2π(20KHz)t)는 주파수 영역에서 20KHz 하나만의 성분을 갖는다.
<br>
<br>
보통 cos과 같이 주기를 갖는 신호를 AC 라고 하고, 주파수를 갖지 않는 동일한 크기의 레벨을 가지는 신호를 DC라고 부른다.
<br>
결국 주파수가 0Hz인 신호를 DC라고 할 수 있다.
<br>
DC는 같은 크기의 신호가 이어지기 때문에 아래 그림처럼 죽 이어지는 이미지이다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/e47f8024-85ad-4e6f-9c5d-40c9471fb195)

<br>

모든 신호는 무한개의 Cos 또는 Sin (주기 함수)의 합으로 나타낼 수 있다는 부분이 주요한 부분이다.
<br>
결국 신호를 주파수 별로 분리해 준다는 얘기이다.
<br>
<br>
어떤 신호 = 어떤 주기의 주파수 성분 + 또 어떤 주기의 주파수 성분 + 또 또 어떤 주기의 주파수 성분 ... 와 같은 개념이다.
<br>
각각의 어떤 주기의 주파수 정수 배항들이 어떤 신호를 이루기 위하여 공헌하는 바는 틀리다.
<br>
그 공헌하는 바들이 각각의 크기로 나타난다.
<br>
<br>
즉, 그 공헌하는 바를 어떤 주파수의 크기라고 할 때,
<br>
어떤 신호 = 어떤 주기의 주파수 크기 * 어떤 주기의 주파수 * 또 어떤 주기의 주파수 크기 * 또 어떤 주기의 주파수 ... 이다.

<br>

## Analog 신호와 Digital 신호, 그리고 Ground
Analog 신호는 연속된 값의 신호이고, Digital 신호는 Boolean Logic의 값으로, 1과 0만 있다.
<br>
Digital 신호는 Analog 신호의 일종이며, 대부분 DC 성분으로 이루어져 있다.
<br>
<br>
Analog 신호는 보통 AC와 DC(교류와 직류) 성분으로 이루어져 있다.
<br>
교류, 즉 AC는 주파수를 가지기만 하면 AC라고 불린다.
<br>
<br>
정리하자면 극성이 바뀌는 신호를 교류 신호, 극성이 바뀌지 않고 Steady한 상태의 신호를 DC라고 부른다.
<br>
아래의 주파수처럼 요동치는 모양의 주파수도 여러 개의 주파수 성분을 더해서 만들어 낼 수 있다.

<br>


![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/a2cd7b46-288a-4d62-973d-756e324728ac)


<br>


Digital 신호는 High와 Low 두 개의 Logic Value 만을 가질 수 있으며, 그 크기(Level)은 정하기 나름이다.
<br>
결국은 신호가 있느냐, 없느냐의 문제인데 그 신호가 있는냐 없느냐는 원래 DC 성분만을 가지고 따진다.
<br>
<br>
예를 들어 1V High, 0V Low라고 약속한다면 1V, 0V 두 가지 DC Level만 가지고 Digital 신호를 만든다.
<br>
또 다르게 2V High, -2V Low라고 약속한다면 그 역시 두 가지 DC Level만 있는 것을 Digital 신호 시스템이라고 볼 수 있다.
<br>
<br>
그리고 Digital 신호에는 Bounce 라는 개념이 있다.
<br>
Bounce는 Digital 신호가 0V -> 1V 또는 1V -> 0V와 같이 Logic Value가 변할 때 파형이 흔들리는 현상이다.
<br>
<br>
Digital 신호에는 이상적인 신호가 있으면 좋겠지만, 현실은 이상적인 신호가 없어 Logic이 변할 때 파형이 요동치게 된다.
<br>
<br>
이렇게 Bounce라는 현상이 발생하면서 이런 Transition에 관련하여, 이런 요동이 얼마나 크냐에 따라 시스템에 영향을 줄 것인지 아닌지를 판가름하기도 한다.
<br>
이런 의미에서 보면 디지털 신호는 DC 성분 뿐 아니라 AC 성분 까지도 여전히 포함하고 있다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/e7e64b19-89f7-406c-9225-b1ef16a17322)

<br>

이런 현상은 전원 Line(Power)에서도 자주 볼 수 있다.
<br>
System의 어떤 Chip이 동작을 시작하는 시점에서 갑자기 전류를 더 끌어 사용할 수 있어, Bounce되는 것 처럼 보이기도 한다.
<br>
이런 경우, 회로는 전력 문제에 직면할 수 있다.
<br>
<br>
최악의 경우 System이 멈출 수도 있다.
<br>
게다가 이런 Bouncing이 있으면 Digital 신호의 Level 인식 문제가 발생할 수 도 있다.
<br>
<br>
즉, 1V인데 0V로, 0V인데 1V로 인식할 수도 있다.
<br>
<br>
만약 전원에서 Bouncing 되는 부분이 있다면 회로적으로 상당히 불안하다고 봐야 한다.
<br>
이럴 때는 Capacitor를 한쪽은 Power 선에, 다른 한쪽은 GND에(즉, 병렬) 달아 Capacitor를 마치 건전지 처럼 사용하여, Power Line의 전압, 전류가 순간적으로 낮아질 때 Capacitor가 저장하고 있던 전기 에너지를 다시 방출하여 Power Line의 전압을 원래대로 유지하는 용도로 믄제를 해결하기도 한다.
<br>
<br>
그리고 중요한 개념으로 High Impedance 상태 개념이 있다.
<br>
Hardware에서는 이를 High Impedance라고 부르 Software 에서는 Floating 이라고 부른다.
<br>
<br>
High Impedance는 어떤 Digital Chip Pin이 내부에서 저항이 무한대가 달린 것과 마찬가지로 된 상태이다.
<br>
다른 Digital Chip과 연결이 되어 있다면 그 Chip 내에서는 이 Pin이 보이지 않는 것과 마찬가지 상태가 되어 버린다.
<br>
<br>
만일 이런 High Impedance Pin과 여러 개의 Pin이 공통 버스로 연결되어 있다면, High Impedance 상태의 Pin들은 다른 Chip들에 의해서 유기 되어진 버스상의 값들의 변화(Transition)에 관여하지 않게 된다.
<br>
<br>
이런 경우라면 HighImpedance 상태의 Pin에서 본다면 무슨 값이 될지 몰라 Floating 되었다고도 볼 수 있다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/2f366c23-353e-4122-99db-fff4917171d6)


<br>


마지막으로 중요한 GND 라는 개념이 있다.
<br>
Ground라고 표기하며, 모든 전기 전자 회로에서 다른 모든 전위에 대하여 기준이 되는 V를 말하며, 일반적으로는 전지의 - 극을 의미하기도 한다.
<br>
GNS는 System 내부에서 모 Current가 몰려드는 곳이다.
<br>
<br>
이 GND 중요한 이유는 Digital 신호의 1과 0을 제대로 구분하는 기준점이 되기 때문이다.
<br>
<br>
GND의 Symbol은 마치 땅(Earth)에 연결된 것 처럼 표시하며, 이 Symbol에 연결 되어 있는 회로 상의 모든 Point는 0V로 같다는 것을 의미한다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/5cee5e6e-e556-44f2-8cec-0d86b1fb1dd5)


<br>

Ground는 Earth와 Signal Ground로 나뉘게 되며, 이때 Earth는 실제 지구를 의미한다.
<br>
이 Earth는 실제 전위 0V를 의미하지는 않으나, 저항이 크지 않아 왠만한 전류는 모두 빨아 들인다.
<br>
<br>
그래서 어떤 커다란 전기기기의 GND는 직접 땅에 연결 하는 것이다.
<br>
<br>
반대로 Signal Ground는 Hardware 측면에서 어떤 전자기기의 GND를 의미하게 된다.
<br>
이 GND는 System 내부의 모든 전위에 대해서 기준점이 되는 점을 말한다.
<br>
<br>
보통은 전지의 --극을 저항이 적은 넓은 Case나 PCB 기판의 뒷면등에 연결하여 전류가 몰려들 수 있는 환경을 만들어 놓은 후, 전위 기준으로 사용한다.
