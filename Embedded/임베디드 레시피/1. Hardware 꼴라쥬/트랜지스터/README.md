# 트랜지스터 정리 내용

## 트랜지스터(Transistor)
Amplifier, 증폭기 등 이런 용어들이 TR(트랜지스터)을 처음 접할 때 나오곤 한다.
<br>
Transistor는 Trans-Resistor이다.
<br>
즉, Resistor의 값을 변화 시킬 수 있다는 의미이다.
<br>
<br>
Resistor는 전류의 양을 조절하는 것인데, 그럼 Transistor는 전류의 양을 마음껏 조절할 수 있다는 의미인 것이다.
<br>
<br>
일단 TR은 NPN형과 PNP형이 있으며, Symbol은 아래와 같이 표기한다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/69aff51a-da59-4dd5-8fc3-f40abe4d63f2)


<br>

이 트랜지 스터는 물리적으로 구조가 다르다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/2fb61d25-6851-4213-86e2-74ffb5ba6061)

<br>

그리고 이것을 회로에 직접 그려보면 아래와 같다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/71618ca5-38b5-411f-8d82-50346b9fdece)

<br>

npn형 TR의 Trans-Resistor로서의 기본 형태는 위의 그림과 같다.
<br>
여기서 TR의 B는 Base, E는 Emitter, C는 Collector라고 부른다.
<br>
<br>
B에 의해 Switch가 ON되면 C와 E사이에 전류가 흐르게 된다.
<br>
<br>
그리고 Resistor로서의 의미로, Switch를 더 세게 누르면 C와 E 사이에 전류가 더 많이 흐르고 약하게 누르면 전류가 덜 흐른다고 가정할 수 있다.
<br>
즉, B를 어떻게 누르느냐에 따라 C와 E 사이에 흐르는 전류량이 바뀐다는 것이고, 바로 저항값이 변한다고 볼 수 있다.
<br>
<br>
그럼, 그 Switch를 세게 누르는 것이 무엇이냐 햐면, B와 E 사이에 전압을 얼마나 세게 주느냐의 문제라고 한다면, 그것이 바로 Transistor의 Technology이다.
<br>
TR은 평소에는 전류가 흐르지 못하다가 화살표 방향으로 전압을 넣어 주면 전압의 양을 얼마나 넣어줄 것이냐에 따라 C와 E사이의 전류량을 결정할 수 있다.
<br>
<br>
가장 중요한 한 가지는 B에 넣어주는 전압(전류)랴의 미묘한 변화에도 저항값이 크게 바뀐다는 것이다.
<br>
결국 E - C 간의 전류 값도 크게 변한다.
<br>
<br>
이 Switch(Base)에 넣어주는 전압(전류)량에 따라 포화, 활성, 차단 영역이 생기며, 각 영역의 의미는 다음과 같다.

- 활성(Activate) 영역 : C - E 간 전류가 B의 작은 입력에 대하여 변하는 영역
- 차단(Block) 영역 : B에 흐르는 전압(전류)가 너무 낮아서 C - E 사이에 전류가 흐르지 못하는 영역
- 포화(Saturation) 영역 : B에 흐르는 전압(전류)가 너무 높아서 C - E 사이의 전류가 더 이상 흐르지 못하는 영역

여기서 세 가지의 영역을 이용할 수 있는 것이 바로 증폭 기능과 Switching 기능이다.
<br>
Switching 기능은 Digital 신호인 0과 1로만 이루어진 영역에서 사용되는 ON/OFF 기능이다.
<br>
흐르는 두 가지 상태만을 표현하면 되니까, 차단 영역과 포화 영역을 이용하여 ON/OFF를 표현한다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/d053dfc9-f6b3-4b67-8e12-363f59669535)

<br>

그리고 증폭 기능, 즉, Amplifier는 Trans Resistor의 의미 그대로인 활성 영역에서 이용할 수 있다.
<br>
결국 증폭기란 아주 작은 전압 신호를 B에 흘려주면 C - E 간에 전류가 더 큰 폭으로 움직이는 원리를 이용하는 것이다.

## 트랜지스터의 종류
TR의 증폭 회로는 세 가지 종류가 있다.
<br>
세 개의 핀 중 어느 핀이 접지되었는가에 따라 Emitter 접지 회로, Collector 접지 회로, Base 접지 회로로 나뉜다.
- Emitter 접지 : 전류 전압이 증폭되고 입력과 출력은 역상이며 전압, 전류 증폭이 모두 가능하므로 일반적인 증폭기에 많이 사용
- Collector 접지 : 전류 증폭만 되고 입력과 출력은 동상이며 주로 최종단에서 전류 증폭용 또는 임피던스 매칭용으로 사용
- Base 접지 : 전압 증폭만 되고 입력과 출력은 모두 동상이며 주파수 특성이 좋아 고주파에 많이 사용됨
