# Pull Up, Pull Down 그리고 Open Collector 정리 내용

## Pull Up, Pull Down
Pull up과 Pull down은 '뭔가를 힙겹게 올려서 받치고 있다'와 '뭔가를 억지로 눌러서 못올라오게 한다' 라는 느낌이라고 말할 수 있다.
<br>
보통 Digital System의 핀에 인가되는 값은 Logic 0일 때 동작 상태, 또는 Logic 1일 때 동작 상태로 많이 사용된다.
<br>
하지만 어느 상태이건 만드는 사람 마음대로 동작 상태를 정하기 나름이다.
<br>
<br>
동작 상태를 Active 라고 부르며, 이때 Logic 0 Active인 핀을 Low Active라고 부르고, Logic 1 Active인 핀을 High Active 라고 부른다.
<br>
<br>
Low Active의 예를 들어 만일 CS(Chip Select)라는 핀이 Low Active일 때는 그것을 명확하게 하기 위하여 ,. _N, -, * 등을 붙여 CS/, CS_N, CS, CS* 등으로 표기하며 CS Bar라고 읽는다.
<br>
<br>
만약에 누군가 CS Bar가 동작을 안한다면, CS라는 핀이 전압 Low 상태에서 동작하는데 그게 잘 안된다는 얘기이다.
<br>
반대로 High Active 라는건 그 반대이며, 아무 것도 붙이지 않는다.
<br>
<br>
그렇다는건 Low Active의 경우 Low가 되었을 때만 동작이 되도록 확실하게 하기 위하여 평소 Default 값을 High로 만들어 놓고, Active 되었을 때만 Low로 만들어주는 것이 가장 확실한 방법이다.
<br>
<br>
반대로 Hgih Active는 평소에 Low 상태로 만들어 두었다가 Active 되었을 때만 3V High로 만들어 주는 것이 좋을 것이다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/a4348a42-5300-4b8b-aa60-b83e02109f46)

<br>

예를 들기 위하여 위와 같은 회로가 있다고 가정한다.
<br>
<br>
Digital Chip은 R에 비해 많이 큰 저항이라고 본다면, Low Active Case에서의 Pull Up이라는 건 Digital Chip의 입장에선 1번 Switch On 상태에서 Digital Chip Input에 GND, 즉 0V의 전압이 가해지게 된다.
<br>
<br>
이는 즉, Switch가 On 될 때는 Low 값이 Digital Chip에 인가됨을 의미한다.
<br>
만일 이 Digital Chip은 Input이 0일 때 동작 가능하다고 한다면, 평소에는 3V Input을 유지할 수 있는 이런 식의 회로 구성이 가장 타당하다.
<br>
<br>
그럼 반대로 High Active Pull Down 이라는 건 Pull Up과 반대로 평상시에는 Low를 유지하다가 Switch On 시켰을 때 High를 Digital Chip에 인가 시켜주는 것을 의미한다.
<br>
<br>
위 회로에서 1번은 Switch On이며, On 시켰을 경우 확실하게 High 3V가 인가되며, Off 시켰을 경우 Digital Chip에 0V가 인가된다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/c171820e-9904-4b29-adb6-1ee6aedffca2)

<br>

이렇듯, Pull Up & Pull Down은 신호의 기본적 Default Level을 무엇으로 둘 것이냐의 문제다.
<br>
<br>
예를 들어 High Active로 동작하는 Digital Chip의 경우 System이 정전기를 맞았거나, 사람이 칩을 만졌을 때, 외부 요인에 의하여 순간적으로 그때의 Input이 High가 되어버린다면 Digital Chip은 의도오 사아관없이 동작하게 되는 경우가 있다.
<br>
<br>
이런 시스템이 만일 핵미사일 발사 시스템과 연관되어 있다면 엄청난 일이다.
<br>
<br>
그리고 Switch On, Switch Off 등을 트랜지스터를 이용해서 구현할 수도 있다.
<br>
예를 들어 아래의 회로와 같이 구현된다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/93a2765c-b700-41c6-85fa-c98f3bd5e14c)

<br>

예를 들어 지금껏 Digital Chip이라 불리던 Chip은 다른 Chip에 의해 Control이 된다고 가정하고, Slave라고 부른다.
<br>
그리고 좌측에 이 Chip을 Control 하는 Chip을 Master라고 부른다.
<br>
<br>
위 회로를 분석해 보면 Vcc는 Pull Up을 위한 전원이고, Rc는 Pull을 위한 저항이다.
<br>
그리고 트랜지스터는 Switch 역할을 한다.
<br>
<br>
R1은 보통 Master Chip의 Output이 Logic 0일 때, 0V를 제대로 줘야 하는데, 가끔 낮은 전압을 내보낼 때도 있다.
<br>
이를 대비하여, 만일 트랜지스터가 On될 만큼의 전압 이하에서는 확실하게 )V로 만들어 주기 위해 R1을 달아 Base로 전류가 흐르지 못하게 만들어 준다.
<br>
<br>
보통 이런식으로 Switch로 사용하는 트랜지스터는 B와 E를 저항으로 직접 연결된 트랜지스터를 한 개의 소자로 해서 판매하기도 한다.
<br>
<br>
Master가 0을 주면 트랜지스터는 Off 상태이며, Slave로 들어가는 Input은 Pull Up되어 High가 된다.
<br>
이때 Slave는 동작하지 않는다.
<br>
<br>
반대로 Master가 1을 주면 틀내지스터는 On 상태가 되어 Slave로 들어가는 Input은 0V를 넣어주어 Slave Chip이 동작하도록 해준다.
<br>
<br>
마지막으로 이런 Switch가 Master Chip에 아예 들어가 있는 경우까지 경우를 생각해 볼 수 있다.

<br>


## Open Collector
마지막으로, 이러한 Switch가 Master Chip 내부에 아예 들어가 있는 경우를 Open Collector 라고 부른다.
<br>
이런 구조는 Collector가 Output으로 나와 있으며 아무 것도 연결되어 있지 않으니까 Open Collector라고 부른다.
<br>
Open Collector는 아래 그림과 같이 여러 개의 Master가 한 개의 Slave에 연결될 때 아주 유리한 구조이다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/9732acdf-3d8a-4e80-b5db-ffe16e2d2feb)

<br>

위 그림을 보면 여러 개의 Master는 Low Active인 Slave의 Out에 Low를 주기 위하여 High를 준다.
<br>
표와 마찬가지로 평소에는 High Pull Up 상태를 유지하다가 둘 중에 하나만 Low가 되면 Slave에는 Low가 전달되어 Slave가 동작하도록 한다.
<br>
<br>
결국에는 Low와 High가 같이 공통 버스에 인가 되어도 서로 부딪히지 않고 Low가 이기게 된다.
<br>
<br>
Open Collector 구조는 하나의 Master와 하나의 Slave에서도 사용 가능하지만, 여러 개의 Master를 Slave에 한 방에 붙여서 전원 하나와 저항 하나로 회로를 간단하게 만들 수 있는 장점이 있다.
<br>
<br>
보통 Digital Chip의 Pin Description을 보면 Open Collector인 핀은 Open Collector라고 쓰여 있어 그것을 고려하여 회로를 구성할 수록 더 진가를 발휘한다.
<br>
<br>
이런 구조는 전문 용어로 Wired OR 이라고 부르기도 한다.
<br>
여러 개의 Master 출력을 한 번에 묶을 수 있으니까 회로도 간단해 지고, 또한 서로 다른 Master의 출력을 Slave의 정격 규격 또는 전류나 전압을 Pull Up에 의하여 쉽게 맞출 수가 있어 편리하게 이용된다.
