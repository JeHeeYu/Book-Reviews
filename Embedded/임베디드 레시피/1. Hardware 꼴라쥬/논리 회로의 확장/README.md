# 논리 회로의 확장 정리 내용

## 논리 회로
논리는 0과 1로 이루어져 있는데, 이 논리는 Computing Architecture에도 적용된다.
<br>
우선 이런 Computing Architecture를 이해 하려면 TR, R, L, C 등이 어떻게 동작하는지 알아야 한다.
<br>
<br>
즉, 어떤 디지털 신호를 넣었을 때 우리가 원하는 Output을 만들어 내려면 논리적인 순서에 의해 데이터를 Manupulation할 필요가 있다.
<br>
이런 것을 논리 회로라고 부르며, 논리 회로가 어떻게 구현되어 있는지를 알아야 한다.
<br>
<br>
논리 회로는 먼저 AND가 있다.
<br>
AND란 논리 곱이라고도 부르며, if(A && B) 와도 같은 Concept이다.
<br>
두 개가 모두 참(True)이어야 Output이 참, 즉, Logic 1이 되는 것을 의미하며. 다음과 같이 표시한다.
<br>
트랜지스터를 이용한 회로는 여러 가지 구현 방법이 있으나, 아래 예제는 가장 간단한 예제이다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/dc510277-4e9a-43de-8ef5-8f4cd4601a08)

<br>

트랜지스터로의 구현은 트랜지스터 두 개를 직렬로 연결하고, A와 B가 모두 입력되어야 TR이 모두 On이 되어 Y Output으로 VCC가 출력된다.
<br>
이런 트랜지스터를 모두 그리기 번거로우니까 AND Symbol로 한 번에 나타내며, 이 Symbol 단위를 게이트(Gate)라고 부르며, 결국 AND 게이트 라고 한다.
<br>
<br>
두 번째로 OR이 있다.
<br>
OR은 논리 합 이라고 부르며, if(A || B) 와 같은 의미이다.
<br>
<br>
두 개중 아무거나 하나만 참일 경우 Logic 1이 되는 것을 의미하며 다음과 같이 표시한다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/8d7d6f1c-41f3-4e60-90ee-5a84c1f36b58)

<br>

트랜지스터로의 구현은 트랜지스터 두 개를 병렬로 연결하고 A와 B중 어느 것이라도 1이면 트랜지스터가 On이 되어 Input VCC가 Output Y로 나가는 형태이다.
<br>
이 역시 AND 게이트와 같이 OR 게이트라고 부른다.
<br>
<br>
그리고 회로를 구현하다 보면 인버터(Inverter가) 필요할 때도 있다.
<br>
이 인버터는 Input을 반전시키는 일을 하며 다음과 같이 표시한다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/7e004d16-0065-460d-9094-19387dd1d724)


<br>

이 그림은 스위치로서의 트랜지스터 역할에서 Pull 할 때 사용되는 방식이다.
<br>
그림과 같이 A가 Logic 1 일 때 Y로 0이 출력되고, A가 Logic 0일 때 Y는 1로 출력 된다.
<br>
<br>
이런식으로 트랜지스터를 잘 이용하면 로직 게이트(Logic Gate)들을 만들어 낼 수 있는데, 이런 로직 게이트로 디지털 회로를 만들어 내고, 로직 회로들을 잘 조합하여 CPU를 만들어 낼 수도 있다.
<br>
<br>
이런 디지털 로직 게이트는 AND, OR, Inverter 외 여러가지가 있다.

<br>


![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/0f5801a8-cf3f-4b1d-bb9a-da70e61353ae)

<br>

보드에서 많이 사용하는 칩 내부는 이런 로직 게이트들을 Intergration해서 특정 목적을 수행하는 로직 회로이다.
<br>
이런 게이트들도 VHDL이나 Verilog등을 이용해 프로그래밍 언어를 할 수 있는데, 이것을 FPGA라고 한다.
<br>
<br>
아래는 이런 로직 게이트를 이용해서 가장 간단한 예로 1 Bit Binary Adder를 만드는 예제이다.
<br>
먼저 Binary Adder의 진리표는 다음과 같다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/25c115b8-b296-468d-9557-13e1547a8afa)


<br>

진리표를 보면 1Bit + 1Bit는 2Bit가 된다.
<br>
사실상 가장 낮은 LSB기리 더하고 LSB가 둘 다 1일 때만 Carry 1이 발생한다.
<br>
그러니, Binary Adder의 진리표는 다음과 같이 나눌 수 있다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/034fd8f8-9d7b-4ff9-b83b-27b84dc1cae7)

<br>

이렇게 보면 왼쪽 표는 OR과 비슷하고, 오른쪽은 AND 그대로이다.
<br>
그러면 Carry는 AND를 이용하고 왼쪽의 OR는 수정해야 할 필요가 있다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/063db469-bee7-46b9-9ed7-61186e56735a)


<br>

마지막 LSB진리표에 (2, 2) 자리인 1 + 1 자리에 1만 들어가면 OR이 완성된다.
<br>
이렇게 되면 제일 비슷한 것을 생각해보면 NAND가 있는데, NAND의 진리표는 다음과 같다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/55b8e640-f03a-4956-8f9e-3ddb5b5aad1f)


<br>

(2, 2)에 1 +1 자리만 0인데, 두 개를 더하면 원하는 1의 자리만 더하는 진리표가 나오게 된다.
<br>
즉, 이것들을 모두 합하면 다음과 같은 논리 회로를 만들어 낼 수 있다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/478238f5-7cc8-4d30-95b8-06fdad325745)

<br>

간단하게 OR하고 NAND를 AND하여 Low LSB 1Bit를 만들어내고, 나머지 High MSB 1Bit는 AND를 해서 만들어 낼 수 있다.
<br>
그런데 가만 보면 OR하고 NAND를 AND해서 만들어내면 너무 복잡해 보이는데, 이 것을 더 간단하게 만들 수 있다.
<br>
즉, OR과 NAND를 AND한 것은 XOR와 같다는 것을 알 수 있다.
<br>
<br>
XOR의 의미는 Carry를 뺀 2진 합으로 0 xor 0 = 0, 0 xor 1 = 1, 1 xor 1 = 0 이 된다.
<br>
그래서 이걸 간단하게 그리면 다음과 같이 된다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/19cbf226-3b06-4cbd-95a0-688de6378a5d)


<br>

이렇게 2Bit Adder이 완성 된다.
<br>
이런 Input과 Output을 만들어 내기 위하여 로직 회로를 구성하는 방법 중 하나가 바로 카르노맵이다.
<br>
이런 카르노맵을 구사하면 간단하게 Input에 대한 원하는 Output을 만들어 내는 논리 조합을 만들어 낼 수 있다.
<br>
<br>
이렇게 Symbol로 보면 간단하지만, 사실 내부에는 수 많은 트랜지스터들이 들어 있다.
<br>
다음과 같이 2Bit Adder를 트랜지스터들의 논리 회로를 실제 그림을 그리면 다음과 같은데, 많이 복잡하다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/193ef35a-ef60-45c9-a136-ee6228c57089)


<br>

이런 논리 회로들은 다음과 같은 모양으로 IC라는 집적 회로에 집적되기도 하며, 예를 들면 다음과 같은 그림이다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/b93ab2f0-3b4d-4d4e-8cd7-e4ff995be96d)

<br>

이런 논리 조합들을 잘 조절하면 우리가 원하는 Output을 만들어내는 회로를 만들어 낼 수 있다.
<br>
보드에 사용되는 디지털 칩이 아래와 같이 논리 회로가 가득 차 있는 모양이다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/e0ae52e7-a2f3-4f19-9f1e-292849f133b3)


<br>

## 마무리
전압을 이야기할 때 Vcc라든가 Vee 등의 Notation을 많이 사용하는데, 그 의미는 다음 표와 같다.

|BJT|FET|'Vxx' 의미|
|:---:|:---:|:---:|
|Vcc|Vdd|Positive Supply Voltage|
|Vee|Vss|Negative Supply Ground|

BJT(Transistor)와 FET 사이의 사용하는 Notation이 다르다.
<br>
Vcc는 Collector 전원, Vee는 Emitter 전원, Vdd는 Drain 전원, Vss는 Sourrce 전원이다.
<br>
<br>
요약하면 Vcc, Vdd는 전원으로 연결되고, Vee, Vss는 보통 Ground 단에 연결된다.
<br>
<br>
통상 BJT 회로를 이용하여 꾸민 회로를 TTL(Transistor - Transistor Logic) 이라고 부르며,
<br>
FET를 이용하여 꾸민 회로를 CMOS 회로라고 부르는데, BJT나 FET나 하는 역할은 비슷하다.
<br>
BJT와 마찬가지로 입력 전압으로 출력 전류를 제어하는 역할을 하는데, 결과론 적으로는 같으나, 원리는 많이 틀리다.
<br>
<br>
FET는 높은 Input Impedance를 가지고 있어 잔력 사용 효율이 좋으며, 단점으로는 속도가 느려 단순 스위치로는 적합하나 Linear Amplifier로는 부접합 하다.
<br>
<br>
정리하자면 FET는 Logic쪽에, BJT는 Amplifier쪽에 어울린다는 의미에서 시작하자면 Logic 쪽에서는 VDD - VSS, Linear Operation 쪽에서는 VCC - GND의 Notation이 많이 사용된다.
