# Memory의 선정과 XIP 정리 내용

## 메모리(Memory)
임베디드 시스템에서 메모리의 선정은 기본적인 시스템 구성과 성능에 가장 큰 영향력을 행사한다.
<br>
또한 메모리 종류의 선정을 어떻게 했느냐에 따라 메모리 맵을 어떻게 구성할 것이냐의 디자인 가이드가 달라진다.
<br>
그에 따라 하드웨어 구성도 많이 달라지고 가격에도 많은 영향을 미친다.
<br>
<br>
아래 구조는 임베디드 시스템에서 가장 많이 사용되는 구성이다.
<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/5f4dadb7-57d2-46f6-aa60-8e40630741c1)

<br>

우선 메모리는 크게 램(RAM)과 롬(ROM)으로 나눌 수 있다.
<br>
보통 램을 휘발성, 롬을 비 휘발성이라고 하는데, 그 이유는 데이터가 날아가냐의 유무이다.
<br>

## RAM

램은 전원이 나가면 램에 들어있던 데이터도 같이 날아가고, 롬은 전원이 나가도 데이터가 계속 저장 되어 있는 성질의 메모리이다.
<br>
예전에는 램에는 데이터가, 롬에는 코드가 들어가는 것이 통상적이었는데, 이제는 많이 변경되었다.
<br>
<br>
XIP(Execute In Place)라는 개녕미 나오는데, XIP는 메모리 상에서 직접 프로그램/코드를 실행할 수 있는 기술을 말하며, 기본적으로는 Random Acces가 가능해야 한다.
<br>
다시 말해 Byte/Word 단위 등의 크기를 직접 Access가 가능해야 한다는 뜻이며, 모든 램은 이러한 요건을 충족한다.
<br>
<br>
즉, 램에 Program XIP를 올리기만 하면 실행 가능하다는 뜻이다.

## 램의 종류
램에서 가장 비싼 램은 SRAM(Static RAM)이다.
<br>
이유는 아무런 전기적인 동작없이 그저 순수하게 Random Access가 가능한 휘발성 메모리인 것이다.
<br>
<br>
반대로 제일 저렴한 램은 DRAM(Dynamic RAM)이다.
<br>
DRAM은 한 번 Memory Cell에 뭔가를 기억 시킨다고 끝이 아니다.
<br>
시간이 지나면 Charging 되었던 전하가 빠져나가서 데이터를 유실하게 되고, 같은 주소 즉, 같은 자리를 계속 읽으면 데이터가 유실된다.
<br>
말하자면 전원을 끄지 않아도 점점 데이터가 유실되는 현상을 볼 수 있다.
<br>
<br>
그래서 DRAM은 일정 시간마다 다시 전하를 충전해 주어야 갖고 있던 데이터를 유실하지 않을 수 있다.
<br>
당연하게 DRAM에는 이런 Charging을 위한 Precharge Circuit을 가지고 있어야 한다.
<br>
<br>
그러나 SRAM에 비해 회로가 단순하고 직접도가 높다.
<br>
부진러히 Charging을 해주어야 하는 대신 같은 가격이라면 SRAM보다 더 큰 SDRAM을 살 수 있는 셈이다.
<br>
<br>
이런 Charging 해주는 부분도 사용자가 PL172라는 규격을 통하여 DRAM 제어를 해주어야 한다는 번거로움이 있다.
<br>
<br>
그리고 PSRAM(Pseudo SRAM)도 있는데, 이 램은 가짜 SRAM이다.
<br>
PSRAM은 SRAM의 장점을 본 떠서 만든 램인데, SRAM처럼 보이는 DRAM이다.
<br>
<br>
PSRAM은 따로 제어를 해줄 필요가 없는 DRAM인데, 하드웨어적으로 Precharge를 해주는 회로가 DRAM에 딸려서 나온다.
<br>
즉, DRAM의 Charging 제어를 해주니까 사용자 입장에서는 마치 SRAM처럼 사용할 수 있는 것이다.
<br>
<br>
결국 같은 크기라면 SRAM > PSRAM > DRAM인 셈이다.

<br>

# ROM
임베디드 시스텡메서 롬은 쓰고 지울 수 있는 Flash Memory가 롬으로 가장 많이 애용된다.
<br>
Flash memory라 함은 롬이긴 하지만 특별한 제어 과정을 거치면 지울 수도 있고 그 위에 쓸 수도 있다.
<br>
<br>
그러니 언제라도 Burning(Programming) 가능한 Flash는 임베디드 환경에서 엄청나게 인기가 많을 수 밖에 없다.
<br>
업그레이드에도 유리하고 심지어 지울 수 있는 기능 때뭉네 File System Storageㄹ도 사용 된다.
<br>
<br>
NOR와 NAND를 같은 크기로 비교하면 NOR가 NAND보다 더 비싸다.(NOR > NAND)
<br>d
일단 NOR와 NAND의 차이는 NOR는 Cell이 병렬로 연결되어 있으며 병렬로 연결되어 있다 보니 Address Line과 Data Line을 모두 가질 수 있으며, 램처럼 Byte 단위로 Random Access가 가능해진다.
<br>
<br>
그리고 NOR는 XIP를 지원하는데, XIP를 지원한다는 얘기는 Software을 직접 실행할 수 있다는 얘기이다.
<br>
<br>
반대로 NAND는 XIP를 지원하지 않는다.
<br>
그 이유는 NAND는 기본적으로 Cell이 직렬로 연결되어 있으며, 작은 단위로는 읽을 수 없고, 한 번 읽을 때 1개 Page 단위로만 읽을 수 있다.
<br>
Small Page를 지원하는 NAND는 512Byte 크기로만 읽을 수 있고, Large Page를 지원하는 NAND는 2KB씩 한 번에 읽을 수 있다.
<br>
<br>
NOR형은 쓰기와 지우기는 느리지만 읽기가 빠르다는 특성이 있다.
<br>
이 부분은 대용량 데이터를 저장하는 데에는 한계가 있다는 뜻이다.
<br>
<br>
반면 NAND형은 읽기는 느리지만 쓰기와 지우기가 빠르다.
<br>
이 부분은 대용량 저장이 가능하다는 장점이 있다.
<br>
<br>
쓰기와 지우기가 빠른 것 중 어떤 것을 선택할 것이냐가 플래시 메모리를 선택하는 기준이다
