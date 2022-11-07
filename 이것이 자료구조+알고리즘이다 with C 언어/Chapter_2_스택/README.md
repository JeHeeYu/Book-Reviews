# Chapter_2_스택 정리 내용

## 스택(Stack)이란
스택은 바닥에서 부터 데이터를 쌓아 올리는 자료구조의 일종이며, 스택의 입/출력은 오로지 꼭대기에서만 이루어진다.
<br>
<b>가장 먼저 들어간 데이터는 가장 나중에 나오는 구조이고(FILO First In - Last Out),</b>
<b>가장 마지막에 들어간 데이터는 가장 먼저 나오는 구조이다(LIFO Last In - First Out),</b>
<br>
<br>
C언어에서 변수를 선언한 후 수명주기가 끝나면 변수를 자동 제거하는 메모리도 스택으로 구현되어 있다.
<br>
그래서 지역 변수는 스택에 할당된다고도 표현한다.
<br>
<br>
<img src="https://user-images.githubusercontent.com/87363461/200297750-403b7fe7-5162-4c5e-a298-c45029dc3d54.JPG" width="300" height="300">
<br>
<br>
스택의 가장 중요한 부분은 <b>요소의 삽입과 삭제가 한쪽 끝에서만 이루어지는 것</b>이다.
<br>
<br>
스택은 네트워크 프로토콜, 컴파일러의 분석기, 이미지 편집 프로그램의 되돌리기 등 중요한 자료구조이다.
<br>
<br>
스택의 핵심 기능은 요소는 삽입하는 Push와 제거하는 Pop 연산 기능이다.
<br>
아래 이미지 중 왼쪽 이미지는 스택 맨 위에 새로운 요소를 담고(Push), 오른쪽 이미지는 나중에 들어온 데이터가 나가는(Pop) 이미지이다.
<br>
<br>
<img src="https://user-images.githubusercontent.com/87363461/200298247-29985853-73f4-419b-a54b-99155fb20fcd.JPG" width="400" height="300">
<img src="https://user-images.githubusercontent.com/87363461/200298335-8fcb281f-46fd-4673-914f-110cb11ecab4.JPG" width="300" height="300">
<br>
<br>

## 스택의 구조체
스택의 구조체는 간단하게 배열을 이용해 구성할 수 있다.
<br>
배열을 이용할 경우 용량을 동적으로 변경할 때 비용이 크지만, 쉽게 구현할 수 있다는 장점이 있다.
<br>
<br>
배열 기반의 스택은 각 노드를 동적으로 생성하고, 제거하는 대신, 스택 생성 초기에 사용자가 부여한 용량을 한꺼번에 생성한다.
<br>
그리고 최상위 노드의 위치를 나타내는 변수를 두고 삽입과 제거 연산을 수행한다.
<br>
<br>
스택의 구조체는 크게 2가지의 데이터 구조가 필요하다.
<ul>
<li>용량</li>
<li>최상위 노드의 위치</li>
<li></li>
</ul>
<pre>
typedef struct _Stack
{
    int capacity;
    int top;
} Stack;
</pre>
capacity 변수는 스택의 용량을, top 변수는 스택의 최상위 위치를 가리킨다.

## 
