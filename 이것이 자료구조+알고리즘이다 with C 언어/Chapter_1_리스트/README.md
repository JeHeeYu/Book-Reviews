# Chapter_1_리스트 정리 내용


## 리스트란
리스트는 이름과 같이 목록 형태로 이루어진 데이터 형식을 말한다.
<br>
리스트의 목록을 이루는 개별 요소를 노드(node)라고 한다.
<br>
<br>
리스트의 첫 번째 노드를 헤드(Head), 마지막 노드를 테일(Tail)이라고 부르며 리스트의 길이는 헤드부터 테일까지의 노드 수이다.
<br>
<br>
<img src="https://user-images.githubusercontent.com/87363461/200107948-de6e7614-5c8d-459d-aaab-b152f830b81a.JPG" witdh="300" height="200">
<br>
<br>
리스트와 비슷한 데이터 형식으로 배열이 존재한다.
<br>
배열과 리스트의 차이는 배열은 생성 시 배열의 크기를 지정해 주어야 한다.(정적)
<br>
반면 리스트는 배열과 반대로 유연하게 크기를 바꿀 수 있다.(동적)
<br>
<br>
리스트의 구조는 링크드 리스트, 더블 링크드 리스트, 환형 링크드 리스트 등이 있다.
<br>
<br>

## 링크드 리스트(Linked List)
링크드 리스트란 <b>노드를 연결해서 만든 리스트</b>라고 한다.
<br>
링크드 리스트의 노드는 데이터를 보관하는 필드, 다음 노드와 연결 고리 역할을 하는 포인터로 이루어진다.
<br>
<br>
<img src="https://user-images.githubusercontent.com/87363461/200108364-b32a5d09-c707-4dfd-b03e-f1c782934c6a.JPG" witdh="200" height="100">
<img src="https://user-images.githubusercontent.com/87363461/200108435-704a72cf-ecb4-4f68-b73e-d2f7b9c749b9.JPG" witdh="700" height="100">
<br>
<br>
링크드 리스트는 포인터만 교환하면 노드 사이에 노드를 끼워 넣거나 제거하는 부분이 간단하게 이루어진다.
<br>
C언어에서 링크드 리스트는 다음과 같은 구조체로 나타낸다.
<pre>
typedef int ElementType;

typedef struct tagNode
{
    ElementType Data; // 데이터
    struct Node* NextNode; // 다음 노드
} Node;
</pre>

### 링크드 리스트의 주요 연산
링크드 리스트의 주요 연산은 두 종류로, 자료구조를 구축하기 위한 연산과 자료구조에 저장된 데이터를 활용하기 위한 연산이다
<ul>
  <li>노드 생성(CreatedNode) 및 소멸(DestroyNode)</li>
  <li>노드 추가(AppendNode)</li>
  <li>노드 탐색(GetNodeAt)</li>
  <li>노드 삭제(RemoveNode)</li>
  <li>노드 삽입(InsertAfter, InsertNewHead)</li>
</ul>
위 리스트 중 생성 및 소멸, 추가 삭제는 자료구조를 구축하기 위한 연산, 탐색은 활용 연산이다.
<br>
<br>
