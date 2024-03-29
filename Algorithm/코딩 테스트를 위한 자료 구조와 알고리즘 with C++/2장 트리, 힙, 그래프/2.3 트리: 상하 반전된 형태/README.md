## 2.3 트리: 상하 반전된 형태
트리의 계층 구조를 화면에 도식적으로 나타내려면 말 그대로 나무 형태로 나타낼 수 있으며, 이때 에지는 나뭇가지처럼 표현된다.
<br>
트리의 중심이 되는 노드를 루트 노드(Root Node)라고 부르며 이 노드는 보통 가장 맨 위에 나타낸다.
<br>
<br>
즉, 트리 구조를 그림으로 나타낼 때는 실제 나무와는 반대로 뿌리가 맨 위에 나타내는 상하 반전된 형태로 표현한다.

<br>

### 2.3.1 연습 문제 7 : [조직도 구조 만들기](https://github.com/JeHeeYu/Book-Reviews/blob/main/Algorithm/%EC%BD%94%EB%94%A9%20%ED%85%8C%EC%8A%A4%ED%8A%B8%EB%A5%BC%20%EC%9C%84%ED%95%9C%20%EC%9E%90%EB%A3%8C%20%EA%B5%AC%EC%A1%B0%EC%99%80%20%EC%95%8C%EA%B3%A0%EB%A6%AC%EC%A6%98%20with%20C%2B%2B/2%EC%9E%A5%20%ED%8A%B8%EB%A6%AC%2C%20%ED%9E%99%2C%20%EA%B7%B8%EB%9E%98%ED%94%84/2.3%20%ED%8A%B8%EB%A6%AC%3A%20%EC%83%81%ED%95%98%20%EB%B0%98%EC%A0%84%EB%90%9C%20%ED%98%95%ED%83%9C/tree_structure.cpp)
이 연습 문제는 회사 조직도를 코드로 구현한 문제이다.

<br>

### 실행 결과

```
CEO 아래에 부사장(을)를 추가했습니다.
부사장 아래에 IT부장(을)를 추가했습니다.
부사장 아래에 마케팅부장(을)를 추가했습니다.
IT부장 아래에 보안팀장(을)를 추가했습니다.
IT부장 아래에 앱개발팀장(을)를 추가했습니다.
마케팅부장 아래에 물류팀장(을)를 추가했습니다.
마케팅부장 아래에 홍보팀장(을)를 추가했습니다.
부사장 아래에 재무부장을(를) 추가할 수 없습니다.
```

이 문제를 트리 그림으로 표현하면 아래와 같다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/7ca739ed-4af5-4fc6-b2b5-da4539feff31)

<br>

### 2.3.2 트리 순회

### 전위 순회(Preorder Traversal)
이 방법은 노드를 먼저 방문하고, 그 다음은 현재 노드의 왼쪽 하위 노드를, 마지막으로 현재 노드의 오른쪽 하위 노드를 재귀적인 방식으로 방문한다.
<br>
여기서 '전위(pre)'는 상위 노드를 하위 노드보다 먼저 방문한다는 뜻을 말한다.
<br>
<br>
연습 문제 7번을 전위 순회 방식으로 탐색하면 다음과 같다.
```
CEO, 부사장, IT부장, 보안팀장, 앱개발팀장, 마케팅부장, 물류팀장, 홍보팀장
```
전위 순회는 항사 부모 노드를 방문한 후 다음 왼쪽 자식 노드, 오른쪽 자식 노드를 차례로 방문한다.
<br>
이러한 방식의 순회를 루트 노드만이 아닌 루트 노드 아래의 모든 서브 트리에 대해 적용한다.
<br>
<br>
전위 순회는 다음과 같이 구현할 수 있다.
```
static void preOrder(node* start)
{
    if(!start)
        return;

    std::cout << start->position << ", ";
    preOrder(start->first);
    preOrder(start->second);
}
```

<br>

### 중위 순회(In-Order Traversal)
중위 순회는 왼쪽 노드를 먼저 방문하고, 그 다음 현재 노드, 마지막으로 오른쪽 노드를 방문하는 방법을 말한다.
<br>
<br>
연습 문제 7번을 중위 순회 방식으로 탐색하면 다음과 같다.
```
보안팀장, IT부장, 앱개발팀장, 부사장, 물류팀장, 마케팅부장, 홍보팀장, CEO
```
중위 순회는 다음과 같이 구현할 수 있다.
```
static void inOrder(node* start)
{
    if(!start)
        return;

    inOrder(start->first);
    std::cout << start->position << ", ";
    inOrder(start->second);
}
```

<br>

### 후위 순회(Post-Order Traversal)
후위 순회는 두 자식 노드를 먼저 방문한 후 현재 노드를 방문하는 방법을 말한다.
<br>
연습 문제 7번을 중위 순회 방식으로 탐색하면 다음과 같다.
```
보안팀장, 앱개발팀장, IT부장, 물류팀장, 홍보팀장, 마케팅부장, 부사장, CEO
```
후위 순회는 다음과 같이 구현할 수 있다.
```
static void postOrder(node* start)
{
    if(!start)
        return;

    postOrder(start->first);
    postOrder(start->second);
    std::cout << start->position << ", ";
}
```

<br>

### 레벨 순서 순회(Level Order Traversal)
레벨 순서 순회 방법은 트리의 맨 위 레벨부터 아래 레벨까지, 왼쪽 노드에서 오른쪽 노드 순서로 방문하는 방법을 말한다.
<br>
즉, 트리의 루트 노드부터 단계별로 차례대로 나열하는 것과 같다.
<br>
<br>
연습 문제 7번을 레벨 순서 순회 방식으로 탐색하면 다음과 같다.
```
CEO,
부사장,
IT부장, 마케팅부장,
보안팀장, 앱개발팀장, 물류팀장, 홍보팀장,
```

<br>

### 연습 문제 8 : [레벨 순서 순회 구현하기](https://github.com/JeHeeYu/Book-Reviews/blob/main/Algorithm/%EC%BD%94%EB%94%A9%20%ED%85%8C%EC%8A%A4%ED%8A%B8%EB%A5%BC%20%EC%9C%84%ED%95%9C%20%EC%9E%90%EB%A3%8C%20%EA%B5%AC%EC%A1%B0%EC%99%80%20%EC%95%8C%EA%B3%A0%EB%A6%AC%EC%A6%98%20with%20C%2B%2B/2%EC%9E%A5%20%ED%8A%B8%EB%A6%AC%2C%20%ED%9E%99%2C%20%EA%B7%B8%EB%9E%98%ED%94%84/2.3%20%ED%8A%B8%EB%A6%AC%3A%20%EC%83%81%ED%95%98%20%EB%B0%98%EC%A0%84%EB%90%9C%20%ED%98%95%ED%83%9C/level_order_traversal.cpp)
