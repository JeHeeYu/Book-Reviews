## std:list

std::list는 std::forward_list의 적은 기능만 제공하는 단점을 보완하기 위해 제공하는 클래스이다.
<br>
std::list는 양쪽 방향으로 연결된 리스트, 즉 이중 연결 리스트(Dounly-Linked List) 구조로 되어 있다.
<br>
덕분에 std::forward_list에 비해 더 많은 기능을 제공하나, 메모리를 조금 더 사용한다.

```
// 이중 연결 리스트 노드 기본 형태
struct doubly_linked_list_node
{
    int data;
    doubly_linked_list_node* next;
    doubly_linked_list_node* prev;
};
```

<br>

### 1.7.1 std::list 멤버 함수

std::list에서 제공하는 대부분의 함수는 std::forward_list의 함수와 같거나 유사하나, 약간의 차이가 있다.
<br>
예를 들어 std::forward_list에서 after로 끝나는 함수는 std::list에서 _after로 끝나지 않는 형태로 바뀐다.
<br>
<br>
즉, insert_after()와 emplace_after() 함수는 insert()와 emplace() 함수와 대응된다.
<br>
<br>
std::list에서는 원소 이동을 역방향을도 할 수 있으므로 원소 삽입을 위해 특정 원소의 이전 원소 반복자를 제공하지 않아도 되며, 대신 정확하게 새로운 원소가 삽입될 위치를 가리키는 반복자를 함수 인자로 전달한다.
<br>
<br>
이외에도 std::list는 빠른 push_back(), emplace_back(), pop_back() 함수를 제공한다.
