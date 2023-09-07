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

<br>

### 1.7.2 연습 문제 6 : [std::list의 삽입 또는 삭제 함수 사용하기](https://github.com/JeHeeYu/Book-Reviews/blob/main/Algorithm/%EC%BD%94%EB%94%A9%20%ED%85%8C%EC%8A%A4%ED%8A%B8%EB%A5%BC%20%EC%9C%84%ED%95%9C%20%EC%9E%90%EB%A3%8C%20%EA%B5%AC%EC%A1%B0%EC%99%80%20%EC%95%8C%EA%B3%A0%EB%A6%AC%EC%A6%98%20with%20C%2B%2B/1%EC%9E%A5%20%EB%A6%AC%EC%8A%A4%ED%8A%B8%2C%20%EC%8A%A4%ED%83%9D%2C%20%ED%81%90/1.7%20std%3A%3Alist/std_list_example.cpp)

<br>

### 실행 결과

```
삽입 & 삭제 후 리스트 : 1 0 2 3 4 5 6
```

<br>

### 이중 연결 리스트에서 원소 삽입 시 포인터 관리 방법

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/abf87fc4-a314-4f76-b3c5-1e200a48c40c)

<br>

std::list에서 원소 삽입은 prev와 next 두 개의 포인터 값을 적절하게 수정해야 하기 때문에 std::forward_list와 비교하면 두 배의 연산량이 필요하다.

<br>

### 1.7.3 양방향 반복자

std::list는 이중 연결 리스트이므로 역방향 반복자를 사용하여 역방향으로의 연산을 수행할 수 있다.
<br>
그러나 std::list 반복자는 임의 접근 반복자보다는 유연하지 않다.
<br>
std::list 반복자를 이용하여 어느 방향으로든 원하는 만큼 이동할 수 있지만, 임의 접근 반복자의 경우처럼 특정 위치로 한 번에 이동하는 것은 불가능하다.
<br>
<br>
그러므로 std::list 반복자를 이용하여 특정 위치로 이동하는 작업은 선형 시간 복잡도를 가진다.
<br>
std::list 반복자는 어느 방향으로든 이동할 수 있는 양방향 반복자(Bidirectional Iterators)라고 부른다.

<br>

### 1.7.4 반복자 무효화

반복자는 메모리 주소를 가리키는 포인터를 이용하여 구현되었기 때문에 경우에 따라 컨테이너가 반복될 경우 제대로 동작하지 않을 수 있다.
<br>
그러므로 컨테이너가 변경되어 특정 노드 또는 원소의 메모리 주소가 바뀌면 사용하던 반복자가 무효화될 수 있고, 이 경우 예측할 수 없는 동작이 발생할 수 있다.
<br>
<br>
예를 들어 벡터를 보면 맨 뒤에 원소를 추가하는 함수 vector::push_back() 함수가 있다.
<br>
이 함수는 경우에 따라 새로 메모리 공간을 할당하고 기존의 모든 원소를 새 메모리 공간으로 복사하는 동작이 발생한다.
<br>
이 경우 기존에 사용하던 모든 반복자와 포인터, 참조는 모두 무효화된다.
<br>
<br>
마찬가지로 vector::insert() 함수를 수행할 때 메모리 재할당이 필요한 경우라면, 이 경우에도 기존의 반복자, 포인터, 참조는 모두 사용하면 안 된다.
<br>
vector::insert() 함수에서 매모리 재할당 없이 원소를 삽입하는 경우라면, 원소 삽입 위치 이후에 있는 원소를 모두 이동해야 하므로 이 위치를 가리키는 반복자는 모두 무효화된다.
<br>
<br>
벡터와 달리 연결 리스트에서는 삽입 또는 삭제 동작에서 원소를 이동할 필요가 없으므로 반복자가 무효화되지 않는다.
<br>
즉, std::list 또는 std::forward_list에서 삽입 동작은 반복자의 유효성에 영향을 미치지 않는다.
<br>
<br>
다만 특정 원소를 삭제하는 경우, 삭제되는 원소를 가리키던 반복자는 당연히 무효화되지만 나머지 원소를 가리키는 반복자는 그대로 사용할 수 있다.

<br>
아래 코드는 다양한 연산이 반복자에 어떤 영향을 주는지 확인하기 위한 코드이다.

```
std::vector<int> vec = { 1, 2, 3, 4, 5 };

// v_it[4]는 vec[4] 원소를 가리킴
auto v_it4 = vec.begin() + 4;

// v_it4 반복자는 무효화됨
vec.insert(vec.begin() + 2, 0);
```

위 경우에서 마지막 문장이 수행되면 v_it4 반복자는 무효화되며, v_it4를 이용하여 원소에 접근하려고 하면 에러가 발생한다.

<br>

```
std::list<int> lst = { 1, 2, 3, 4, 5 };

auto l_it4 = next(lst.begin(), 4);

// l_it4 반복자는 유효함
lst.insert(next(lst.begin(), 2), 0);
```

<br>

