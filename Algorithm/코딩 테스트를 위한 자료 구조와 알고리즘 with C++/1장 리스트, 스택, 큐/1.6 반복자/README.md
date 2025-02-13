## 반복자

반복자는 포인터와 비슷하지만, STL 컨테이너에 대해 공통의 인터페이스를 제공한다.
<br>
반복자를 이용한 연산은 어떤 컨테이너에서 정의된 반복자인지에 따라 결정된다.
<br>
벡터와 배열에서 사용되는 반복자는 기능 면에서 가장 유연하다.
<br>
<br>
벡터와 배열은 연속된 자료 구조를 사용하기 때문에 특정 위치의 원소에 곧바로 접근할 수 있다.
<br>
이러한 반복잡을 임의 접근 반복자(Random Access Iterator)라고 한다.
<br>
<br>
그러나 std::forward_list의 경우 기본적으로 역방향으로 이동하는 기능을 제공하지 않으며 바로 이전 노드로 이동하려면 맨 처음 노드부터 시작해서 찾아가야 한다.
<br>
따라서 std::forward_list에서는 증가 연산만 가능하며, 이러한 반복자를 순방향 반복자(Forward Iterator)라고 한다.
<br>
<br>
반복자 타입에 따라 사용할 수 있는 함수 중에 advance(), next(), prev() 함수가 있다.
<br>
advance() 반복자는 거리 값을 인자로 받고, 반복자를 거리 값만큼 증가 시킨다.
<br>
prev()와 next() 함수도 반복자와 거리 값을 인자로 받고 해당 반복자에서 지정한 거리만큼 떨어진 위치의 반복자를 반환한다.
<br>
이들 함수는 해당 반복자가 지원할 경우에만 동작한다.
<br>
<br>
예를 들어 순방향으로만 이동 가능한 순방향 반복자에 대해 prev() 함수를 사용하면 에러가 발생한다.
