## 1.3 std::array

std::array는 메모리를 자동으로 할당하고 해제한다.
<br>
std::array는 원소의 타입과 배열 크기를 매개변수로 사용하는 클래스 템플릿이다.
<br>
<br>
아래 예제는 크기가 10인 int 타입의 std::array 배열을 선언한 후 원소 값을 설정하거나 출력하는 예제 코드이다.

```
std::array<int, 10> arr1;    // 크기가 10인 int 타입 배열 선언

arr1[9] = 1;    // 첫 번째 원소를 1로 설정

std::cout << "arr1 배열의 첫 번째 원소 : " << arr1[0] << std::endl;

std::array<int, 4> arr2 = { 1, 2, 3, 4 };
std::cout << "arr2의 모든 원소: ";

for(int i = 0; i < arr2.size(); i++)
    std::cout << arr2[i] << " ";

std::cout << std::endl;

// 실행 결과
arr1 배열의 첫 번째 원소: 1
arr2의 모든 원소: 1 2 3 4
```

std::array는 C 스타일 배열과 똑같은 방식으로 배열 원소에 접근할 수 있는 [] 연산자를 제공한다.
<br>
[] 연산자에 접근하고자 하는 배열 원소 인덱스를 지정할 경우 빠른 동작을 위해 전달된 인덱스 값이 배열의 크기보다 작은지 검사하지 않는다.
<br>
<br>
대신 std::array는 at(index) 형식의 함수도 함께 제공하며, 이 함수는 인자로 전달된 index 값이 유효하지 않으면 std::out_of_range(exception)를 발생시킨다.
<br>
<br>
그러므로 at() 함수가 [] 연산자보다 조금 느린 편이지만, at() 함수를 잘 이용하면 예외를 적절하게 처리할 수 있다.
<br>
<br>
배열 인덱스를 사용자 입력으로 받거나 또는 다른 이유로 인해 유효하지 않은 인덱스에 접근할 수 있는 상황이라면 다음과 같은 예외 처리 코드를 만들 수 있다.

```
std::array<int, 4> arr3 = { 1, 2, 3, 4 };

try
{
    std::cout << arr3.at(3) << std::endl; // 에러 아님
    std::cout << arr3.at(4) << std::endl; // std::out_of_range 예외 발생
}

catch (const std::out_of_range& ex)
{
    std::cerr << ex.what() << std::endl;
}
```
std::array 객체를 다른 함수에 전달하는 방식은 기본 데이터 타입을 전달하는 것과 유사하다.
<br>
값 또는 참조(referene)로 전달할 수 있고, conts를 함께 사용할 수도 있다.
<br>
<br>
C 스타일 배열을 함수에 전달할 때처럼 포인터 연산을 사용한다거나 참조 또는 역참조(de-reference) 연산을 하지 않아도 된다.
<br>
그러므로 다차원 배열을 전달하는 경우에도 std::array를 사용하는 것이 가독성이 훨씬 좋다.
<br>
<br>
다음은 사용자 정의 함수 print()에 std::array 배열을 값으로 전달하는 예제 코드이다.

```
void print(std::array<int, 5> arr)
{
    for(auto ele : arr)
        std::cout << ele << ", ";
}

std::array<int, 5> arr = { 1, 2, 3, 4, 5 };
print(arr);

// 실행 결과

1, 2, 3, 4, 5,
```

이 print() 함수의 매개변수 데이터 타입에 전달받을 배열 크기가 고정되어 있기 때문에 다른 크기의 배열을 전달할 수 없다.
<br>
예를 들어 std::array<int, 10>을 전달하면 컴파일러는 함수 매개변수와 일치하지 않는다거나 또는 해당 매개변수를 형식으로 변환할 수 없다는 에러 메시지를 출력한다.
<br>
<br>
만약 다양한 크기에 대해 동작하는 함수를 만들려면 print()를 함수 템플릿으로 선언하고 배열 크기를 템플릿 매개변수로 전달하면 된다.
<br>
<br>
즉, 아래와 같은 형태로 작성한다.

```
template <size_t N>
void print(const std::array<int, N>& arr);
```

함수에 std::array 객체를 전달할 경우, 기본적으로 새로운 배열에 모든 원소가 복사된다.
<br>
즉, 자동으로 깊은 복사가 동작한다.
<br>
<br>
만약 이러한 동작을 피하고 싶다면 참조 또는 const 참조를 사용할 수 있다.
<br>
<br>
배열의 원소를 차례대로 접근하는 연산은 매우 자주 발생한다.
<br>
std::array는 반복자(iterator)와 범위 기반 for(range-based for) 문법을 이용하여 차례대로 접근할 수 있다.

```
std::array<int, 5> arr = { 1, 2, 3, 4, 5 };

for(auto element : arr)
{
    std::cout << element << ' ';
}

// 실행 결과
1 2 3 4 5
```

배열의 모든 원소를 차례대로 출력하기 위해 인덱스 값을 사용하는 for문을 사용할 수 있다.
<br>
이 경우에는 배열의 크기를 정확하게 지정해야 하며 인덱스 값이 배열 크기보다 같거나 커지면 에러가 발생한다.
<br>
<br>
std::array는 begin()과 end()라는 이름의 멤버 함수를 제공하며, 가장 첫 번째 원소와 마지막 원소의 위치를 반환한다.
<br>
특정 원소 위치에서 다음 원소 위치로 이동하려면 반복자에 증가 연산자(++) 또는 덧셈 연산자(+) 같은 산술 연산을 수행할 수 있다.
<br>
<br>
즉, 범위 기반 for 반복문은 begin() 위치부터 시작하여 증가 연산자(++)를 통해 차례대로 원소를 이동하다가 end() 위치에 도달하면 종료한다.
<br>
<br>
반복자는 std::array, std::vector, std::map, std::set, std::list 처럼 반복 가능한 STL 컨테이너에 대해 사용할 수 있다.
<br>
<br>
컨테이너 내부의 위치를 나타내는 데 필요한 모든 기능에 대해서도 반복자가 사용된다.
<br>
예를 들어 특정 위치에 원소를 삽입하거나, 특정 위치 또는 범위에 있는 원소를 삭제하는 등의 작업에서도 반복자가 사용된다.
<br>
반복자를 사용함으로써 소스 코드의 재사용성, 유지 보수, 가독성 측면에서 이점을 얻을 수 있다.
<br>
<br>
이제 범위 기반 반복문은 다음과 같이 바꿔서 사용할 수 있다.

```
for(auto it = arr.begin(); it != arr.end(); it++)
{
    autl element = (*it);
    std::cout << element << ' ';
}
```

const_iterator 또는 reverse_iterator 같은 형태의 반복자도 사용할 수 있다.
<br>
const_iterator 반복자는 일반 반복자의 const 버전이다.
<br>
<br>
const로 선언된 배열에 대해 begin() 또는 end() 같은 함수를 사용하면 const_iterator를 반환한다.
<br>
reverse_iterator를 사용하면 배열을 역방향으로 이동할 수 있다.
<br>
이 반복자를 ++ 같은 연산자와 함께 사용할 경우, 일반 반복자와 반대 방향으로 이동하게 된다.
<br>
<br>
[] 연산자와 at() 함수 외에 std::array에서 원소 접근을 위해 사용할 수 있는 멤버 함수는 다음과 같다.
- front() : 배열의 첫 번째 원소에 대한 참조 반환
- back() : 배열의 마지막 원소에 대한 참조 반환
- data() : 배열 객체 내부에서 실제 데이터 메모리 버퍼를 가리키는 포인터 반환

```
std:array<int, 5> arr = { 1, 2, 3, 4, 5 };
std::cout << arr.front() << std::endl;       // 1 출력
std::cout << arr.back() << std::endl;        // 5 출력
std::cout << *(arr.data() + 1) << std::endl; // 2 출력
```

std::array는 깊은 비교(Deep Comperison)를 위한 관계 연산자(Relational Operator)와 깊은 복사를 위한 복사 할당 연산자(Copy-Assignment Operator)도 지원한다.


<br>

### 1.3.1 연습 문제 1 :
