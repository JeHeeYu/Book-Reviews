# 프로토타입 패턴 정리 내용

## 프로토타입(Prototype) 패턴
프로토타입 패턴은 소프트웨어 디자인 패턴 용어로, 생성할 객체들의 타입이 프로토타입인 인스턴스로부터 결정되도록 하며, 인스턴스는 새 객체를 만들기 위해 자신을 복제하게 된다
<br>
<br>
프로토타입 패턴을 이용하면 어떤 모델 객체를 복제할 수 있고, 복제본을 커스터마이즈할 수도 있고, 사용할 수도 있다.
<br>
프로토타입 패턴에서 가장 까다로운 부분은 복제를 구현하는 부분이다.

## 객체 생성
대부분 객체를 생성할 때 생성자를 이용한다.
<br>
하지만 이미 잘 설정된 객체가 있다면 같은 것을 복제하는 것이 가장 쉽다.
<br>
<br>
빌더 패턴을 이용해 복잡한 생성 과정이 단순화되어 있다면 복제를 하는 것이 더욱 의미가 있다.
<br>
<br>
아래는 중복된 객체가 있는 단순 예제 코드이다.

```
Contact john{ "John Doe", Address{"123 East Dr", "London", 10 } };
Contact jane{ "Jane Doe", Address{"123 East Dr", "London", 11 } };
```

john과 jane은 사무실 방만 다르고 같은 빌딩에서 일 하고 있다.
<br>
이 두 사람뿐만 아니라 다른 많은 직원도 직장 주소가 "123 East Dr in London"일 것이다.
<br>
따라서 수많은 객체가 같은 값으로 중복되게 초기화되는 작업이 발생한다.
<br>
<br>
여기서 프로토타입 패턴을 사용할 수 있다.
<br>
<br>
사실 프로토타입 패턴은 객체의 복제가 주요 기능이다.
<br>
당연하게도 객체를 복제하는 하나의 일관된 방법은 없다.

<br>

## 평범한 중복 처리
복제의 목적이 값을 사용하는 것에 있고, 복제 대상 객체의 모든 항목이 값으로만 되어 있다면 복제하는데 문제가 될 것이 없다.
<br>
예를 들어 연락처(Contact)와 주소(Address)가 아래와 같이 정의되어 있다.

```
struct Address
{
    string street, city;
    int suite;
}

struct Contact
{
    string name;
    Address address;
}
```

이 구조는 아래와 같이 코드를 사용하는데 아무런 문제가 없다

```
// 프로토타입 객체
Contact worker{"", Address{"123 East Dr", "London", 0}};

// 프로토타입을 복제하여 일부 수정
Contact john = worker;
john.name = "John Doe";
john.address.suite = 10;
```

하지만 실제 이렇게 쉬운 경우는 드물며 아래 코드와 같이 내부 객체가 포인터로 된 경우가 많다.

```
struct Contact
{
    string name;
    Address *address; 
}
```

이러한 코드는 Contanct john = prototype 코드가 수행될 때 포인터가 복제되기 때문에 큰 문제가 발생한다.
<br>
포인터가 복제되어 prototype과 john 둘 다 같은 address 객체를 가지게 된다.
<br>
<br>
즉, john의 address를 수정했을 뿐인데 prototype의 address도 바뀌게 된다.

<br>

## 복제 생성자를 통한 중복 처리

중복을 피하는 가장 단순한 방법은 객체의 복제 생성자에서 내부 구성 요소들(연락처와 주소) 모두를 적합하게 다루도록 정의하는 것이다.
<br>
예를 들어 아래와 같이 주소를 포인터로 저장되어 있다.

```
struct Contact
{
    string name;
    Address* address;
}
```








