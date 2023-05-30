# 데코레이터 패턴 정리 내용

## 데코레이터(Decorator) 패턴 정리 내용
데코레이터 패턴은 객체들을 새로운 행동들을 포함하는 특수 래퍼 객체들 내에 넣어서 위 행동들들 객체들이 연결시키는 구조적 디자인 패턴이다.
<br>
예를 들어 동료가 작성한 클래스르 기반으로 어떤 기능을 확장해야 하는 상황이다.
<br>
원본 코드를 수정하지 않고 사용해야 하는 방법을 찾아야 한다.
<br>
<br>
가장 쉽게 생각나는 방법은 상속을 이용하는 것이다.
<br>
동료의 클래스를 부모로하는 자식 클래스를 만들어 거기에 새로운 기능을 추가하는 것이다.
<br>
몇몇 메서드들을 오버라이딩 한다면 원본 코드에 수정을 가하지 않고 작업을 진행할 수 있다.
<br>
<br>
하지만 항상 이렇게 사용할 수 있는 것은 아니다.
<br>
예를 들어 상속을 사용할 수 없는 상황이 있는데, std::vector의 경우 버추얼 소멸자가 없다는 제한이 있다.
<br>
<br>
무엇보다도 상속을 활용하기 어려운 가장 큰 이유는, 수정하는 이유가 여러 가지인 경우 단일 책임 원칙(SRP)에 따라 그 수정 사항들을 각각 완전히 분리하는 것이 바람직하기 때문이다.
<br>
<br>
즉, 데코레이터 패턴은 이미 존재하는 타입에 새로운 기능을 추가하면서도 원래 타입의 코드에 수정을 피할 수 있게 해준다.(열림-닫힘 원칙(OCP)이 준수됨)
<br>
여기에 더해서 파생해야 할 타입의 개수가 과도하게 늘어나는 것도 막을 수 있다.

<br>

## 시나리오
예를 들어 도형을 나타내는 클래스 Shape가 기존에 존재하고 있었고 이를 상속받아 색상이 있는 도형(ColoredShape)과 투명한 도형(TransparentShape)을 추가했다고 가정한다.
<br>
그리고 나중에 두 가지 특성을 모두 필요로 하는 경우가 발생하여 추가로 ColoredTransparentShape를 만들었다고 가정한다.
<br>
<br>
결과적으로 두 가지 기능을 추가하기 위해 클래스를 3개 만들었다.
<br>
이런 식이라면 기능이 하나 더 추가될 경우 7개의 클래스를 만들어야 할 수도 있다.
<br>
<br>
설상가상으로 Squre, Circle 등과 같은 도형의 파생 클래스들까지 적용해야 할 수도 있다.
<br>
<br>
세 가지의 추가 기능을 두 가지의 도형에 적용하려면 상속받아 만들어야 할 클래스가 14개까지 늘어난다.
<br>
이런 상황은 비효율적이며 감당할 수 없다.
<br>
<br>
이러한 코드를 다음과 같이 작성할 수 있다.
<br>
예를 들어 추상 클래스 Shape이 아래와 같이 정의되어 있다.

```
struct Shape
{
    virtual string str() const = 0;
};
```

이 클래스에서 str()은 버추얼 함수이고 특정 도형의 상태를 텍스트로 나타낸다.
<br>
이 클래스를 베이스로 하여 원, 사각형 같은 도형을 정의하고 str() 인터페이스를 구현할 수 있다.

```
struct Circle : Shape
{
    float radius;

    explicit Circle(const float radius)
        : radius{radius} {}

    void resize(float factor) { radius *= factor; }

    string str() const override
    {
        ostringstream oss;
        oss << "A Circle of radius " << radius;
        return oss.str();
    }
};

/ Sqaure 구현 생략
```

평범한 상속으로는 효율적으로 새로운 기능을 도형에 추가할 수가 없다는 것을 알 수 있다.
<br>
따라서 접근 방법을 달리해서 컴포지션을 활용한다.
<br>
<br>
컴포지션은 데코레이터 패턴에서 객체들에 새로운 기능을 확장할 때 활용되는 메커니즘이다.
<br>
이 접근 방식은 다시 두 가지 서로 다른 방식으로 나누어진다.

- <b>동적 컴포지션</b> : 참조를 주고받으면서 런타임에 동적으로 무언가를 합성할 수 있게 한다.<br>이 방식은 최대한의 유연성을 제공한다.<br>예를 들어 사용자 입력에 따라 런타임에 반응하여 컴포지션을 만들 수 있다.
- <b>정적 컴포지션</b> : 템플릿을 이용하여 컴파일 시점에 추가기능이 합성되게 한다.<br>이것은 코드 작성 시점에 객체에 대한 정확한 추가 기능 조합이 결정되어야만 한다는 것을 암시한다.<br>즉, 나중에 수정될 수 없다.

<br>

## 동적 데코레이터(Dynamic Decorator)
예를 들어 도형에 색을 입히려고 하는 상황이다.
<br>
상속 대신 컴포지션으로 ColoredShape를 만들 수 있다.
<br>
<br>
이미 생성된 Shape 객체의 참조를 가지고 새로운 기능을 추가한다.

```
struct ColoredShape : Shape
{
    Shape& shape;
    string color;

    ColoredShape(Shape& shape, const string& color)
        : shape{shape}, color{color} {}

    string str() const override
    {
        ostringstream oss;
        oss << shape.str() << " has the color " << color;
        return oss.str();
    }
};
```

위 코드에서 볼 수 있듯이 ColoredShape은 그 자체로서도 Shape이다.
<br>
ColoredShape는 다음과 같이 이용될 수 있다.

```
Circle circle{0.5f};
ColoredShape redCircle{circle, "red"};
cout << redCircle.str();
// 출력 결과 "A circle of radius 0.5 has the color red"
```

만약 여기에 더하여 도형이 투명도를 가지게 하고 싶다면 마찬가지 방법으로 다음과 같이 쉽게 구현할 수 있다.

```
struct TransparentShape : Shape
{
    Shape& shape;
    uint8_t transparency;
    
    TransparentShape(Shape& shape, const uint8_t transparency)
        : shape{shape}, transaprentcy{transparency} {}
        
    string str() const override
    {
        ostringstream oss;
        oss << shape.str() << " has " << static_cast<float>(transparency) / 255.f * 100.f << "% transparency";
        
        return oss.str();
    }
};
```

이렇게 하면 0 ~ 255 범위의 투명도를 지정하면 그것을 퍼센티지로 출력해주는 새로운 기능이 추가되었다.
<br>이러한 추가 기능은 그 자체만으로는 사용할 수 없고 적용할 도형 인스턴스가 있어야만 한다.

```
Square sqaure{3};
TransparentShape demiSquare{sqaure, 85};
cout << demiSquaure.str();
// 출력 결과 "A square with side 3 has 33.333% transparency"
```

하지만 편리하게도 ColoredShape와 TransparentShape를 합성하여 색상과 투명도 두 기능 모두 도형에 적용되도록 할 수 있다.

```
TransparentShape myCircle {
    ColoredShape { 
        Circle{23}, "green"
    }, 64
};

cout << myCircle.str();

// 출력 결과 A circle of radius 23 has the color green has 25.098% transparency"
```

위 코드에서 볼 수 잇듯이 모두 즉석에서 생성하고 있다.
<br>
하지만 이 코드는 문제도 있다. 
<br>
비상식적인 합성도 가능하다.
<br>
<br>
같은 데코레이터를 중복해서 적용해버릴 수도 있다.
<br>
<br>
예를 들어 ColoredShape{ColoredShape{...}}과 같은 합성은 비상식적이지만 동작한다.
<br>
이렇게 중복된 합성은 "빨간색이면서 노란색"이라는 모순된 상황을 야기한다.
<br.
<br>
OOP의 테크닉들을 활용하면 예로든 중복 합성이 방지되도록 할 수 있다.
<br>
하지만 또 아래와 같은 경우에서 문제가 발생한다.

```
ColoredShape{TransparentShape{ColoredShape{...}}}
```



이런 경우는 탐지해내가기 훨씬 어려우며, 만약 가능하다고 해도 투자 대비 효과를 따져봐야 한다.

<br>

## 정적 데코레이터(Static Decorator)

예제 시나리오에서 Circle에는 resize() 멤버 함수가 있으나 이 함수는 Shape 인터페이스와 관계가 없다.
<br>
그래서 resize() 함수는 Shape 인터페이스에 없기 때문에 데코레이터에서 호출할 수가 없다.
<br>
따라서 아래 코드는 컴파일이 안되는 오류가 발생한다.

```
Circle circle{3};
ColoredShae redCircle{circle, "red"};
redCircle.resize(2); // 컴파일 오류
```

런타임에 객체를 합성할 수 있는지는 별로 상관하지 않는다.
<br>
하지만 데코레이션된 객체의 멤버 함수와 필드에는 모두 접근할 수 있어야 한다.
<br>
<br>
그러기 위해서 템플릿과 상속을 활용하되 여기에서의 상속은 가능한 조합의 수가 폭발적으로 증가하지 않는다.
<br>
보통의 상속 대신 믹스인(MixIn) 상속이라 불리는 방식을 이용한다.
<br>
<br>
믹스인 상속은 템플릿 인자로 받은 클래스를 부모 클래스로 지정하는 방식을 말한다.
<br>
<br>
기본 아이디어는 다음과 같다.
<br>
새로운 클래스 ColoredShape를 만들고 템플릿 인자로 받은 클래스를 상속받게 한다.
<br>
이때 템플릿 파라미터를 제약할 방법은 없다.
<br>
즉, 어떤 타입이든 올 수 있다.
<br>
<br>
따라서 static_assert를 이용해 Shape 이외의 타입이 지정되는 것을 막는다.

```
template <typename T> struct ColoredShape : T
{
    static_assert(is_base_of<Shape, T>::value,
        "template argument must be a Shape");
        
    string color;
    
    string str() const override
    {
        ostringstream oss;
        oss << T::str() << " has the color " << color;
        
        return oss.str();
    }
}; // TransparentShape<T>의 구현 생략
```

ColoredShape와 TransparentShape의 구현을 기반으로 하여 색상이 있는 투명한 도형을 아래와 같이 합성할 수 있다.

```
ColoredShape<TransparentShape<Square>> square{"blue"};
square.size = 2;
square.transparentcy = 0.5;
cout << square.str();

// 이제 sqaure의 어떤 멤버든 접근 가능
sqaure.resize(3);
```

위 코드는 정상적으로 동작하지만 완벽한 코드는 아니다.
<br>
이유는 모든 생성자를 한 번에 편리하게 호출하던 부분을 잃어버렸기 때문이다.
<br>
<br>
기존 바깥의 클래스는 생성자로 초기화할 수 있지만 도형의 크기, 색상, 투명도까지 한 번에 설정할 수는 없다.

<br>
<br>

데코레이션을 완성하기 위해 ColoredShape와 Transparent에 생성자를 전달한다.
<br>
이 생성자들은 두 종류의 인자를 받는다.
<br>
<br>
첫 번째 인자들은 현재의 템플릿 클래스에 적용되는 것들이다.
<br>
두 번째 인자들은 부모 클래스에 전달될 제너릭 파라미터 팩이다.

```
template <typename T> struct TransparentShape : T
{
    uint8_t transparency;
    
    template <typename...Args>
    TransparentShape(const uint8_t transparency, Args ...args)
        T(std::forward<Args>(args)...)
        , transparency{ transparency } {}
    ...
}; // ColoredShape에서도 동일하게 구현
```

위의 생성자는 임의의 개수를 인자로 받을 수 있다.
<br>
앞쪽 인자는 투명도 값을 초기화하는 데 이용되고 나머지 인자들은 그 인자가 어떻게 구성되었느냐와 관계없이 단순히 상위 클래스에 전달된다.

<br>
<br>

생성자들에 전달되는 인자의 타입과 개수, 순서가 맞지 않으면 컴파일 에러가 발생하기 때문에 올바르게 맞춰질 수밖에 없다.
<br>
<br>
클래스 하나에 디폴트 생성자를 추가하면 파라미터의 설정에 훨씬 더 융통성이 생긴다.
<br>
하지만 인자 배분에 혼란과 모호성이 발생할 수도 있다.
<br>
<br>
이러한 생성자들에 explicit 지정자를 부여하지 않도록 주의해야 한다.
<br>
만약 그렇게 할 경우 복수의 데코레이터를 합성할 때 C++의 복제 리스트 초기화 규칙 위반 오류가 발생한다.
<br>
<br>
이러한 내용들을 아래와 같이 구현할 수 있다.

```
ColoredShape2<TransparentShape2<Square>> sq = { "red", 51, 5 };
cout << sq.str() << endl;
// 출력 결과 "A square with side 5 has 20% transparency has the color red"
```

이것으로 정적 데코레이터의 구현을 모두 다루었다.


<br>


## 함수형 데코레이터(Fuction Decorator)

데코레이터 패턴은 클래스를 적용 대상으로 하는 것이 보통이지만 함수에도 동등하게 적용될 수 있다.
<br>
<br>
예를 들어 코드에 문제를 일으키는 특정 동작이 있다고 가정한다.
<br>
그 동작이 수행될 때마다 모든 상황을 로그로 남겨 엑셀에 옮겨다가 상태를 분석하고 싶은 상황이다.
<br>
<br>
로그를 남기기 위해 아래와 같이 로그를 남긴다.

```
cout << "Entering function\n";

// 작업 수행...
cout << "Exiting function\n";
```





























