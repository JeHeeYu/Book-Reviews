# 팩토리 패턴 정리 내용

## 시나리오

예를 들어 직교 좌표계의 좌표 정보를 지정하려고 한다.
<br>
아래와 같이 간단하게 구현할 수 있다.

```
struct Point
{
    Point(const float x, const float y)
      : x{x}, y{y} {}
    float x, y; // 직교 좌표계 좌푯값
};
```
그리고 만약 극좌표계로 좌푯값을 저장하려면 아래와 같이 할 수 있다.

```
Point(const float r, const float theta)
{
    x = r * cos(theta);
    y = r * sin(theta);
}
```

위의 코드는 문제가 있는데, 생성자도 두 개의 float 값을 파라미터로 하기 때문에 극좌표계의 생성자와 구분할 수 없다.
<br>
한 가지 단순한 구분 방법은 좌표계 종류를 enum 타입 값으로 파라미터에 추가하는 것이다.

```
enum class PointType
{
    cartesian,
    polar
};

Point(float a, float b, PointType type = PointType::cartesian)
{
    if(type == PointType::cartesian)
    {
        x = a;
        y = b;
    }
    else
    {
        y = a * cos(b);
        y = a * sin(b);
    }
}
```

위와 같은 코드는 기능적으로는 문제가 없지만, 좋은 코드라고 할 수 없다.

<br>

## 팩토리 메서드(Factory Method)

시나리오의 예제에서 생성자의 문제는 항상 타입과 같은 이름을 가진다는 것이다.
<br>
즉, 일반적인 함수 이름과 달리 생성자의 이름에는 추가적인 정보를 표시할 수가 없다.
<br>
그리고 생성자 오버로딩으로는 같은 float 타입인 x, y와 theta를 구분할 수 없다.
<br>
<br>
이러한 문제를 해결해야 하는데, 아래 코드는 생성자를 protected로 하여 숨기고, 대신 Point 객체를 만들어 리턴하는 static 함수로 제공하는 예제이다.

```
struct Point
{
protected:
    Point(const float x, const float y)
      : x{x}, y{y} {}
public:
    static Point NewCartesian(float x, float y)
    {
        return { x, y };
    }
    static Point NewPolar(float r, float theta)
    {
        return { r * cos(theta), r * sin(theta) };
    }
};
```

여기서 각각의 static 함수들을 팩토리 메서드라고 부른다.
<br>
이러한 메서드가 하는 일은 Point 객체를 생성하여 리턴하는 것뿐이다.
<br>
<br>
함수의 이름과 좌표 파라미터의 이름 모두 그 의미가 무엇인지, 어떤 값이 인자로 주어져야 하는지 명확하게 표현하고 있다.
<br>
<br>
이제 좌표점을 생성할 때 다음과 같이 명료하게 할 수 있다.

```
auto p = Point::NewPolar(5, M_PI_4);
```

위 코드는 가독성이 매우 좋은 코드이다.
<br>
r = 5이고, theta = 𝝅/4 인 극좌표를 만든다는 것을 바로 알 수 있다.


<br>


## 팩토리(Factory)
빌더와 마찬가지로 Point를 생성하는 함수들을 별도의 클래스에 몰아넣을 수 있다.
<br>
그러한 클래스를 팩토리라고 부른다.
<br>
<br>
팩토리 클래스를 만들기 위해 아래와 같이 Point 예제 클래스가 있다고 가정한다.

```
struct Point
{
    float x, y;
    friend class PointFactory;
private:
    Point(float x, float y) : x(x), y(y) {}
};
```

- Point의 생성자는 private으로 선언되어 사용자가 직접 생성자를 호출하는 경우가 없게 한다.<br>이 부분은 필수 사항은 아니지만 같은 일을 하는데 두 가지 방벙블 제공하여 사용자를 혼란스럽게 할 필요는 없음













