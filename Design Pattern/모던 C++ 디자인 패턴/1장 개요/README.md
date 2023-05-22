# 1장 개요 정리 내용

## 1. 중요한 개념

### 1-1. 이상한 재귀 탬플릿 패턴(CRTP, Curiously Recurring Template Pattern)

이 패턴은 하나의 패턴이지만, 디자인 패턴이라고 하기에는 어렵지만 C++ 세계에서의 코드 패턴 중 하나이다.
<br>
이 패턴은 아이디어는 단순한데, 자기 자신을 베이스 클래스의 템플릿 인자로 상속받는 것이다.

<br>

```
struct Foo : SomeBase<Foo>
{
    ...
}
```

<br>

이러한 이유로 상속받는 한 가지 이유는 베이스 클래스의 구현부에서 타입이 지정된 this 포인터를 사용할 수 있게 된다는 것 때문이다.
<br>
<br>
예를 들어 SomeBase를 상속받는 모든 서브 클래스들이 begin() / end() 메서드 쌍을 구현한다고 가정해 보자.
<br>
<br>
SomeBase의 메서드 안에서 서브 클래스의 객체를 순회할 방법이 있을까 생각해 보면, 부모 클래스인 SomeBase에 begin() / end() 인터페이스가 정의되어 있지 않은 한 불가능하다.
<br>
하지만 CRTP를 적용하면 this를 서브 클래스 타입으로 캐스팅할 수 있다.

<br>

```
template <typename Derived>
struct SomeBase
{
    void foo()
    {
        for(auto& item : *static_cast<Derived*>(this))
        {
            ...
        }
    }
}
```

<br>

### 1-2. 첨가(Mixin) 상속
C++에서 클래스를 정의할 때 자기 자신을 템플릿 인자로 할 수 있다.
<br>
예를 들어 다음과 같다.

<br>

```
template <typename T> struct Mixin : T
{
    ...
}
```

<br>

이러한 방식을 첨가 상속(Mixin Inheritance)라고 한다.
<br>
첨가 상속을 이용하면 계층적으로 여러 타입을 형성할 수 있다.
<br>
<br>
예를 들어, 아래와 같이 변수를 선언했다.

```
Foo<Bar<Baz>> x;
```

이렇게 변수를 선언하면 새로운 타입 FooBarBaz를 따로 구현하지 않고도 세 가지 타입의 속성을 가지게 할 수 있다.
  

<br>

### 1-3. 속성

get/set 메서드를 가지는 클래스 내부 변수(보통 private으로 선언되는)를 보통 그 클래스의 속성이라고 부른다.
<br>
표준 C++에서는 보통 다음과 같은 형태의 속성을 표현한다.

```
class Person
{
    int age;

public:
    int get_age() const { return age; }
    void set_age(int value) { age = value; }
};
```

많은 프로그래밍 언어들(C# 또는 코틀린 등)이 속성을 언어 자체의 내장 기능으로 제공한다.
<br>
하지만 C++은 그렇지 않다.
<br>
<br>
그럼에도 대부분의 컴파일러(MSVC, Clang, Intel)에서 비표준적인 방법으로 지원하고 있다.
<br>
<br>
즉, 아래와 같이 코드를 작성하면

```
class Person
{
    int age_;
    
public:
    int get_age() const { return age_; }
    void set_age(int value) { age_ = value; }
    __declspec(property(get=get_ace, put=set_age)) int age;
};
```


아래와 같이 사용할 때 자동으로 get/set 메서드가 호출된다.

```
Person person;
p.age = 20; // calls p.set_age(20)
```


<br>


## 2. SOLID 디자인 원칙

SOLID는 다음과 같은 디자인 원칙들을 아우르는 약어이다.

- 단일 책임 원칙(SRP, Single Responsibility Principle)
- 열림-닫힘 원칙(OCP, Open-Closed Principle)
- 리스코프 치환 법칙(LSP, Liskov Substitution Principle)
- 인터페이스 분리 법칙(Interface Segregation Principle)
- 의존성 역전 원칙(Dependency Inversion Principle)

이 원칙들은 2000년대 초 로버트 마틴(Robert C. Martin)에 의해 소개된 원칙이다.
<br>
<br>
사실 이 다섯 가지 원칙은 로버트 저서와 블로그에서 소개된 수십 가지 원칙 중에서 선정된 것이다.

### 2-1. 단일 책임 원칙(SRP, Single Responsibility Principle)

예를 들어 아래와 같이 메모장과 관련된 기능을 하는 예제 코드가 있다.

```
struct Journal
{
    string title;
    vector<string> entries;
    
    explicit Journal(const string& title) : title{title} }|
}
```

이제 메모장의 각 항목이 기입된 순서대로 저장되게 하는 기능을 만들 수 있다.

```
void Journal::add(const string& entry)
{
    static int count = 1;
    entries.push_back(boost::lexical_cast<string>(count++)
      + ": " + entry);
}
```

이제 다음과 같이 메모장을 이용할 수 있다.

```
Journal j{"Dear Diary"};
j.add("I cried today");
j.add("I ate a bug");
```

이 함수가 Journal 클래스에 포함되는 것은 매우 자연스럽고 상식적이다.
<br>
이유는 각 항목을 기록할 수 있게 하고 관리할 책임이 메모장에 있기 때문이다.
<br>
<br>
따라서 그러한 책임을 완수하기 위한 함수라면 Journal에 포함되는 것이 공정하다.
<br>
<br>
이제 메모장에 영구적으로 파일에 저장하는 기능을 마늘었다.

```
void Journal::save(const string& filename)
{
    ofstream ofs(filename);
    for (auto& s : entries)
        ofs << s << endl;
}
```

이러한 방식은 문제가 있는데, 메모장의 책임은 메모 항목들을 기입/관리하는 것 이지 디스크에 쓰는 것이 아니다.
<br>
만약 디스크에 파일을 쓰는 기능을 데이터의 기입/관리를 담당하는 클래스가 함께 책임지도록 한다면 데이터 저장 방식이 바뀔 때(예를 들어 로컬 디스크 대신 원격 클라우드에 저장)마다 그러한 클래스들을 일일이 모두 수정해야 한다.
<br>
<br>
따라서 이 예제에서의 파일 저장 기능은 메모장과 별도로 취급하여 별도의 클래스로 만드는 것이 바람직하다.
<br>
예를 들어 다음과 같이 만들 수 있다.

```
struct PersistenceManger
{
    static void save(const Journal& j, const string& filename)
    {
        ofstream ofs(filename);
        for(auto& s : j.entries)
            ofs << s << endl;
    }
}
```

단일 책임 원칙이 의미하는 바가 정확히 바로 이런 것이다.
<br>
각 클래스는 단 한 가지의 책임을 부여받아, 수정할 이유가 단 한가지 이유여야 한다.
<br>
<br>
Journal 클래스는 기록할 항목에 대해 뭔가 바꿀 것이 있을 때 코드 수정이 되어야 한다.
<br>
예를 들어 각 항목이 날짜와 시간을 접두어로 가지게 해야 한다면 add() 함수를 그렇게 하도록 수정해야 한다.
<br>
반면에 영구적인 저장 방식을 바꾸어야 한다면 PersistenceManager를 수정해야 한다.

<br>
<br>

SRP를 위배하는 안티 패턴은 극단적인 예로 전지전능 객체가 있다.
<br>
전지전능 객체는 가증한 많은 기능을 담아 하나의 괴물 같은 클래스를 이룬다.

### 2-2. 열림-닫힘 원칙(OCP, Open-Closed Principle)

예를 들어 데이터베이스에 어떤 제품군에 대한 정보가 저장 되어 있으며 개별 제품은 서로 다른 색상과 크기를 가지며 아래와 같이 정의한다.

```
enum class Color { Red, Green, Blue };
enum class Size { Small, Medium, Large };

struct Product
{
    string name;
    Color color;
    Size size;
};
```

이제 주어진 제품 집합을 조건에 따라 필터링하는 기능을 만들고 싶다고 할 때 다음과 같이 조건에 합치는 제품들의 집합을 가지도록 구현할 수 있다.

```
struct ProductFilter
{
    typedef vector<Product*> Items;
}
```

이제 필터링 조건으로 색상을 기준으로 삼는 필터를 만들려고 한다.
<br>
이 경우 색상만을 기준으로 제품을 구분하는 멤버 함수를 아래와 같이 정의할 수 있다.

```
ProductFilter::Items ProductFilter::by_color(Items items, Color color)
{
    Items result;
    
    for(auto& i : items)
        if(i->color == color)
            result.push_back(i);
            
    return result;
}
```

이렇게 색상을 기준으로 필터링하는 접근방법은 잘 동작하고 나쁘지 않다.
<br>
<br>
이러한 코드가 상용 제품으로 배포되었을 때, 문제가 생긴다.
<br>
어느 정도 시간이 흐른 후, 크기를 기준으로 한 필터링 기능도 구현해 달라는 요구 사항이 들어올 수 있다.
<br>
<br>
이 요구 사항을 구현하기 위해 ProductFilter.cpp 파일을 찾아 다시 코드를 추가한다.

```
ProductFilter::Items ProductFilter::by_size(Items items, Size size)
{
    Items result;
    
    for(auto& i : items)
        if(i->size == size)
            result.push_back(i);
            
    return result;
}
```

두 멤버 함수 모두 코드가 흡사하며, 뭔가 같은 작업을 반복하는 느낌이다.
<br>
여기서 범용적으로 임의의 조건을 지정받는 필터 함수를 만들지 않는 이유가 있다.
<br>
<br>
첫 번째 이유는 필터 조건마다 처리 형태가 다를 수 있기 때문이다.
<br>
예를 들어 어떤 레코드 타입은 인덱싱으로 접근되어 특정한 방식으로 탐색 되어야 하고, 어떤 레코드는 GPU에서 병렬로 탐색이 가능할 수도 있다.
<br>
또 다른 레코드 타입들은 어느 것에도 해당하지 않을 수 있다.

<br>
<br>
또 다시 사용자가 다른 요구사항을 전달 했는데, 이번에는 색상과 크기를 모두 지정하여 필터링 하길 원한다.
<br>
아래와 같이 함수를 다시 추가한다.

```
ProductFilter::Items ProductFilter::by_color_and_size(Items items, Color color, Size size)
{
    Items result;
    
    for(auto& i : items)
        if(i->color == color && i->size == size)
            result.push_back(i);
            
    return result;
}
```

이러한 시나리오에서 필요한 것이 바로 열림-닫힘 원칙이다.
<br>
열림-닫힘 원칙은 타입이 확장에는 열려 있지만 수정에는 닫혀 있도록 강제하는 것을 뜻한다.
<br>
<br>
다르게 말해, 기존 코드의 수정 없이(이미 배포된 잘 동작하던 코드를 다시 컴파일 할 필요 없이) 필터링을 확장할 수 있는 방법이 필요하다.
<br>
<br>
이렇게 하기 위한 방법으로 먼저, 필터링 절차를 개념적으로 두 개의 부분으로 나누어야 한다.(SRP 원칙 적용)
<br>
첫 번째는 '필터' 이고(항목 집합을 넘겨받아 그 일부를 리턴) 다른 하나는 '명세' 이다.(데이터 항목을 구분하기 위한 조건 정의)
<br>
<br>
'명세'는 다음과 같이 매우 단순하게 정의할 수 있다.

```
teplate <typename T> struct Specification
{
    virtual bool is_satisfied(T* item) = 0;
}
```

다음으로 Specification<T>에 기반하여 필터링을 수행할 방법이 필요한데, 이를 위해 다음과 같이 Filter<T>를 정의한다.
  
```
template <typename T> struct Filter
{
    virtual vector<T*> filter(
        vector<T*> items,
        Specification<T>& spec) = 0;
};
```

동일하게 filter라는 이름의 함수를 만들고 전체 항목의 집합과 명세를 인자로 받아 명세에 합치되는 항목들을 리턴한다.
<br>
여기서는 항목들이 vector<T*>에 저장된다고 가정하고 있다.
<br>
<br>
하지만 실제 환경에서는 좀 더 유연하게 임의의 컬렉션 타입을 지원할 수 있도록, 반복자 또는 별도로 준비한 인터페이스를 filter()에 넘겨줄 수도 있다.
<br>
<br>
이러한 준비를 기반으로 하면 다음과 같이 개선된 필터를 쉽게 구현할 수 있다.

```
struct BetterFilter : Filter<Product>
{
    vector<Product*> filter(
        vector<Product*> items,
        Specification<Product>& spec) override
    {
        vector<Product*> result;
        for(auto& p : items)
            if(spec.is_satisfied(p))
                result.push_back(p);
        result result;
    }
}
```

인자로 전달되는 Specification<T>는 타입이 강하게 규정된(가능한 필터 명세 개수가 제한된) std::function 이라고 볼 수 있다.
<br>
<br>
이렇게 OCP 에서는 기존에 작성하고 테스트했던 코드에 다시 손을 대서는 안된다.

  
  <br>

  
  
### 2-3.

















