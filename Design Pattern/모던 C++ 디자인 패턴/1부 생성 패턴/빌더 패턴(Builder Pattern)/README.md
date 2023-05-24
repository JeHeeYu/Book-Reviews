# 빌더 패턴 정리 내용

## 빌더(Build) 패턴
빌더 패턴이란 생성이 까다로운 객체를 쉽게 처리하기 위한 패턴을 말한다.
<br>
즉, 생성자 호출 코드 단 한 줄로 객체를 생성할 수 없는 객체를 다룬다.
<br>
<br>
그러한 타입의 객체들은 조합으로 구성되거나, 상식적인 것을 벗어난 까다로운 로직을 필요로 한다.
<br>


## 시나리오
예를 들어 웹 페이즈를 그리기 위한 컴포넌트들을 생성해야 한다.
<br>
먼저 단순하게 단어를 나타내는 항목 두 개를 html의 비순차('< ul >') 리스트('< li >') 태그로 출력해 보면 아래와 같이 간단하게 출력할 수 있다.

```
string words[] = { "hello", "world" };
ostring oss;
oss << "<ul>";
for(auto w : words)
    oss << " <li>" << w << "</li>";
oss << 
```
이 코드는 목적한 대로 출력을 하기는 하나, 융통성이 없는 코드이다.
<br>
항목마다 앞에 점을 찍거나 순서대로 번호를 매겨야 한다면 이 코드를 손쉽게 수정하기는 어렵다.
<br>
<br>
대안으로 객체 지향(OOP) 스타일을 적용할 수 있는데, HtmlElement 클래스를 정의하여 각 html 태그에 대한 정보를 저장한다.

```
struct HtmlElement
{
    string name;
    string text;
    vector<HtmlElement> elements;
    
    HtmlElement() {}
    HtmlElement(const string& name, const string& text)
      : name(name), text(text) { }
      
    string str(int indent = 0) const
    {
        //  컨텐츠를 양식에 맞추어 출력
    }
};
```

이러한 접근 방법을 활용하여 출력 양식이 좀 더 쉽게 드러나도록 리스트를 생성할 수 있다.

```
string words[] = { "hello", "world" };
HtmlElement list{"ul", ""};
for(auto w : words)
    list.elements.emplace_back{HtmlElement{"li", w}};
printf(list.str().c_str());
```

이 코드는 OOP에 기반하여 항목 리스트를 표현하고 있다.
<br>
양식을 제어하기가 좀 더 쉬우면서도 목적하는 출력을 할 수 있다.
<br>
<br>
하지만 각각의 HtmlElement를 생성하는 작업이 편하다고 볼수 없는데, 이 부분을 빌더 패턴을 활용하여 개선할 수 있다.

<br>

## 단순한 빌더
