# 퍼사드 패턴 정리 내용

## 퍼사드(Facade) 패턴
퍼사드 패턴은 라이브러리, 프레임워크에 대한 또는 다른 클래스들의 복잡한 집합에 단순화된 인터페이스를 제공하는 구조적 디자인 패턴이다.
<br>
운영 체제에서 제공하는 터미널/콘솔의 커맨드 라인 인터페이스와 비슷하듯 미묘한 차이가 있다.

<br>

## 터미널 동작
터미널 윈도의 첫 번째 구성 요소는 버퍼이다.
<br>
버퍼에는 렌더링할 문자들이 저장된다.
<br>
<br>
버퍼는 메모리의 사각형 공간으로 보통 char/wchar_t 타입의 1차원 또는 2차원 배열로 만들어진다.
<br>
버퍼는 콘솔에서 눈에 보여지는 공간보다 훨씬 더 클 수 있다.
<br>
<br>
과거의 출력 히스토리를 저장하고 있어 이전 화면으로 스크롤하여 내용을 다시 확인할 수 있다.

<br>
<br>

버퍼는 보통 현재의 입력 라인을 가리키는 어떤 표시를 가진다. (예를 들어 어떤 정숫값)
<br>
그 표시를 이용해 버퍼가 가득 찼을 때, 항상 새로운 라인을 할당하는 것이 아니라 가장 오래된 라인부터 덮어쓰면서 새로운 데이터를 받아들인다.
<br>
<br>
다음으로 뷰포트(Viewport)라는 개념이 등장한다.
<br>
뷰포트는 버퍼의 특정 영역을 화면에 렌더링한다.
<br>
당연하게 뷰포트는 버퍼의 크기보다 작거나 같다.

<br>
<br>

마지막으로 콘솔(터미널 윈도)가 있다.
<br>
콘솔은 뷰포트 화면에 보여주고 사용자로부터 입력을 받는다.
<br>
콘솔은 퍼사드의 역할이다.
<br>
뒤에서 일어나는 복잡한 작업들을 단순화하여 표현한다.
<br>
<br>
대부분의 경우 사용자는 하나의 버퍼와 하나의 뷰포트를 이용한다.
<br>
하지만 콘솔 윈도를 세로로 분할하여 뷰포트 두 개를 두고 각각의 뷰포트가 자기만의 버퍼를 가지게 할 수도 있다.
<br>
리눅스에서는 screen 커맨드를 이용하여 그러한 콘솔 윈도를 만들 수 있다.

<br>

## 고급 터미널
운영 체제에서 제공하는 전형적인 터미널의 한 가지 문제는 대량 데이터를 파이핑하여 출력할 때 대단히 느리다는 점이다.
<br>
예를 들어 MS 윈도의 터미널 윈도(cmd.exe)는 GDI를 이용해 문자를 렌더링하여 대단히 느리다.
<br>
요즘처럼 전용 그래픽 장치가 기본으로 존재하는 환경에서는 전혀 불필요한 방식이다.
<br>
<br>
빠르게 동작해야 하는 금융 거래 환경에서는 하드웨어의 도움을 받아 렌더링을 한다.
<br>
문자들은 사전에 렌더링된 텍스처로 표현되어 OpenGL과 같은 그래픽 하드웨어용 API를 호출한다.
<br>
<br>
거래 터미널은 여러 개의 버퍼와 뷰포트로 이루어진다.
<br>
전형적인 경우 여러 가지 거래, 교환 데이터들이 동시에 서로 다른 버퍼에서 업데이트된다.
<br>
<br>
이 모든 정보가 하나의 화면에 표현되어야 한다.
<br>
<br>
버퍼는 단순히 1차원 또는 2차원 선형 저장소를 넘어서서 다른 고급 기능들도 제공한다.

<br>
예를 들어 아래의 TableBuffer는 표 형태로 정리하여 출력하는 버퍼이다.

```
struct TableBuffer : IBuffer
{
    TableBuffer(vector<TableColumnSpec> spec, int totalHeight) { ... }
    
    struct TableColumnSpec
    {
        string header;
        int width;
        enum class TableColumnAlignment {
            Left, Center, Right
        } alignment;
    }
};
```


위의 코드는 어떤 주어진 명세에 따라 데이터를 표로 정리하여 화면에 보여준다.
<br>
뷰 포트는 버퍼로부터 데이터를 가져오는 역할을 하며, 다음은 뷰포트가 가지는 속성 중 일부의 속성이다.

- 보여줄 버퍼로의 참조
- 크기
- 뷰포트가 버퍼보다 작으면 버퍼의 어느 지점을 보여줘야 할지 결정
- 전체 콘솔 윈도에서 뷰포트의 위치
- 커서의 위치

## 퍼사드의 위치
예로 위의 예에서 금융 거래 시스템에서는 콘솔 자체가 퍼사드이다.
<br>
콘솔 내부에서 많은 수의 서로 다른 객체들이 동작한다.

```
struct Console
{
    vector<Viewport*> viewports;
    Size charSize, gridSize;
    ...
};
```


콘솔의 초기화 작업은 일반적으로 대단히 복잡하고 손이 많이 간다.
<br>
그럼에도 불구하고 콘솔은 퍼사드이기 때문에 최대한 편리한 API를 제공하기 위해 노력 해야 한다.
<br>
<br>
이를 위해 콘솔 내부의 모든 구성 요소를 초기화하는데 필요한 파라미터들을 직관적으로 이해하기 쉽게 정의한다.

```
Console::Console(bool fullscreen, int char_width, int char_height, int width, int height, optional<Size> client_size)
{
    // 구현부 예 : 
    // 버퍼와 뷰포트를 함께 묶어 생성하여 적절한 컬렉션 객체에 담기
    // 이미지 텍스처 생성
    // 전체 화면 모드 여부에 맞추어 그리드 크기 계산
}
```

다른 방법으로 모든 파라미터는 객체 하나에 묶어서 전달할 수도 있다.
<br>
이렇게 하면 파라미터들의 적절한 디폴트 값을 한 곳에서 관리할 수 있다.

```
Console::Console(const ConsoleCreationParameters& ccp) { ... }
{
    optional<Size> client_size;
    int char_width{10};
    int char_height{14};
    int width{20}; 
    int height{30};
    bool fullscreen{false};
}
```

<br>

## 요약
퍼사드 디자인 패턴은 하나 이상의 복잡한 서브 시스템 앞에 단순한 인터페이스를 두기 위한 방법이다.
<br>
앞의 예에서 다수의 버퍼와 다수의 뷰포트가 연동되는 복잡한 콘솔 또는 단일 버퍼에 단일 뷰포트만 존재하는 단순한 콘솔 등 편리하고 직관적인 API를 통해 이용할 수 있게 하였다.
