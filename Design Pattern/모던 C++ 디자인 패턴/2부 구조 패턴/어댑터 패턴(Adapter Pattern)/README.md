# 어댑터 패턴 정리 내용

## 어댑터(Adapter) 패턴
어댑터 패턴은 클래스의 인터페이스를 사용자가 기대하는 다른 인터페이스로 변환하는 패턴으로, 호환성이 없는 인터페이스 때문에 함께 동작할 수 없는 클래스들이 함께 작동하도록 해준다.

## 시나리오
예를 들어 픽셀을 그리는데 적합한 어떤 그리기 라이브러리가 있고 이 라이브러리를 사용해야만 그림을 그릴 수 있다고 가정한다.
<br>
하나하나에 점을 찍는 방식으로 무엇이든 그릴 수는 있지만 기하학적 도형을 그리기에는 너무 저수준 작업이다.
<br>
따라서 기하학적 도형을 픽셀 기반 표현으로 바꾸어주는 어댑터가 필요하다.
<br>
<br>
먼저 아래와 같이 그리기 객체를 정의한다

```
struct Point
{
    int x, y;
};

struct Line
{
    Point start, end;
};
```

이제 기하학적 도형을 모두 담을 수 있도록 일반화 한다.
<br>
가장 일반적인 방법은 선분의 집합으로 표현하는 것이다.
<br>
<br>
아래와 같이 퓨어 버추얼 반복자 메서드의 쌍으로 정의한다.
  
```
struct VectorObject
{
    virtual std::vector<Line>::iterator begin() = 0;
    virtual std::vector<Line>::iterator end() = 0;
};
```

이렇게 하면 사각형 Rectangle을 다음과 같이 그 시작점과 크기를 입력받아 생성하고, 사각형을 구성하는 선분들을 vector 타입 필드에 저장하여 그 꼭짓점들만 노출하는 방식으로 정의할 수 있다.

```
struct VectorRectangle : VectorObject
{
    VectorRectangle(int x, int y, int width, int height)
    {
        lines.emplace_back(Line{ Point{x, y}, Point{x + width, y} });
        lines.emplace_back(Line{ Point{x + width, y}, Point{x + width, y + height} });
        lines.emplace_back(Line{ Point{x, y}, Point{x, y + height} });
        lines.emplace_back(Line{ Point{x, y + height}, Point{x + width, y + height} });
    }
    
    std::vector<Line>::iterator begin() override 
    {
        return lines.begin();
    }
    
    std::vector<Line>::iterator end() override
    {
        return lines.end();
    }
    
private:
    std::vector<Line> lines;
};
```

이제 이 코드를 이용해 화면에 선분을 비롯한 사각형을 그리고 싶다고 가정한다.
<br>
안타깝게도 이 코드로 그릴 수 없는데, 그림을 그리기 위한 인터페이스는 아래와 같은 함수 하나 뿐이다.

```
void DrawPoints(CPaintDC& dc, std::vector<Point>::iterator start, std::vector<Point>::iterator end)
{
    for(auto i = start; i != end; ++i)
    {
        dc.SetPixel(i->x, i->y, 0);
    }
}
```

위코드는 마이크로소프트 MFC 라이브러리의 CPaintDC 클래스이다.
<br>

그리기 인터페이스는 점을 찍는 것밖에 없지만, 선분을 그려야 한다.
<br>
<br>
이러한 이유로 어댑터가 필요


<br>

## 어댑터(Adapter)
사각형 몇 개를 그려야 하는 상황이다.

```
vector<shared_ptr<VectorObject>> vectorObjects {
    make_shared<VectorRectangle>(10, 10, 100, 100),
    make_shared(VectorRectangle>(30, 30, 60, 60)
}
```

이 객체들을 그리기 위해서는 사각형을 이루는 선분의 집합에서 각각의 선분마다 많은 수의 점이 반환되어야 한다.
<br>
이를 위해 별도의 클래스를 만든다.
<br>
<br>
이 클래스는 선분 하나를 점의 집합을 만들어 저장하고 각 점들을 순회할 수 있도록 반복자로 노출한다.

```
struct LineToPointAdapter
{
    typedef vector<Point Points;
    
    LineToPointAdapter(Line& line) { }
    
    virtual Points::iterator begin() { return points.begin(); }
    virtual Points::iterator end() { return points.end(); }
    
private:
    Points points;
};
```

선분을 점의 집합으로 변환하는 작업은 생성자에서 일어난다.
<br>
즉, 이 어댑터는 '성급한 접근법' 을 취한다.
<br>
실제 변환 코드는 복잡하지 않다.

```
LineToPointAdapter(Line& line)
{
    int left = min(line.start.x, line.end.x);
    int right = max(line.start.x, line.end.x);
    int top = min(line.start.y, line.end.y);
    int botton = max(line.start.y, line.end.y);
    int dx = right - left;
    int dy = line.end.y - line.start.y;
    
    // 가로 또는 세로 선분들
    if(dx == 0)
    {
        // 세로
        for(int y = top; y <= bottom; ++y)
        {
            points.emplace_back(Point{ left, y });
        }
    }
    else if(dy == 0)
    {
        for(int x = left; x <= right; ++x)
        {
            points.emplace_back(Point{ x, top });
        }
    }
}
```

위 코드는 매우 쉽다.
<br>
수직, 수평, 선분만 다루고 나머지는 무시한다.
<br>
<br>
이 어댑터를 이용하면 몇몇 기하 도형들을 그릴 수 있다.
<br>
앞서 예제 코드에서 정의한 사각형 두 개를 아래와 같이 단순한 코드로 그릴 수 있다.

```
for(auto& obj : vectorObjects)
{
    for(auto& line : *obj)
    {
        LineToPointAdapter lpo{ line };
        DrawPoints(dc, lpo.begin(), lpo.end());
    }
}
```

이 코드는 기하 도형에서 선분 집합을 정의하면 그 선분들로 LineToPointAdapter를 생성하여 점들의 집합으로 변환하고, 그 점둘을 순회할 수 있는 시작 반복자와 끝 반복자를 DrawPoints에 넘겨주어 그림을 그린다.

<br>

## 일시적 어댑터



























