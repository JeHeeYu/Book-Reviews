# Chapter_7_뷰를 배치하는 레이아웃 정리 내용
## LinearLayout - 선형으로 배치
LinearLayout은 뷰를 가로나 세로 방향으로 나열하는 레이아웃 클래스이다.
<br>
orientation에 horizontal 또는 vertical 값으로 방향을 지정한다.
<br>
<br>
LinearLayout은 방향만 설정하면 뷰를 추가한 순서대로 나열한다.
<br>
혹시라도 화면에서 벗어나도 줄을 자동으로 바꾸지 않는다.
<br>
<br>
가로세로 중첩된 구조는 LinearLayout을 중첩해서 사용해야 한다.
<br>
레이아웃 클래스도 뷰이므로 다른 레이아웃 클래스에 포함할 수 있다.
<br>
<img src="https://user-images.githubusercontent.com/87363461/189577438-e7e33d22-24e7-44c8-bcdd-b0ac13e4b3fa.JPG" width="300" height="200">
<br>
가로 방향으로 배치하는 LinearLayout에 세로로 배치하는 LinearLayout을 추가할 수 있다.
<pre>
android:orientation="horizontal"
    android:orientation="vertical"
</pre>
### layout_width - 여백을 채우는 속성
layout_weight 속성은 화면에 생기는 여백을 뷰로 채울 수 있는 속성이다.
<br>
layout_weight 속성에 숫자를 지정할 수 있는데, 이 숫자는 가중치를 나타낸다.
<br>
즉, 뷰가 차지하는 여백의 비율을 의미한다.
<pre>
android:layout_width="1"
</pre>
### layout_gravity - 뷰를 정렬하는 속성
layout_graivity 속성은 뷰를 정렬하는 속성으로, 기본값은 left/top이다.
<br>
대각선 방향 지정 시 | 기호를 이용해 연결한다.
<pre>
android:gravity="right|bottom"
android>layout_gravity="center_horizontal"
</pre>
gravity와 layout_gravity는 모두 뷰를 정렬하는 속성이지만, 정렬 대상이 다르다.
<br>
<b>gravity</b> : 콘텐츠를 대상으로 정렬
<b>layout_gravity</b> : 뷰 자체를 정렬

[[그래비티 속성 예제]](https://github.com/JeHeeYu/Book-Reviews/tree/main/Do%20it!%20%EA%B9%A1%EC%83%98%EC%9D%98%20%EC%95%88%EB%93%9C%EB%A1%9C%EC%9D%B4%EB%93%9C%20%EC%95%B1%20%ED%94%84%EB%A1%9C%EA%B7%B8%EB%9E%98%EB%B0%8D%20with%20%EC%BD%94%ED%8B%80%EB%A6%B0/Chapter_7_%EB%B7%B0%EB%A5%BC%20%EB%B0%B0%EC%B9%98%ED%95%98%EB%8A%94%20%EB%A0%88%EC%9D%B4%EC%95%84%EC%9B%83/%EA%B7%B8%EB%9E%98%EB%B9%84%ED%8B%B0%20%EC%86%8D%EC%84%B1%20%EC%98%88%EC%A0%9C)
<br>
<br>
[결과 화면]
<br>
<img src="https://user-images.githubusercontent.com/87363461/189579466-c1e8eefe-8264-4a6f-9760-c0d53547f82d.JPG" width="200" height="400">
## RelativeLayout - 상대 위치로 배치
RelativeLayout은 상대 뷰의 위치를 기준으로 정렬하는 레이아웃 클래스다.
<br>
즉, 화면에 이미 출력된 특정 뷰를 기준으로 방향이 지정하여 배치한다
<br>
이때 속성에 위치를 정해야 하고, 입력하는 값은 기준이 되는 뷰의 id이다.
<pre>
android:layout_above      // 기준 뷰의 위쪽에 배치
android:layout_below      // 기준 뷰의 아래쪽에 배치
android:layout_toLeftOf   // 기준 뷰의 왼쪽에 배치
android:layout_toRightOf  // 기준 뷰의 오른쪽에 배치
</pre>
### align - 맞춤 정렬하는 속성
align 속성은 상대 뷰의 어느 쪽에 맞춰서 정렬할지 정하는 속성으로,
<br>
기준이 되는 뷰의 id값으로 설정한다.
<pre>
android:layout_alignTop           // 기준 뷰와 위쪽을 맞춤
android:layout_alignBotton        // 기준 뷰와 아래쪽을 맞춤
android:layout_alignLeft          // 기준 뷰와 왼쪽을 맞춤
android:layout_alignRight         // 기준 뷰와 오른쪽을 맞춤
android:layout_alignBaseline      // 기준 뷰와 텍스트 기준선을 맞춤

android:layout_alignParentTop           // 부모의 위쪽에 맞춤
android:layout_alignParentBotton        // 부모의 아래쪽에 맞춤
android:layout_alignParentLeft          // 부모의 왼쪽에 맞춤
android:layout_alignParentRight         // 부모의 오른쪽에 맞춤
android:layout_centerHorizontal         // 부모의 가로 방향 중앙에 맞춤
android:layout_centerVertical           // 부모의 세로 방향에 주앙에 맞춤
android:layout_centerInParent           // 부모의 가로, 세로 중앙에 맞춤
</pre>
[[그래비티 속성 예제]](https://github.com/JeHeeYu/Book-Reviews/tree/main/Do%20it!%20%EA%B9%A1%EC%83%98%EC%9D%98%20%EC%95%88%EB%93%9C%EB%A1%9C%EC%9D%B4%EB%93%9C%20%EC%95%B1%20%ED%94%84%EB%A1%9C%EA%B7%B8%EB%9E%98%EB%B0%8D%20with%20%EC%BD%94%ED%8B%80%EB%A6%B0/Chapter_7_%EB%B7%B0%EB%A5%BC%20%EB%B0%B0%EC%B9%98%ED%95%98%EB%8A%94%20%EB%A0%88%EC%9D%B4%EC%95%84%EC%9B%83/RelativeLayout%20%EC%98%88%EC%A0%9C)
<br>
<br>
[결과 화면]
<br>
<img src="https://user-images.githubusercontent.com/87363461/189580988-547891ae-3585-4329-b0f9-d328c2524372.JPG" width="200" height="400">
## FrameLayout - 겹쳐서 배치
FrameLayout은 뷰를 겹쳐서 출력하는 레이아웃 클래스이다.
<br>
카드를 쌓듯이 뷰를 추가한 순서대로 위에 계속 겹쳐서 출력하는 레이아웃이다.
<br>
<br>
FrameLayout은 가로세로 방향으로 배치하지 않고, 상대 위치를 조절하는 속성도 없다.
<br>
단순히 겹쳐서 출력하는 레이아웃이므로 특별한 속성이 없다.
<br>
<br>
FrameLayout은 똑같은 위치에 여러 뷰를 겹쳐서 놓고, 어떤 순간에 하나의 뷰만 출력할 때 사용한다.
<br>
뷰의 표시 여부를 설정하는 visibility 속성과 함께 사용한다.
[[FrameLayout 예제]](https://github.com/JeHeeYu/Book-Reviews/tree/main/Do%20it!%20%EA%B9%A1%EC%83%98%EC%9D%98%20%EC%95%88%EB%93%9C%EB%A1%9C%EC%9D%B4%EB%93%9C%20%EC%95%B1%20%ED%94%84%EB%A1%9C%EA%B7%B8%EB%9E%98%EB%B0%8D%20with%20%EC%BD%94%ED%8B%80%EB%A6%B0/Chapter_7_%EB%B7%B0%EB%A5%BC%20%EB%B0%B0%EC%B9%98%ED%95%98%EB%8A%94%20%EB%A0%88%EC%9D%B4%EC%95%84%EC%9B%83/FrameLayout%20%EC%98%88%EC%A0%9C)
<br>
<br>
[결과 화면]
<br>
<img src="https://user-images.githubusercontent.com/87363461/189582041-2ff6e641-af49-4792-ba8c-1f29477f1aa3.JPG" width="200" height="400">
<img src="https://user-images.githubusercontent.com/87363461/189582093-67be86b8-201a-4fd8-926d-d4e511f0d26e.JPG" width="200" height="400">
<img src="https://user-images.githubusercontent.com/87363461/189582112-50b24810-7ff9-4d0a-a55e-e869760d268d.JPG" width="200" height="400">
## GridLayout - 표 형태로 배치
GridLayout은 행과 열로 구성된 테이블 화면을 만드는 레이아웃 클래스다.
<br>
orientation 속성으로 가로 세로 방향으로 뷰를 나열하는데, 줄바꿈을 자동으로 해준다.
<br>
<br>
orientation으로 방향을 설정하고 rowCount나 columnCount로 설정한 개수만큼 뷰를 추가한다.
<pre>
orientation : 방향 설정
rowCount : 세로로 나열할 뷰 개수
columnCount : 가로로 나열할 뷰 개수
</pre>
GridLayout에 layout_row나 layout_column 속성을 이용해 특정 뷰의 위치를 조정할 수 있다.
<pre>
layout_row : 뷰가 위치하는 세로 방향 인덱스
layout_column : 뷰가 위치하는 가로 방향 인덱스
</pre>
특정 뷰의 크기를 확장해야 할 때는 layout_gravity 속성을 이용한다.
<pre>
layout_gravity="fill_horizontal"
</pre>
열이나 행을 병합하기 위해  layout_columnSpan, layout_rowSpan 속성을 이용한다.
<pre>
layout_columnSpan : 가로로 열 병합
layout_rowSpan : 세로로 행 병합
</pre>
[[GridLayout 예제]](https://github.com/JeHeeYu/Book-Reviews/tree/main/Do%20it!%20%EA%B9%A1%EC%83%98%EC%9D%98%20%EC%95%88%EB%93%9C%EB%A1%9C%EC%9D%B4%EB%93%9C%20%EC%95%B1%20%ED%94%84%EB%A1%9C%EA%B7%B8%EB%9E%98%EB%B0%8D%20with%20%EC%BD%94%ED%8B%80%EB%A6%B0/Chapter_7_%EB%B7%B0%EB%A5%BC%20%EB%B0%B0%EC%B9%98%ED%95%98%EB%8A%94%20%EB%A0%88%EC%9D%B4%EC%95%84%EC%9B%83/GridLayout%20%EC%98%88%EC%A0%9C)
<br>
<br>
[결과 화면]
<br>
<img src="https://user-images.githubusercontent.com/87363461/189584766-1bfd98f4-1f53-4282-873f-d5ebf3a53912.JPG" width="200" height="400">
## 마무리 실습
[[마무리 실습]](https://github.com/JeHeeYu/Book-Reviews/tree/main/Do%20it!%20%EA%B9%A1%EC%83%98%EC%9D%98%20%EC%95%88%EB%93%9C%EB%A1%9C%EC%9D%B4%EB%93%9C%20%EC%95%B1%20%ED%94%84%EB%A1%9C%EA%B7%B8%EB%9E%98%EB%B0%8D%20with%20%EC%BD%94%ED%8B%80%EB%A6%B0/Chapter_7_%EB%B7%B0%EB%A5%BC%20%EB%B0%B0%EC%B9%98%ED%95%98%EB%8A%94%20%EB%A0%88%EC%9D%B4%EC%95%84%EC%9B%83/%EB%A7%88%EB%AC%B4%EB%A6%AC%20%EC%8B%A4%EC%8A%B5)
<br>
<br>
[결과 화면]
<br>
<img src="https://user-images.githubusercontent.com/87363461/189586956-787ad16c-e2e3-41c4-b631-d4a165578db0.JPG" width="200" height="400">
