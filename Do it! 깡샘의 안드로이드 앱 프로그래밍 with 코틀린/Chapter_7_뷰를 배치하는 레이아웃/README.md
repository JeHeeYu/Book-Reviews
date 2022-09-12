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
<LinearLayout
  android:orientation="horizontal" >
  <LinearLayout
    android:orientation="vertical" >
  </LinearLayout>
</LinearLayout>
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
