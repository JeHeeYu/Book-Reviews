# Chapter_8_사용자 이벤트 처리하기 정리 내용
## 터치 이벤트(Touch Event)
앱의 화면에서 발생하는 사용자 이벤트는 터치다.
<br>
앱은 사용자의 터치를 인식하고 화면을 손가락으로 눌렀는지 떼었는지 스와이프 했는지에 따라 동작한다.
<br>
스와이프란 화면에 손가락을 댄 상태로 쓸어 넘기는 동작을 말한다.
<br>
<br>
앱의 화면에서 발생하는 사용자의 터치 이벤트를 처리할 때<br>
액티비티 클래스에 터치 이벤트의 콜백 함수인 onTouchEvent()를 선언하면 된다.
<pre>
override fun onTouchEvent(event: MotionEvent?): Boolean {
    return super.onTouchEvent(event)
}
</pre>
액티비티에 onTouchEvent() 함수를 재정의해서 선언만 해 놓으면 사용자가 이 액티비티 화면을 터치하는 순간 자동으로 호출된다.
<br>
함수에 전달되는 매개변수는 MotionEvent 객체이며, 이 객체에 터치의 종류와 발생 지점(좌푯값)이 담긴다.
### 터치 이벤트 종류
터치 이벤트는 3가지로 구분된다.
<ul>
<li> Action_DOWN : 화면을 손가락으로 누른 순간의 이벤트 
<li> ACTION_UP : 화면에서 손가락을 떼는 순간의 이벤트
<li> ACTION_MOVE : 화면을 손가락으로 누른 채로 이동하는 순간의 이벤트
</ul>
만약 화면을 손가락으로 살짝 누른 후 떼면 onTouchEvent()가 2번 발생한다.
<br>
<pre>
override fun onTouchEvent(event: MotionEvent?): Boolean {
    when (event?.action) {
        MotionEvent.ACTION_DOWN -> {
            Log.d("Touch", "Touch Down Event")
        }
        MotionEvent.ACTION_UP -> {
            Log.d("Touch", "Touch Up Event")
        }
    }
    return super.onTouchEvent(event)
}
</pre>
### 터치 이벤트 발생 좌표 얻기
터치 이벤트를 처리할 때 이벤트 종류 외 이벤트가 발생한 지점을 알아야 할 때도 있다.br>
<br>
이 좌표도 onTouchEvent() 함수의 매개변수인 MotionEvent 객체로 얻는다.

<ul>
<li> x : 이벤트가 발생한 뷰의 X 좌표
<li> y : 이벤트가 발생한 뷰의 Y 좌표
<li> rawX : 화면의 X 좌표
<li> rawY : 화면의 X 좌표
</ul>
<pre>
override fun onTouchEvent(event: MotionEvent?): Boolean {
    when (event?.action) {
        MotionEvent.ACTION_DOWN -> {
            Log.d("Touch", "Touch Down Event : ${event.x}, rawX : ${event.rawX}")
        }
    }
    return super.onTouchEvent(event)
}
</pre>
x와 rawX 모두 좌푯값이지만 의미하는 바는 다르다.
<br>
<ul>
<li> x : 터치 이벤트가 발생한 뷰에서의 좌푯값
<li> rawX : 화면에서의 좌푯값
</ul>
<img src="https://user-images.githubusercontent.com/87363461/189588735-b082757a-65ad-4404-a497-a8b7d1e2f8bf.JPG" width="200" height="300">
<br>
