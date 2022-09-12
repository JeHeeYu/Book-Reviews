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

터치 이벤트를 처리할 때 이벤트 종류 외 이벤트가 발생한 지점을 알아야 할 때도 있다.
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

## 키 이벤트(Key Event)
키 이벤트는 사용자가 폰의 키를 누르는 순간에 발생한다.
<br>
액티비티에서 키 이벤트를 처리하려면 키 이벤트 관련 콜백 함수를 정의해야 한다.
<ul>
<li> onKeyDown : 키를 누르는 순간의 이벤트
<li> onKeyUp : 키를 떼는 순간의 이벤트
<li> onKeyLongPress : 키를 오래 누르는 순간의 이벤트
</ul>
<pre>
override fun onKeyDown(keyCode: Int, event: KeyEvent?): Boolean {
    KeyEvent.KEYCODE_0 -> Log.d("Key", "0 Key")
    KeyEvent.KEYCODE_0 -> Log.d("Key", "A Key")
    return super.onKeyDown(keyCode, event)
}
override fun onKeyUp(keyCode: Int, event: KeyEvent?): Boolean {
    KeyEvent.KEYCODE_0 -> Log.d("Key", "0 Key")
    KeyEvent.KEYCODE_0 -> Log.d("Key", "A Key")
    return super.onKeyUp(keyCode, event)
}
</pre>
키 이벤트 함수의 첫 번째 매개변수는 키의 코드이며 이 값으로 사용자가 어떤 키를 눌렀는지 식별할 수 있다.
<br>
<br>
그런데 이러한 키 이벤트가 발생하는 키는 폰에서 제공하는 소프트 키보드의 키를 의미하지 않는다.
<br>
그래서 소프트 키보드 키를 눌렀을 때 키 이벤트는 발생하지 않는다.
<br>
앱에서 글을 입력할 때 화면 아래에서 올라오는 키보드를 소프트 키보드(Soft Keyboard)라고 한다.
<br>
<img src="https://user-images.githubusercontent.com/87363461/189632049-46a381bb-11d1-4cc9-a6a9-6a85bc2238d1.JPG" width="250" height="250">
<br>
즉, 키 이벤트는 안드로이드의 하드웨어 키의 이벤트를 처리할 수 있다.
<br>
안드로이드의 하드웨어 키는 <b>뒤로가기, 홈, 오버뷰(이 세 가지를 합쳐서 내비게이션 바), 전원 버튼, 볼륨 조절</b> 버튼이 있다.
<br>
<img src="https://user-images.githubusercontent.com/87363461/189632560-9ef027a2-36f8-4965-89f8-9946beaf4e10.JPG" width="200" height="400">
<br>
이 중에서, 전원 버튼과 볼륨 조절 버튼에 대한 이벤트를 키 이벤트로 취급해 처리할 수 있다.
<pre>
override fun onKeyDown(keyCode: Int, event: KeyEvent?): Boolean {
    KeyEvent.KEYCODE_BACK -> Log.d("Key", "Back Key")
    KeyEvent.KEYCODE_VOLUME_UP -> Log.d("Key", "Up Key")
    KeyEvent.KEYCODE_VOLUME_DOWN -> Log.d("Key", "Down Key")
    return super.onKeyDown(keyCode, event)
}
</pre>
특별히 뒤로가기 버튼만 따로 이벤트가 존재한다.
<pre>
override fun onBackPressed() { }
</pre>

## 뷰 이벤트(View Event)
뷰 이벤트는 일정한 구조에 따라 처리된다. 다른 콜백 함수들과 다르게 뷰 이벤트는 콜백 함수만 선언해서 처리할 수 없다.
<br>
뷰 이벤트는 이벤트 소스와 이벤트 핸들러로 역할이 나뉘며 이 둘을 리스너로 연결해야 한다.
<ul>
<li> 이벤트 소스(Event Source) : 이벤트가 발생한 객체
<li> 이벤트 핸들러(Event Handler) : 이벤트 발생 시 실행할 로직이 구현된 객체
<li> 리스너(Listener) : 이벤트 소스와 이벤트 핸들러를 연결해 주는 함수
</ul>
대부분 이벤트 핸들러는 이름 형식이 OnXXXListener인 인터페이스를 구현해서 만든다.
<br>
대표적으로 onClickListener, OnLongClickListener, OnItemClickListener 등의 인터페이스가 있다.
<pre>
binding.checkbox.setOnCheckdChangeListener(object: CompoundButton.OnCheckedChangeListener)

checkbox : 이벤트 소스
setOnCheckdChangeListener : 리스너
object : 이벤트 핸들러
</pre>
### 클릭과 롱클릭 이벤트 처리
ClickEvent, LongClickEvent는 뷰의 최상위 클래스인 View에 정의된 이벤트이다.
<br>
아래는 두 이벤트의 핸들러다.
<ul>
<li> open fun setOnClickListener(l: View.OnClickListener?): Unit
<li> open fun setOnLongClickListener(l: View.OnLongClickListener?): Unit
</ul>
<pre>
// 버튼의 클릭 이벤트
binding.button.setOnClickListener { }

// 버튼의 롱클릭 이벤트
binding.button.setOnLongClickListener { }
</pre>
## 마무리 실습
[[시계 앱의 스톱워치 기능 만들기]](https://github.com/JeHeeYu/Book-Reviews/tree/main/Do%20it!%20%EA%B9%A1%EC%83%98%EC%9D%98%20%EC%95%88%EB%93%9C%EB%A1%9C%EC%9D%B4%EB%93%9C%20%EC%95%B1%20%ED%94%84%EB%A1%9C%EA%B7%B8%EB%9E%98%EB%B0%8D%20with%20%EC%BD%94%ED%8B%80%EB%A6%B0/Chapter_8_%EC%82%AC%EC%9A%A9%EC%9E%90%20%EC%9D%B4%EB%B2%A4%ED%8A%B8%20%EC%B2%98%EB%A6%AC%ED%95%98%EA%B8%B0/%EB%A7%88%EB%AC%B4%EB%A6%AC%20%EC%8B%A4%EC%8A%B5)
<br>
<br>
[결과 화면]
<br>
<img src="https://user-images.githubusercontent.com/87363461/189638253-6b39dcbd-f460-445a-8154-98847d798cd1.JPG" width="200" height="400">
<img src="https://user-images.githubusercontent.com/87363461/189638260-1a4b4d88-8e8a-4041-ba28-d4ce06cc4e54.JPG" width="200" height="400">
<img src="https://user-images.githubusercontent.com/87363461/189638262-7d7ee6af-c320-4039-869f-23b844bbd159.JPG" width="200" height="400">
<img src="https://user-images.githubusercontent.com/87363461/189638279-ce4e80fd-b26e-4f1b-896d-87d3031eeeba.JPG" width="200" height="400">
