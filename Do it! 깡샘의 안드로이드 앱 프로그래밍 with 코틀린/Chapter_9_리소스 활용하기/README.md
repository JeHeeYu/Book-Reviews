# Chapter_9_리소스 활용하기 정리 내용
## 리소스(Resource)
안드로이드 앱 개발에서 리소스란 정적인 자원이라고 할 수 있습니다.
<br>
앱이 동작하면서 변경되지 않는 정적인 콘텐츠는 코드에 작성하지 않고 리소스로 분리해서 외부 파일로 만들어 이용할 수 있다.
<br>
이처럼 리소스를 이용하면 코드가 간결해져 유지 보수가 쉬워져 편리하다.
<br>
<br>
앱에서 이용하는 리소스는 크게 <b>앱 리소스와 플랫폼 리소스</b>로 구분된다.
### 앱 리소스
앱 리소스란 개발자가 직접 추가한 리소스를 의미한다.
<br>
앱을 개발하기 위해 모듈을 만들면 자동으로 res 디렉터리가 생기고, 이 리소스 디렉터리에 리소스 파일을 만든다.
<img src="https://user-images.githubusercontent.com/87363461/189640430-ddf35165-cdb1-47ab-a553-527d842befa8.JPG" width="250" heigt="230">
<br>
<br>
모듈을 만들면 기본으로 디렉터리 4개가 만들어 지는데, 리소스 파일의 종류는 더 많다.
<br>
즉, 기본 디렉터리 뿐만 아니라 더 많은 디렉터리를 res 아래에 만들어 사용할 수 있다.
<br>
|디렉터리명|리소스 종류|
|:------:|------------------|
|animator|속성 애니메이션 XML|
|anim|트윈 애니메이션 XML|
|color|색상 상태 목록 정의 XML|
|drawble|이미지 리소스|
|mipmap|앱 실행 아이콘 리소스|
|layout|레이아웃 XML|
|menu|메뉴 구성 XML|
|raw|원시 형태로 이용되는 리소스 파일|
|values|단순 값으로 이용되는 리소스|
|xml|특정 디렉터리가 정의되지 않은 나머지 XML 파일|
|font|글꼴 리소스|
<br>
<b>리소스 디렉터리명은 고정</b>이며, res 디렉터리 아래에 개발자가 임의로 이름을 붙인 디렉터리를 만들 수 없다.
<br>
하위 디렉터리도 추가할 수 없다.
<br>
<br>
리소스 파일명은 values에 추가하는 파일을 제외하고는 모두 자바의 이름 작성 규칙을 지켜야 하며, 알파벳 대문자를 사용할 수 없다.
<br>
이런 규칙이 있는 이유는 리소스 디렉터리와 파일을 코드에서 그대로 사용하지 않고, R에 식별자로 등록해서 이용하기 때문이다.
### layout 디렉터리 - 레이아웃 리소스
XML 파일을 저장하는 디렉터리로, 사용자가 보는 화면을 구성하는 XML 파일을 저장한다.
### drawble 디렉터리 - 이미지 리소스
이미지 리소스를 저장하는 디렉터리로, 이곳에 저장할 수 있는 이미지는 PNG, JPG, GIF, 9.PNG 파일이다.
<br>
또한 XML로 작성한 이미지도 이 디렉터리에 저장할 수 있다.
### mipmap 디렉터리 - 실행 아이콘 리소스
앱을 기기에 설치하면 나타나는 실행 아이콘의 이미지 리소스가 저장되는 디렉터리이다.
<br>
### values 디렉터리 - 값 리소스
값으로 이용되는 리소스를 저장하는 디렉터리로 문자열, 색상, 크기, 스타일, 배열 등의 값을 XML로 저장할 수 있다.
<br>
그런데 values에 저장되는 리소스는 다른 디렉터리의 리소스와 이용 방법이 조금 다르다.
<br>
다른 디렉터리의 리소스는 파일명이 R인 파일 식별자에 추가되므로 코드에서 이 식별자를 구분해서 사용한다.
<br>
<br>
예를 들어 layout 디렉터리에 activity_main.xml 파일은 코드에서 R.layout.activity_main으로 이용한다.
<br>
그런데 values 문자열에 있는 strings.xml은 R.values.strings 처럼 이용하지 않는다.
<br>
즉, values 디렉터리의 리소스 파일은 파일명이 R인 파일에 식별자로 등록되지 않고, 
<br>리소스 파일에 값을 지정한 name 태그의 속성값이 등록된다.
<pre>
string.xml : 문자열 리소스 등록
color.xml : 색상 리소스 등록
dimen.xml : 크기 리소스 등록
style.xml : 스타일 리소스 등록
</pre>
## 플랫폼 리소스
플랫폼 리소스는 안드로이드 플랫폼에서 제공하는 리소스들을 말한다.
<br>
<img src="https://user-images.githubusercontent.com/87363461/189643545-10df80a8-9f58-4a84-a180-2e647522b61d.JPG" width="350" height="300">
<br>
플랫폼 리소스도 R 파일에 등록된 식별자로 이용할 수 있다.
<br>
그런데 플랫폼 리소스는 앱에 있는 리소스가 아니므로 앱의 R 파일이 아니라 android.R 이라는 플랫폼 라이브러리에 등록되어 있다.
<br>
따라서 android.R 파일을 이용해 플랫폼 리소스를 이용할 수 있다.
<pre>
binding.imageView.setImageDrawble(ResourcesCompat.getDrawable(resources, android.R.drawble.alert_dark_frame, null))
binding.textView.text=getString(android.R.string.emptyPhoneNumber)
</pre>
XML에서 앱 리소스 기호는 @ 기호로 R 파일의 리소스를 이용하는데, 플랫폼 리소스를 이용할 때는 @andrid:패턴으로 한다.
<pre>
// 플랫폼 리소스
android:src="@android:drawble...

// 앱 리소스
android:src=@drawble...
</pre>
## 리소스 조건 설정
리소스 조건 설정이란 어떤 리소스를 특정 환경에서만 적용되도록 설정하는 것을 말한다.
<br>
예를들어, 앱을 실행하는 아이콘이 기기마다 화면 크기가 달라지면 선명도가 달라지는 문제가 있다.
<br>
따라서 보통 앱을 개발할 때는 아이콘이 선명하게 출력되도록 기기 크기별로 5개씩 준비해야 한다.
<br>
<br>
그런데 문제는 이미지를 어느 기기의 크기에 적용해야 하는지를 결정하는 것이다.
<br>
이때 리소스 조건을 이용할 수 있다. 즉, 리소스를 각 기기의 크기에 맞게 적용하는 작업을 리소스 조건으로 지정한다.
<br>
<br>
리소스 조건을 이용하려면 우선 아이콘의 파일명을 똑같이 지정해야 한다.
<br>
모두 같은 이름으로 만들면 R 파일에는 식별자가 하나만 생성된다.
<br>
그리고 이름이 같은 파일을 하나의 디렉터리가 아닌, mipmap-mpdi, mipmap-hdpi 처럼 각각의 디렉터리에 담는다.
<br>
이때 리소스 디렉터리 이름에서 붙임표(-) 뒤의 단어가 리소스의 조건이다.
<br>
즉, 이 디렉터리의 리소스가 어느 때애 적용되어야 하는지를 명시하는 조건이다.
<br>
<br>
<img src="https://user-images.githubusercontent.com/87363461/189647771-60835f13-d256-46e2-babe-464bcfc0f7e5.JPG" width="300" height="250">
<br>
<br>
그런데 실행 아이콘의 디렉터리 조건은 앱을 만들 때 플랫폼이 알아서 res 디렉터리 아래에 mipmap 디렉터리를 여러 개 만든다.
### 화면 회전에 대응
리소스 조건을 이용해 화면 회전에 대응하는 UI를 만들 수 있다.
<br>
화면 회전에 대응하기 위해 가로와 세로 방향일 때의 출력할 레이아웃 XML 파일을 각각 준비해야 한다.
<br>
추가하는 방법은 기존 layout 디렉터리에 세로 방향(기본) 파일을 하나 만든다.
<br>
그리고 layout 폴더 우클릭 - New - Layout Resource File - Orientiation - Landscape 순으로 한다.
<br>
<br>
<img src="https://user-images.githubusercontent.com/87363461/189651410-597dae01-18d8-44ba-bd08-60cbe7a08a6a.JPG" width="200" height="300">
### 국제 언어 제공하기
글로벌 서비스를 제공하는 앱이라면 리소스 문자열을 각국 언어로 제공해야 한다.
<br>
안드로이드 앱에서는 이 부분도 리소스 조건 설정으로 처리한다.
<br>
<br>
strings.xml 파일에 string 태그로 문자열 리소스를 작성하는데, 각 언어에 맞게 리소스 디렉터리 명을 지정하면 된다.
<br>
## 폰 크기의 호환성
안드로이드 앱 개발을 하기 위해 다양한 기기와 호환되는 화면을 만드는 것이 중요하다.
<br>
안드로이드 시스템은 기기의 크기를 ldpi, mdpi, hdpi, xhdpi, xxhdpi, xxxhdpi로 구분한다.
<br>
여기서dpi는 <b>dots per inch</b>의 줄임말로 1인치 안에 있는 도트의 개수를 의미한다.
|크기|설명|
|:------:|------------------|
|ldpi|저밀도 화면이며 ~ 120dpi|
|mdpi|중밀도 화면이며 ~ 160dpi|
|hdpi|고밀도 화면이며 ~ 240dpi|
|xhdpi|초고밀도 화면이며 ~ 320dpi|
|xxhdpi|초초고밀도 화면이며 ~ 480dpi|
|xxxhdpi|초초초고밀도 화면이며 ~ 640dpi|
<br>
이러한 이유로 콘텐츠의 크기를 지정할 때 논리적인 단위를 사용해야 한다.
<br>
안드로이드에서 앱을 개발할 때 크기 지정에 사용할 수 있는 단위는 아래와 같다.
<ul>
<li><b>dp(dip: density-independent pixcels)</b> : 스크린의 물리적 밀도에 기반을 둔 단위</li>
<li><b>sp(sip: scale-independent pixcels)</b> : dp와 유사하며 글꼴 크기에 적용</li>
<li><b>pt(points) : 스크린 크기의 1/72을 1pt로 함</li>
<li><b>px</b> : 픽셀</li>
<li><b>mm</b> : 밀리미터</li>
<li><b>in</b> : 인치</li>
</ul>
안드로이드에서는 논리적인 단위인 dp와 sp로 크기를 지정하길 권장하며,
<br>
dp는 일반 크기, sp는 글꼴 크기를 의미한다.
### 화면 정보 가져오기
안드로이드 시스템은 기본으로 크기 호환성을 지원하지만, 개발자가 직접 코드에서 조정할 수 있다.
<br>
이때 기기의 크기 정보를 가져와야 하는데, 이때 문제가 API 레벨 30버전과 이전 버전에서 차이가 있다.
<br>
<br>
30 이전 버전에서는 DisplayMetrics로 크기 정보를 가져왔지만, 30 버전 이후 부터 이 방법을 지원하지 않는다.
<br>
30 이후 버전부터는 WindowMetrics를 이용해야 한다.
<pre>
<code>
// 실행되는 버전이 30 이상일 경우
if(Build.VERSION.SDK_INT >= Build.VERSION_CODES.R) {
    val windowMetrics: WindowMetrics = windowManager.currentWindowMetrics
    binding.textView.text = "width : ${windowMetrics.bounds.width()}, height : ${windowMetrics.bounds.height()}"
}

else {
    val display = windowManager.defaultDisplay
    val displayMetrics = DisplayMetrics()
    display?.getRealMetrics(displayMetrics)
    binding.textView.text = "width : ${displayMetrics.widthPixels}, height : ${displayMetrics.heightPixels}"
}
</code>
</pre>
여기서 Build.VERSION.SDK_INT는 앱이 실행되는 기기의 버전, 
<br>
Build.VERSION_CODES.R은 안드로이드 11버전 즉, API 레벨 30을 의미한다.
