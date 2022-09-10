# Chapter_5_앱의 기본 기능 구현하기 정리 내용
## 액티비티-뷰 구조
안드로이드 앱의 기본 구조는 <b>컴포넌트를 기반으로 한다.</b>
<br>
즉, 안드로이드 앱은 액티비티, 서비스, 브로드캐스트 리시버, 콘텐츠 프로바이더와 같은 컴포넌트를 적절하게 조합해서 만든다.
<br>
이 중에서 화면을 출력하는 컴포넌트는 <b>액티비티 뿐이다.</b>
<br>
앱에서 화면을 출력하고 싶다면, 액티비티를 만들어야 하고, 이렇게 만든 액티비티에서 출력한 내용이 화면에서 보이는 것이다.
<br>
<img src="https://user-images.githubusercontent.com/87363461/189301467-1a4233c5-fa48-42dd-b961-7abc66e588ca.JPG" width="300" height="200">
<img src="https://user-images.githubusercontent.com/87363461/189307037-6743c789-561b-4c27-8c22-42f58844a06a.JPG" width="200" height="200">
<br>
액티비티는 화면을 출력하는 컴포넌트일 뿐, 그 자체가 화면이 아니다.
<br>
액티비티에서 적절한 화면을 구성해야 하며, 단순히 액티비티만 실행한다면 텅 빈 흰색 화면만 출력된다.
<br>
<br>
만약 화면에 내용을 표시하려면 뷰(View) 클래스를 이용해 구성해야 한다.
<br>
화면에 문자열을 출력하는 TextView 클래스, 이미지를 출력하는 ImageView 클래스 등의 클래스를 뷰 클래스라고 한다.
<br>
결국 <b>액티비티가 실행되면서 뷰 클래스를 이용해 화면을 구성하고 기기의 화면에 출력한다.</b>
## 액티비티 화면 구성 방법
액티비티에서 뷰로 화면을 구성하는 방법은 총 2가지가 존재한다.
<br>
액티비티 코드로 작성하는 방법과, 레이아웃 XML 파일로 작성하는 방법이 있다.
<br>
### 액티비티 코드로 화면 구성
액티비티에서 직접 코드로 만들려면 하기 예제를 참조한다.
<br>
[[액티비티에서 코드로 화면을 만드는 방법.kt]](https://github.com/JeHeeYu/Book-Reviews/blob/main/Do%20it!%20%EA%B9%A1%EC%83%98%EC%9D%98%20%EC%95%88%EB%93%9C%EB%A1%9C%EC%9D%B4%EB%93%9C%20%EC%95%B1%20%ED%94%84%EB%A1%9C%EA%B7%B8%EB%9E%98%EB%B0%8D%20with%20%EC%BD%94%ED%8B%80%EB%A6%B0/Chapter_5_%EC%95%B1%EC%9D%98%20%EA%B8%B0%EB%B3%B8%20%EA%B8%B0%EB%8A%A5%20%EA%B5%AC%ED%98%84%ED%95%98%EA%B8%B0/%EC%95%A1%ED%8B%B0%EB%B9%84%ED%8B%B0%EC%97%90%EC%84%9C%20%EC%BD%94%EB%93%9C%EB%A1%9C%20%ED%99%94%EB%A9%B4%EC%9D%84%20%EB%A7%8C%EB%93%9C%EB%8A%94%20%EB%B0%A9%EB%B2%95.kt)
<br>
이 예제에서는 필요한 뷰 객체를 코드로 직접 생성하면서 크기, 출력, 데이터 등도 일일이 객체에 대입한다.
<br>
TextView 2개와 ImageView 1개를 추가한다.
<br>
그리고 LinearLayout 객체를 액티비티 컴포넌트 함수인 setContentView()로 전달해 화면에 출력한다.
### 레이아웃 XML로 화면 구성하기
Layout-XML 파일로 작성한 예제 코드이다.
<br>
[[XML 코드로 화면을 만드는 방법.kt]](https://github.com/JeHeeYu/Book-Reviews/tree/main/Do%20it!%20%EA%B9%A1%EC%83%98%EC%9D%98%20%EC%95%88%EB%93%9C%EB%A1%9C%EC%9D%B4%EB%93%9C%20%EC%95%B1%20%ED%94%84%EB%A1%9C%EA%B7%B8%EB%9E%98%EB%B0%8D%20with%20%EC%BD%94%ED%8B%80%EB%A6%B0/Chapter_5_%EC%95%B1%EC%9D%98%20%EA%B8%B0%EB%B3%B8%20%EA%B8%B0%EB%8A%A5%20%EA%B5%AC%ED%98%84%ED%95%98%EA%B8%B0/XML%20%EC%BD%94%EB%93%9C%EB%A1%9C%20%ED%99%94%EB%A9%B4%EC%9D%84%20%EB%A7%8C%EB%93%9C%EB%8A%94%20%EB%B0%A9%EB%B2%95)
<br>
<br>
액티비티 코드로 구현한 화면을 XML 파일로 구현한 예이다.
<br>
XML을 이용해 화면을 구현하면 액티비티 코드에 이 내용을 작성하지 않아도 된다.
<br>
하지만 코드에서 화면을 구현한 XML을 명시해 어떤 화면을 출력할 것인지 알려 주어야 한다.
<br>
<br>XML에서 화면을 구현하므로, 액티비티에 setContentView() 함수에 출력할 XML을 명시한다.
<pre>
setContentView(R.layout.activity_main)
</pre>
[결과 화면]
<br>
<img src="https://user-images.githubusercontent.com/87363461/189311403-48d968bf-cbfb-4ad5-a1b0-e04d65524274.JPG" width="200" height="400">
## 뷰 클래스의 기본 구조
안드로이드에서 화면을 만들어 표시하는 컴포넌트는 액티비티이며, 액티비티가 실행되면서 뷰 클래스를 이용해 화면을 구성한다.
<br>
안드로이드는 TextView, ImageView, EditText 등 많은 뷰 클래스를 제공한다.
### 뷰 객체의 계층 구조
액티비티 화면을 구성할 때 사용하는 클래스는 모두 <b>View의 하위 클래스이다.</b>
<br>
그래서 화면 구성과 관련한 클래스를 통칭하여 <b>뷰 클래스</b>라고 한다.
<br>
<img src="https://user-images.githubusercontent.com/87363461/189462765-281cfd85-7705-4317-9b02-192047be3f82.JPG" width="400" height="200">
<br>
뷰 클래스의 계층 구조이다.
<br>
<b>View</b> : 모든 뷰 클래스의 최상위 클래스로, 액티비티는 View의 서브 클래스만 화면에 출력한다.
<br>
<br>
<b>ViewGroup</b> : View의 하위 클래스지만, 자체 UI는 없어서 화면에 출력해도 아무것도 나오지 않는다.
<br>
다른 뷰 여러 개를 묶어서 제어할 목적으로 사용되는 그릇의 역할을 하는 클래스다.
<br>
일반적으로 컨테이너 기능을 담당한다고 하며, ViewGroup의 서브 클래스인 레이아웃 클래스를 사용한다.
<br>
<br>
<b>TextView</b> : 특정 UI를 출력할 목적으로 사용하는 클래스로 문자열을 출력한다.
<br>
<br>
객체의 계층 구조에서 중요한 역할을 하는 것이 레이아웃 클래스이다.
<br>
레이아웃 클래스만으로는 액티비티 화면에 출력되는 것이 없으나, TextView, ImageView등 
<br>
객체 여러 개를 담아 한꺼번에 제어할 목적으로 사용한다.
<br>
하기 예제 처럼 레이아웃 클래스 안에 다른 뷰를 포함해 화면을 구성한다.
<br>
[[레이아웃 클래스에 뷰 포함]](https://github.com/JeHeeYu/Book-Reviews/blob/main/Do%20it!%20%EA%B9%A1%EC%83%98%EC%9D%98%20%EC%95%88%EB%93%9C%EB%A1%9C%EC%9D%B4%EB%93%9C%20%EC%95%B1%20%ED%94%84%EB%A1%9C%EA%B7%B8%EB%9E%98%EB%B0%8D%20with%20%EC%BD%94%ED%8B%80%EB%A6%B0/Chapter_5_%EC%95%B1%EC%9D%98%20%EA%B8%B0%EB%B3%B8%20%EA%B8%B0%EB%8A%A5%20%EA%B5%AC%ED%98%84%ED%95%98%EA%B8%B0/%EB%A0%88%EC%9D%B4%EC%95%84%EC%9B%83%20%ED%81%B4%EB%9E%98%EC%8A%A4%EC%97%90%20%EB%B7%B0%20%ED%8F%AC%ED%95%A8.kt)
<br>
[결과 화면]
<br>
<img src="https://user-images.githubusercontent.com/87363461/189463198-7c869463-2dc2-44e5-b39f-22ec0c946a5e.JPG" width="200" height="400">
<br>
이렇게 버튼 객체를 2개 생성하고, 이 객체를 묶어 한꺼번에 정렬, 출력 등 편리하게 제어할 수 있다.
### 레이아웃 중첩
뷰의 계층 구조는 레이아웃 객체를 중첩해서 복잡하게 구성할 수도 있다.
<br>
하기 예제는 레이아웃 객체 안에 레이아웃과 버튼을 중첩하는 예제이다.
<br>
[[레이아웃 중첩]](https://github.com/JeHeeYu/Book-Reviews/tree/main/Do%20it!%20%EA%B9%A1%EC%83%98%EC%9D%98%20%EC%95%88%EB%93%9C%EB%A1%9C%EC%9D%B4%EB%93%9C%20%EC%95%B1%20%ED%94%84%EB%A1%9C%EA%B7%B8%EB%9E%98%EB%B0%8D%20with%20%EC%BD%94%ED%8B%80%EB%A6%B0/Chapter_5_%EC%95%B1%EC%9D%98%20%EA%B8%B0%EB%B3%B8%20%EA%B8%B0%EB%8A%A5%20%EA%B5%AC%ED%98%84%ED%95%98%EA%B8%B0/%EB%A0%88%EC%9D%B4%EC%95%84%EC%9B%83%20%EC%A4%91%EC%B2%A9)
<br>
[결과 화면]
<br>
<img src="https://user-images.githubusercontent.com/87363461/189463410-b2e512d9-679b-4fce-9b4a-8496b832b227.JPG" width="200" height="400">
<br>
이처럼 객체를 계층 구조로 만들어 이용하는 패턴을
<br>컴포지트 패턴(Composite Pattern) 또는 문서 객체 모델(Document Object Model) 이라고 한다.
<br>
<img src="https://user-images.githubusercontent.com/87363461/189463506-c0a75baf-9d8c-49b9-acd9-d97b74f4b62c.JPG" width="350" height="200">
<br>
### 레이아웃 XML의 뷰를 코드에서 사용하기
레이아웃 XML의 뷰를 코드에서 사용하기 위해서 객체에 id 속성을 부여해야 한다.
<br>
id는 꼭 지정해야 하는 속성은 아니며, 뷰를 구별할 필요가 없을 때는 생략해도 된다.
<br>
<pre>
android:id="@+id/text1" >  // id 속성 부여
</pre>
id 속성은 android:id="@+id/id"의 형태로 추가하며, text1이 id 값이다.
<br>
이 id 값으로 식별자를 구분해서 이용하는 것으로 값은 앱에서 유일해야 한다.
<br>
이처럼 XML에 id 속성을 추가하면 자동으로 R.java 파일에 상수 변수로 추가된다.
<br>
XML에서 속성값이 @로 시작하면 R.java 파일읠 의미하며. text1이라는 상수 변수를 추가하라는 의미이다.
<br>
<br>
코드에서 R.java 파일의 상수 변수로 객체를 얻을 수 있으며, findViewById() 함수를 이용한다.
<pre>
// .kt File

// 액티비티 화면 출력(뷰 객체 생성)
setContentView(R.layout.activity_main)

// id값으로 뷰 객체 획득
val textView1: TextView = findViewById(R.id.text1)
</pre>
### 뷰의 크기를 지정하는 방법
뷰를 레이아웃 XML에 등록하여 화면을 구성할 때 생략할 수 없는 속성이 크기이다.
<br>
이 크기를 설정하는 속성은 layout_width, layout_height이다.
<pre>
android:layout_width="match_parent"
android:layout_height="wrap_content"
</pre>
layout_width, height 속성을 이용해 객체의 크기를 나타내는 속성으로, 3가지를 선택할 수 있다.
<br>
<b>match_parent</b> : 부모의 크기를 전체 뜻함
<br>
<br>
<b>wrap_content</b> : 자신의 콘텐츠를 화면에 출력할 수 있는 적절한 크기
<br>
<br>
<b>수치</b> : 가로세로 크기를 수치로 지정하며, 단위는 생략할 수 없다.(px, dp)
<br>
<br>
