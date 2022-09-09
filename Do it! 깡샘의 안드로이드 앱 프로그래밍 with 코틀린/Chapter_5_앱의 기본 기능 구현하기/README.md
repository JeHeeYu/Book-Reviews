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
## 뷰 클래스의 기본 구조
안드로이드에서 화면을 만들어 표시하는 컴포넌트는 액티비티이며, 액티비티가 실행되면서 뷰 클래스를 이용해 화면을 구성한다.
<br>
안드로이드는 TextView, ImageView, EditText 등 많은 뷰 클래스를 제공한다.
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
