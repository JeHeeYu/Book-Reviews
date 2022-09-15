# Chater_11_제트팩 라이브러리 정리 내용
# 11-1 제트팩(Jetpack) 라이브러리
구글에서 안드로이드 앱을 개발하는데 필요한 다양한 라이브러리 모음을 <b>제트팩</b> 이라는 이름으로 제공한다.
<br>
제트팩은 안드로이드 플랫폼이 기본으로 제공하는 플랫폼 API 외에 따로 추가한 라이브러리이다.
## 플랫폼 API
플랫폼 API는 ART(Android RunTime)에서 제공하는 안드로이드의 핵심 라이브러리이다.
<br>
ART는 대부분 android나 java로 시작하는 패키지명을 사용한다.
<br>
<br>
java.lang.String, java.util.Data 등의 자바 클래스부터 android.app.Activity 등도 모두 플랫폼 API이다.
<br>
<br>
<img src="https://user-images.githubusercontent.com/87363461/190378966-fae1be39-2389-45f3-a8fd-5c12afc1d236.JPG" width="300" height="100">
<br>
<br>
## 제트팩
제트팩은 androidx로 시작하는 패키지명을 사용한다.
<br>
제트펙은 크게 3가지의 목적으로 제공된다.
<br>
<li>앱을 개발하는 데 필요한 권장 아키텍처를 제공한다.</li>
<li>API 레벨의 호환성 문제를 해결한다.</li>
<li>플랫폼 API에서 제공하지 않는 다양한 기능을 제공한다.</li>
<br
제트팩에서는 앱을 개발할 때 적용할 수 있는 다양한 아키텍처를 제시하며,
<br>
뷰 모델(ViewModel), 라이브 데이터(LiveData), 룸(Room), 페이징(paging) 등과 같은 라이브러리를 제공한다.
<br>
<br>
제트팩은 API 레벨의 호환성 문제를 해결해 준다.
<br>
예를 들어 툴바를 구현하려면 플랫폼 API에서 android.widget.Toolba 클래스를 이용해야 하는데, API 레벨이 21이다.
<br>
하위 버전에서 오류가 발생한다.
<br>
<br>
그런데 제트펙의 appcompat 라이브러리에서 제공하는 androidx.appcompat.widget.Toolbar 클래스를 이용하면 문제가 해결된다.
<br
따라서 목적이 같은 클래스를 제트팩의 라이브러리에서도 제공한다면 대부분 제트팩의 클래스를 이용해야 한다.
## 화면 구성과 관련된 androidx 라이브러리
<li>androidx.appcompat : 앱의 API 레벨 호환성을 해결한다.</li>
<li>androidx.recyclerview : 목록 화면을 구성한다.</li>
<li>androidx.viewpager2 : 스와이프로 넘기는 화면을 구성한다.</li>
<li>androidx.fragment : 액티비티처럼 동작하는 뷰를 제공한다.</li>
<li>androidx.drawerlayout : 옆에서 서랍처럼 열리는 화면을 구성한다.</li>

# 11-2 appcompat 라이브러리 - API 호환성 해결
androidx 라이브러리에서 가장 많이 사용하는 appcompat 라이브러리는 안드로이듸 앱의 화면을 구성하는
<br>
액티비티를 만들며 API 레벨의 호환성 문제를 해결해 준다.
<br>
appcompat 라이브러릴 사용하려면 gradle 파일의 dependencies 항목(의존설 설정)에 추가해야 한다.
<br>
그런데 이 선언은 안드로이드 스튜디오 모듈을 만들 때 자동으로 추가된다.
<pre>
// appcompat 라이브러리 선언
implementation 'androidx.appcompat:appcompat:1.3.1'
</pre>
appcompat 라이브러리를 이용해서 액티비티를 만들 때 플랫폼의 API의 Activity가 아니라
<br>
다음처럼 appcompat의 AppCompatActivity 클래스를 상속받아 작성한다.
<per>
// appcompat 라이브러리 사용
import androidx.appcompat.app.AppCompatActivity
class MainActivity : AppCompatActivity() { }
</pre>

## 액션바(Actionbar)
액티비티의 구성 요서인 액션바는 화면 위쪽에 타이틀 문자열이 출력되는 영역을 의미한다.
<br>
액티비티가 출력되는 전체 창은 액션바와 콘텐츠 영역으로 구분되고, 액션바 영역에는 타이틀이 출력된다.
<br>
그리고 콘텐츠 영역에는 setContentView() 함수가 출력하는 내용이 출력된다.
<br>
<br>
<img src="https://user-images.githubusercontent.com/87363461/190382289-3d026785-2b9b-4de0-9e4b-8a9fc797e7c8.JPG" width="300" height="400">
<br>
<br>
액션바의 기본 구성도 타이틀, 내비게이션 아이콘, 액션 아이템, 오버플로 메뉴 등 다양한 요소를 출력할 수 있고,
<br>
타이틀은 보이지 않게 설정할 수 있다.
<br>
<br>
<img src="https://user-images.githubusercontent.com/87363461/190382738-15c17c62-1fc1-4a88-8c32-777ba180b447.JPG" width="300" height="150">
<br>
<br>
### 액션바 색상 설정
안드로이드 앱을 실행하면 기본으로 액션바가 출력된다. 이때 액션바의 색상은
<br>
이 앱에 자동으로 적용되는 테마에 의해서 결정된다.
<br>
<br>
테마 스타일은 res/values 디렉터리에 있는 themes.xml 파일에 선언되어 있다.
<br>
themes.xml 파일의 스타일이 앱의 액티비티에 테마로 자동 설정되고 이 테마 파일에 선언된 색상이 액티비티에 적용된다.
<br>
이 스타일은 머터리얼 디자인에서 제공하는 Theme.MaterialComponents.DayNight.DarkActionbar를 상속받아 작성된다.
<br>
<br>
<img src="https://user-images.githubusercontent.com/87363461/190385303-5bce0ba6-ac2e-4155-8128-ddeb9e0f22c6.JPG" width="300" height="400">
<br>
<br>
colorPrimary와 colorSecondar는 앱의 브랜드를 표현하는 색상이다.
<br>
colorPrimary는 액션바와 버튼의 배경색으로 사용하고, colorSecondary는 활성 상태를 표현한다.
<br>
즉, 텍스트 뷰의 링크, 체크박스, 라디오 버튼이 체크(스위치 켬) 상태일 때<br>
플로팅 액션 버튼의 배경색 등에 colorSecondary를 사용하고, statusBarColor는 상태바의 배경색으로 사용한다.
<br>
<br>
또한 colorOnPrimary, colorOnSecondary는 colorPrimary, colorSecondary가 적용된 곳 내용의 전경색으로 사용한다.
<br>
그리고 colorPrimaryVariant, colorSecondaryVariant는 그림자의 색상으로 사용한다.

### 액션바 숨기기 
액티비티 창은 기본적으로 액션바를 포함하는데 액션바를 포함하고 싶지 않으면, 액션바를 만들 때
<br>
style 태그에 Theme.Material.Components.DayNit.NoActionBar를 상속받으면 액션바가 나오지 않는다.
<br>
<br>
만약 NoAtionBar를 상속받아 작성할 수 없는 경우라면 테마의 <item> 속성을 설정할 수도 있다.

### 업 버튼 설정
업 버튼은 액티비티 화면이 앱의 첫 화면이 아닐 때 이전으로 돌아가는 기능을 한다.
<br>
<br>
<img src="https://user-images.githubusercontent.com/87363461/190390020-c35bd344-b294-4a84-b41e-e99f259579f6.JPG" width="200" height="100">
<br>
<br>
업 버튼은 액티비티가 등록되는 매니페스트 파일에서 설정하는 방법과 액티비티 코드로 정하는 방법이 있다.
<pre>
// 매니페스트 파일에서 업 버튼 설정
android:parentActivityName=".MainActivity"
</pre>
매니페스트 파일에서 <activity> 태그에 parentActivityName 속성을 등록하는 것 만으로도 업 버튼이 나온다.
<br>
<br>
그런데 업 버튼을 눌렀을 때 로직을 실행하고 싶으면 액티비티에 onSupportNavigateUp() 함수를 재정의 한다.
<br>
이렇게 하면 사용자가 업 버튼을 누를 때 onSupportNavigateUp() 함수가 자동으로 호출된다.
<pre>
override fun onSupportNavigateUp(): Boolean {
    Log.d("Up", "onSpportNavigateUp()")
    return super.onSupporNavigateUp()
}
</pre>
메니페스트 파일에 속성을 선언하지 않고 액티비티 코드로 버튼이 나오게 할 수 있다.
<pre>
class TwoActivity : AppCompatActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        suppertActionBar?.setDisplayHomeAsUpEnable(true)      // 업 버튼 출력
    }
    override fun onSupportNavigateUp(): Boolean {
    Log.d("Up", "onSpportNavigateUp()")
    return super.onSupporNavigateUp()
    }
}
</pre>
supportActionBar?.setDisplayHomeAsUpEnabled(true) 구문으로 액션바에 업 버튼이 나온다.
<br>
이때도 업 버튼을 클릭하면 onSupportNavigateUp() 함수가 자동으로 호출된다.
<br>
<br>
만약 매니페스트 파일에 parentActivityName 속성이 설정되어 있지 않으면 자동으로 이전 화면으로 돌아가지 않는다.
<br>
이때는 onSupportNavigate() 함수에서 onBackPressed() 구문으로 코드를 작성해 주어야 한다.
