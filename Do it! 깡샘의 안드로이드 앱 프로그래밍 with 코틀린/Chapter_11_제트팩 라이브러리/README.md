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
### 메뉴 구성
메뉴는 액션바의 중요한 구성 요소로 액티비티 화면에서 사용자가 이벤트를 사용할 수 있도록 한다.
<br>
<br>
액티비티에 메뉴를 추가하면 액션바 오른쪽에 오버플로 버튼이 나타난다.
<br>
사용자가 이 오버플로 버튼을 누르면 메뉴가 아래로 확장되어 나타난다.
<br>
그리고 오버플로 메뉴 중에서 몇몇은 액션바에 아이콘으로 나오게 할 수 있는데 이를 액션 아이템이라고 한다.
<br>
<br>
<img src="https://user-images.githubusercontent.com/87363461/190883377-90995798-f5ad-4c44-8c04-6271a503aabd.JPG" width="200" height="250">
<br>
<br>
액티비티에 메뉴를 추가하고 싶으면 onCreateOptionsMenu()와 onPrepareOptionsMneu() 함수를 이용한다.
<br>
두 함수는 액티비티의 메뉴를 구성할 때 자동으로 호출되는데, 차이점은 호출되는 시점이 다르다.
<br>
<br>
opCreateOptionsMenu() 함수는 액티비티가 실행되면서 처음에 한 번만 호출된다.
<br>
그리고 액티비티에 정적인 메뉴를 구성할 때 사용한다.
<br>
<br>
onPrepareOptionsMenu() 함수는 액티비티가 실행되면서 한 번 호출된 후 오버플로 메뉴가 나타날 때마다 반복해서 호출된다.
<br>
그리고 메뉴가 화면에 나올 때마다 동적으로 구성하고 싶은 경우 사용한다.
<br>
<br>
그런데 액티비티의 메뉴는 주로 사용자 이벤트를 처리하려고 사용하는 것이 대부분이므로,
<br>
메뉴는 대부분 onCreateOptionsMenu() 함수를 이용해 구성한다.
<pre>
// onCreateOptionsMenu() 함수 생성자
fun add(groupId: Int, itemId: Int, order: Int, title: CharSequence!): MenuItem!

// 메뉴 구성 함수
override fun onCreateOptionsMenu(menu: Menu?): Boolean {
    val menuItem1: MenuItem? = menu?.add(0, 0, 0, "menu1")
    val menuItem2: MenuItem? = menu?.add(0, 0, 0, "menu2")
    return super.onCreateOptionsMenu(menu)
}
</pre>
<img src="https://user-images.githubusercontent.com/87363461/190883584-92107d9b-d85a-4cd3-b702-01314eb763f6.JPG" width="150" height="100">
<br>
<br>
onCreateOptionsMenu() 함수의 매개변수로 전달되는 Menu 객체를 메뉴바로 생각하면 되고, 
<br>
이 Menu 객체에 메뉴를 추가할 때 add 함수를 이용한다.
<br>
<br>
두 번째 매개변수는 메뉴의 식별자로, 어떤 메뉴를 클릭했는지 식별할 때 사용한다.
<br>
<br>
네 번째 매개변수는 메뉴의 문자열이다.
<br>
<br>
add() 함수의 반환값은 MenuItem 객체이며 이 객체가 메뉴 하나를 의미한다.
<br>
<br>
onCreateOptionsMenu() 함수로 메뉴를 구성하면 액션바에 오버플로 버튼이 나온다.
<br>
이 메뉴를 사용자가 선택했을 때 이벤트 처리는 onOptionsItemSelected() 함수를 이용한다.
<br>
이 함수의 매개변수는 이벤트가 발생한 메뉴 객체인 MenuItem이다.
<br>
MenuItem의 itemId 속성으로 이벤트가 발생한 메뉴 객체의 식별값을 얻어서 이벤트를 처리한다.
<pre>
// 메뉴 선택 시 이벤트 처리
override fun onOptionsItemSelected(item: MenuItem): Boolean = when (item.itemId) {
    0 -> {
        Log.d("Menu", "menu1 Click")
        true
    }
    1 -> {
        Log.d("Menu", "menu2 Click")
        true
    }
    else -> super.onOptionsItemSelected(item)
}
</pre>

### 리소스로 메뉴 구현
액티비티의 메뉴는 대부분 정적으로 제공되므로 코드가 아니라 리소스 XML파일로 구성한다.
<br>
파일은 res 폴더 아래 menu 디렉터리에 만든다.
<pre>
< menu xmlns:android="http://schemas.android.com/apk/res/andrid"
    xmlns:app="http://schemas.android.com/apk/res-auto">
< item
    android:id="@+id/menu1"
    android:title="menu1" />
< item
    android:id="@+id/menu2"
    android:icon="@android:drawble/ic_menu_add"
    android:title="menu2"
    app:showAsAction="always" />
< item
    android:id="@+id/menu3"
    android:icon="@android:drawble/ic_menu_search"
    android:title="menu2"
    app:showAsAction="ifRoom" />
</menu>
</pre>
메뉴 XML의 <item> 태그 하나가 메뉴 하나에 해당한다.
<br>
id 속성은 레이아웃 XML에서 뷰의 id값과 마찬가지로 메뉴를 식별하는데 사용한다.
<br>
title과 icon 속성은 메뉴 문자열과 아이콘을 지정한다.
<br>
메뉴는 기본으로 오버플로 메뉴로 나오며 만약 액션바에 아이콘으로 나타나게 하려면 showAsAction 속성을 이용한다.
<pre>
never(기본) : 항상 오버플로 메뉴를 출력
ifRoom : 만약 액션바에 공간이 있다면 액션 아이템, 업다면 오버플로 메뉴 출력
always : 항상 액션 아이템으로 출력
</pre>
<pre>
// 액티비티 코드에 메뉴 XML 적용
override fun onCreateOptionsMenu(menu: Menu?): Boolean {
    menuInflater.inflate(R.menu.menu_main, menu)
    return super.onCreateOptionsMenu(menu)
}
</pre>
<img src="https://user-images.githubusercontent.com/87363461/190884448-19202f49-c6c0-4fcf-80ef-ab3d23e503c4.JPG" width="150" height="100">
<br>

### 액션 뷰(Action View) 이용
액션뷰는 액션바에서 특별한 기능을 제공하며 대표적으로 androidx.appcompat.widget.SearchView가 있다.
<br>
서치 뷰는 액션바에서 검색 기능을 제공한다.
<pre>
// 서치 뷰 사용
< item android:id="@+id/menu_search"
    android:title="search"
    app:showAsAction="always"
    app:actionViewClass="androidx.appcompat.widget.SearchView" />
</pre>
액션 뷰를 메뉴에 적용할 때 actionViewClass 속성을 이용하고, 이용할 액션뷰 클래스를 등록하면 된다.
<br>
등록하는 것만으로도 액션바에 검색 버튼이 생기고 버튼을 클릭하면 검색어를 입력받는 뷰가 나온다.
<br>
<br>
XML이 아닌 코드에서 검색 관련된 기능을 구현하려면 SearchView 객체를 얻어야 한다.
<br>
서치 뷰가 메뉴로 제공되므로 SearchView를 등록한 MenuItem 객체를 얻고 MenuItem 객체에 등록된 SearchView 객체를 구하면 된다.
<pre>
override fun onCreateOptionsMenu(menu: Menu): Boolean {
    val inflater = menuInflater
    inflater.inflate(R.menu.menu_main, menu)
    val menuItem = menu?.findItem(R.id.menu_search)
    val searchView = menuItem?.actionView as SearchView
    searchView.setOnQueryTextListener(object: SearchView.OnQueryTextListenser {
        override fun onQueryTextChange(newText: String?): Boolean {
            // 검색어 변경 이벤트
            return true
        }
        override fun onQueryTextSubmit(query String?): Boolean {
            // 키보드의 검색 버튼을 클릭한 순간 이벤트
            return true
        }
    })
    return true
}
</pre>
<img src="https://user-images.githubusercontent.com/87363461/190884804-46797dd9-a2cd-488b-966f-aca1e4131cb6.JPG" width="300" height="100">
<br>
MenuItem 객체는 findItem() 함수의 매개변수에 MenuItem의 식별값을 주어 얻는다.
<br>
MenuItem에 등록된 액션 뷰는 actionView 속성으로 얻는다.
<br>
그리고 검색과 관련된 이벤트를 처리할 때는 SearchView의 setOnQueryTextListener() 함수로 이벤트 핸들러를 지정한다. 

### 툴바(Toolbar)
툴바를 사용하는 목적은 액션바와 같지만, 액션바는 액티비티 창이 출력하는 액티비티의 구성 요소지만, 툴바는 개발자가 직접 제어하는 뷰이다.
<br>
<br>
아래 그림은 왼쪽은 액션바를, 오른쪽은 툴바를 이용하는 예제이다.
<br>
오른쪽 그림은 액티비티 창이 출력되면서 액션바를 출력하지 않는다.
<br>
<img src="https://user-images.githubusercontent.com/87363461/193797349-a09badc7-86a0-4394-bbb2-35e2a02dc9be.JPG" width="300" height="300">
<br>
<br>
툴바를 사용하려면 액티비티 테마 설정에서 액션바가 화면에 출력되지 않게 해야 한다.
<br>
그리고 액티비티 화면을 구성하는 레이아웃 XML 파일에 툴바를 등록한다.
<pre>
< androidx.appcompat.widget.Toolbar
    android:id="@+id/toolbar"
    android:layout_width="match_parent"
    android:layout_height="wrap_parent"
    style="@style/Widget.MaterialComponents.Toolbar.Primary" / >
</pre>
툴바를 XML에 등록 후 액션바의 내용이 툴바에 적용되도록 지정해 주어야 한다.
<br>
이때는 setSupportActionBar(binding.toolbar)를 이용한다.
<pre>
class MainActivity : AppCompatActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        setSupportActionBar(binding.toolbar)
    }
}
</pre>

## 호환성을 고려한 기본 뷰 클래스
플랫폼 API에서 제공하는 기본 뷰를 appcompat 라이브러리에서도 제공한다.
<br>
예를 들어 플랫폼 API에서 문자열을 출력하는 TextView 클래스를 appcompat 라이브러리에서는 AppCompatTextView라는 클래스로 제공한다.
<br>
<br>
이처럼 플랫폼 API에서 제공하는 클래스를 appcompat 라이브러리에서도 제공해주는 이유는 호환성 문제를 해결하기 위해서다.
<br>
TextView를 사용하다 보면 문자열의 줄 높이를 지정하는 setLineHeight()라는 함수가 있는데, 이 함수는 API레벨 28에서 추가되었다.
<br>
따라서 setLineHeight() 함수를 사용하려면 호환성을 고려해 다음처럼 작성해야 한다.
<pre>
if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.P) {
    binding.platformTextView.lineHeight = 50
}
</pre>
그런데 만약 TextView를 사용하지 않고 appcompat 라이브러리의 AppCompatTextView 클래스를 이용한다면 API레벨 호환성을 알아서 처리해준다.
<pre>
binding.appcompatTextView.lineHegiht = 50
</pre>

## 프래그먼트(Fragment)
프래그먼트란 텍스트 뷰나 버튼처럼 액티비티 화면을 구성하는 뷰인데, 그 자체만으로는 화면에 아무것도 출력되지 않는다.
<br>
프래그먼트가 다른 뷰와 다른점은 액티비티처럼 동작한다는 것이다.
<br>
즉, 액티비티에 작성할 수 있는 모든 코든느 프래그먼트에서도 사용 가능하다.
<br>
<br>
프래그먼트는 태블릿처럼 화면이 넓은 기기에서 동작하는 앱을 개발할 수 있도록 제공되었다.
<br>
한 화면은 하나의 액티비티 클래스에서 작성해야 하는데, 화면이 크면 액비티비 클래스에 너무 많은 코드를 해야 하는 문제가 있다.
<br>
<br>
프래그먼트는 대부분 androidx.fragment 라이브러리를 이용해 구현하며 androidx.fragment 라이브러리에서 제공한다.
<pre>
dependencies {
    implementation 'androidx.fragment:fragment-ktx:1.3.6'
}
</pre>
프래그먼트 화면을 구성하기 위해 먼저 레이아웃 XML 파일을 작성해야 한다.
<br>
Fragment 클래스를 상속받으면 프래그먼트 클래스에서 최소한으로 작성해야 하는 함수는 onCreateView()이다.
<br>
이 함수가 자동 호출되며 반환한 View 객체가 화면에 출력된다.
<pre>
import androidx.fragment.app.Fragment
class OneFragment : Fragment() {
    lateinit var binding: FragmentOneBinding
    override fun onCreateView(
        inflater: LayoutInflater, container: ViewGroup?,
        savedInstanceState: Bundle?): View? {
            binding.FragmentOneBinding.inflate(inflater, container, false)
            return binding.root
        }
}
</pre>
프래그먼트도 뷰이므로 액티비티의 화면을 구성하는 레이아웃 XML에 등록화며 화면에 출력할 수도 있다.
<br>
이때는 <fragment> 태그로 액티비티 화면에 프래그먼트를 출력한다.
<br>
<fragment> 태그의 name 속성에 프래그먼트 클래스를 지정하면 된다.
<pre>
< fragment
    android:name="com.example.test11.OneFramgent"
    android:id="@id/framgentView"
    android:layout_width="match_parent"
    andoird:layout_height="match_parent" / >
</pre>

### 액티비티 코드에서 프래그먼트 출력
코드에서 직접 프래그먼트 객체를 생성하기 위해 레이아웃 XML파일에 프래그먼트가 출력될 뷰가 있어야 한다.
<pre>
< LinearLayout
    android:id="@+id/fragment_content"
    android:layout_width="match_parent"
    android:layout_height="match_parent"
    anroid:orientation="vertical"
< /LinearLayout >
</pre>
