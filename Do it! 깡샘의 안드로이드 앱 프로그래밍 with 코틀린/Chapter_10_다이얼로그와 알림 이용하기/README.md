# Chapter_10_다이얼로그와 알림 이용하기 정리 내용
## API 레벨 호환성
SDK 버전을 설정하는 targetSdk와 minSdk 두 항목에 설정하는 값은 API 레벨로 앱 개발에 큰 영향을 주는 중요한 정보이다.
<pre>
minSdk 21
targetSdk 31
</pre>
위와 같다면 targetSdk에 설정한 31버전의 API로 앱을 개발한다는 의미이다.
<br>
그런데 minSdk를 21로 지정했으므로 이 앱은 21 버전(안드로이드 5.0 롤리팝) 기기부터 설치할 수 있다.
<br>
결국 이 앱은 31 버전의 API로 개발하지만 21 버전 기기에서도 오류가 발생하지 않고 동작해야 한다.
<br>
<br>
따라서 앱을 개발할 때 <b>minSdk 설정값보다 상위 버전에서 제공하는 API를 사용한다면 호환성을 고려해야 한다.</b>
<br>
SDK 레벨은 안드로이드 API 공식 문서에서 확인할 수 있다.
<br>
Added in API level 31의 경우는 31 버전에서 추가된 클래스라는 의미이다.
<br>
31버전 미만 버전에서 이 클래스를 이용할 경우 오류가 발생한다.
<br>
<br>
이처럼 API 레벨 호환성에 문제가 있는 API를 사용할 때는 
<br>
<b>@기호로 시작하는 애너테이션(Annotation)을 추가해 오류를 해결 할 수 있다.</b>
<pre>
@RequiresApi(Build.VERSION_CODES.S)
fun noti() {
  val builder: Notification.Builder = Notification.Builder(this, "1")
      .setStyle(Notification.CallStyle.forIncomingCall(caller, declineIntent, answerIntent)
}
</pre>
API 레벨 호환성에 문제가 있는 API르 사용한 함수나 클래스 선언부 위에 @RequiresApi 애너테이션을 추가한다.
<br>
이렇게 추가하면 안드로이드 스튜디오에서 오류가 발생하지 않는다.
<pre>
@TargetApi(Build.VERSION_CODES.S)
fun noti() {
  val builder: Notification.Builder = Notification.Builder(this, "1")
      .setStyle(Notification.CallStyle.forIncomingCall(caller, declineIntent, answerIntent)
}
</pre>
위처럼 애너테이션을 @TargetApi로 선언할 수도 있다.
<br>
<br>
그런데 문제는 API 호환성 애너테이션은 안드로이드 스튜디오에서 오류를 무시할 설정일 뿐이다.
<br>
앱이 실행될 때 API 레벨 호환성 문제를 막으려면 직접 코드로 처리해 줘야 한다.
<br>
예를 들어 Notification.CallStyle 클래스는 다음처럼 S 버전에서만 실행되게 할 수 있다.
<pre>
if(Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
  val builder: Notification.Builder = Notification.Builder(this, "1")
      .setStyle(Notification.CallStyle.forIncomingCall(caller, declineIntent, answerIntent)
}
</pre>
Build.VERSION.SDK_INT는 앱이 실행되는 기기의 API 레벨로 조건문에서 이 값을 이용해 특정 버전만 실행되도록 한다.
## 퍼미션(Permission) 설정하기
퍼미션은 앱의 특정 기능에 부여하는 접근 권한을 말한다.
<br>
개발하는 앱이 다른 앱이나 안드로이드 시스템에서 보호하는 특정 기능을 이용할 때 퍼미션 사용을 설정해야 한다.
<br>
마찬가지로 내가 만든 기능을 다른 앱에서 사용할 수 없도록 보호하고 권한을 얻은 앱에서만 허용하고 싶을 때 퍼미션을 설정한다.
### 퍼미션 설정과 사용 설정
B앱에서 A앱의 컴포넌트와 연동하는 코드만 잘 구현했다면 A 앱의 컴포넌트를 B앱에서 얼마든지 사용할 수 있다.
<br>
그런데 만약 A앱의 컴포넌트에 퍼미션을 설정하면 B앱에서 연동할 때 문제가 발생한다.
<br>
<br>
<img src="https://user-images.githubusercontent.com/87363461/189881461-0bbbb59f-be89-4eb3-8c63-18f6f2cee481.JPG" width="400" height="200">
<img src="https://user-images.githubusercontent.com/87363461/189881468-bf79c29e-6639-450d-b79f-c7d954abae19.JPG" width="400" height="200">
<br>
<br>
A앱의 개발자가 <b>매니페스트</b> 파일에 permission 태그로 퍼미션을 설정하면 이를 이용하는
<br>
B앱의 코드를 아무리 잘 구현하더라도 실행되지 않는다.
<br>
이때는 B앱의 매니페스트 파일에 <uses-permission> 태그로 해당 퍼미션을 이용하겠다고 설정해 주어야 한다.
<ul>
<li><b>permission</b> 태그 : 기능을 보호하려는 앱의 매니페스트 파일에 설정한다.</li>
<li><b>uses-permission</b> 태그 : 퍼미션으로 보호된 기능을 사용하려는 앱의 매니페스트 파일에 설정한다.</li>
</ul>
<br>
<img src="https://user-images.githubusercontent.com/87363461/189881773-4acc50c7-2c7e-4734-9cb9-eadb62741ba5.JPG" width="400" height="200">
<br>
매니페스트 파일에 퍼미션을 설정할 때 permission 태그와 다음 속성을 이용한다.
<ul>
<li><b>name</b> : 퍼미션의 이름</li>
<li><b>label, description</b> : 퍼미션 설명</li>
<li><b>protectionLevel</b> : 보호 수준</li>
</ul>
<img src="https://user-images.githubusercontent.com/87363461/189882557-4e3544af-f6b5-4a1d-9c7c-d5962d9a2fc2.JPG" width="700" height="200">
<br>
<br>
name 속성값은 개발자가 정하는 이름으로, 퍼미션을 구별하는 식별자 역할을 한다.
<br>
<br>
label과 description 속성값은 이 퍼미션을 이용하는 외부 앱에서 권한 인증 화면에 출력할 퍼미션의 정보이다.
<br>
<br>
protectionLevel 속성값은 보호 수준을 의미하여 다음 같은 값을 지정할 수 있다.
<li><b>normal</b> : 낮은 수준의 보호로 사용자에게 권한을 요청하지 않음</li>
<li><b>dangerous</b> : 높은 수준의 보호로 사용자에게 권한 요청</li>
<li><b>signature</b> : 같은 키로 인증한 앱만 실행</li>
<li><b>signatureOrSystem</b> : 안드로이드 시스템 앱이거나 같은 키로 인증한 앱만 실행</li>
<br>
매니페스트 파일에 퍼미션 설정후 컴포넌트에 적용을 해야 퍼미션 적용이 정상적으로 이루어 진다.
<pre>
android:permission="com.example.TEST_PERMISSION"
</pre>
이제 이 컴포넌트는 com.example.TEST_PERMISSION에 의해 보호되며 이 컴포넌트를 이용하는 곳에서는
<br>
uses-permission 태그에 선언해줘야 정상적으로 실행이 된다.
<pre>
android:permission="com.example.TEST_PERMISSION"
</pre>
이처럼 외부 앱과 연동 시 퍼미션 설정을 사용해야 하며, 시스템에서 보호하는 기능도 사용해 주어야 한다.
<br>
시스템이 보호하는 기능은 대표적으로 하기와 같다.
<ul>
<li><b>ACCESS_FINE_LOCATION</b> : 위치 정보 접근</li>
<li><b>ACCESS_NETWORK_STATE</b> : 네트워크 정보 접근</li>
<li><b>ACCESS_WIFI_STATE</b> : 와이파이 네트워크 정보 접근</li>
<li><b>BATTERY_STATS</b> : 배터리 정보 접근</li>
<li><b>BLUETOOTH</b> : 블루투스 장치에 연결</li>
<li><b>BLUETOOTH_ADMIN</b> : 블루투스 장치를 검색하고 페어링</li>
<li><b>CAMERA</b> : 카메라 장치에 접근</li>
<li><b>INTERNET</b> : 네트워크 연결</li>
<li><b>READ_EXTERNAL_STORAGE</b> : 외부 저장소에서 파일 읽기</li>
<li><b>WRITE_EXTERNAL_STORAGE</b> : 외부 저장소에서 파일 쓰기</li>
<li><b>READ_PHONE_STATE</b> : 전화기 정보 접근</li>
<li><b>SEND_SMS</b> : 문자 메시지 발신</li>
<li><b>RECEIVE_SMS</b> : 문자 메시지 수신</li>
<li><b>RECEIVE_BOOT_COMPLETED</b> : 부팅 완료 시 실행</li>
<li><b>VIBRATE</b> : 진동 울리기</li>
</ul>

### 퍼미선 허용 확인
퍼미션은 API 레벨 1부터 있었던 내용이지만, API 23레벨 버전부터 정책이 바뀌었다.
<br>
API 23레벨 이전에는 개발자가 매니페스트 파일에 uses-permission 태그로 선언만 하면 앱 이용에 문제가 없었다.
<br>
그런데 API 레벨 23 버전부터 허가제로 바뀌어 사용자가 권한 화면에서 이를 거부할 수 있게 되었다.
<br>
<br>
<img src="https://user-images.githubusercontent.com/87363461/189884466-816a0101-c222-4406-ab8d-3eb8e7c7a44f.JPG" width="300" height="400">
<br>
<br>
만약 사용자가 앱의 권한 설정에서 특정 퍼미션 거부 시 uses-permission 태그를 선언하지 않은 것과 같아 기능을 이용할 수 없다.
<br>
만약 거부했을 경우 앱 실행 시 다시 퍼미션 허용을 달라고 요청해야 한다.
<br>
<br>
사용자가 머피션을 허용했는지 확인하기 위해 checkSelfPermission() 함수를 제공한다.
<pre>
open static fun checkSelfPermission(
    @NonNull context: Context,
    @NonNull permission: String     // 퍼미션 구분 이름
): Int
</pre>
두 번째 매개변수가 퍼미션을 구분하는 이름이며 결괏값은 다음 중 하나의 상수로 전달된다.
<ul>
<li><b>PackageManager.PERMISSION_GRANTED</b> : 권한을 허용한 경우</li>
<li><b>PackageManager.DENIED</b> : 권한을 거부한 경우</li>
<li><b>RECEIVE_BOOT_COMPLETED</b> : 부팅 완료 시 실행</li>
</ul>
<pre>
val status = ContextCompat.checkSelfPermission(this, "android.permission.ACCESS_FINE_LOCATION")
if(status == PackageManager.PERMISSION_GRANTED) { }
else { }
</pre>
만약 퍼미션을 거부한 상태라면 사용자에게 해당 퍼미션을 허용해 달라고 요청해야 한다.
<br>
사용자에게 퍼미션 요청 시 ActivityResultLauncher 클래스를 이용한다.
<br>
이 클래스는 액티비티에서 결과를 돌려받아야 할 때 사용하며, 대표적으로 퍼미션 허용 요청이 있다.
<br>
<br>
ActivityResultLauncher 객체는 registerForActivityResult() 함수를 호출해서 만든다.
<pre>
public final <I, O> ActivityResultLauncher<I> register
ForActivityResult(@NonNull ActivityResultContract<I, O> contract, @NonNull ActivityResultCallback<O> callback)
</pre>
registerForActivityResult() 함수는 매개변수가 2개이다.
<br>
첫 번째는 어떤 요청인지를 나타내는 ActivityResultContract 타입 객체이며 다양한 요청에 대응하는 서브 클래스들이 있다.
<br>
대표적으로 다른 액티비티를 실행하고 결과를 돌려받을 때 StartActivityForResult,
<br>
퍼미션 허용을 요청할 때는 Requestpermission을 사용한다.
<br>
<br>
두 번째 매개변수는 결과를 받았을 때 호출되는 콜백이다.
<pre>
// 퍼미선 요청 확인
val requestPermissionLauncher = registerForActivityResult(ActivityResultContracts.RequestPermission()) {
    isGranted -> 
    if(isGranged) { } // granted Callback
    else { }    // denied Callback
}

// 퍼미션 요청 실행
requestPermissionLaunch.launch("android.permission.ACCESS_FINE_LOCATION")
</pre>
requestForActivityResult() 함수로 ActivityResultLauncher 객체를 만들었다면,
<br>
필요한 곳에서 ActivityResultLaunch 객체의 launch() 함수를 호출하여 퍼미션 요청을 실행한다.
<br>
<br>
요청 결과는 registerForActivityResult() 함수의 두 번째 매개변수로 등록한 콜백으로 전달된다.
## 다이얼로그(Dialog) 종류
다이얼로그란 사용자와 상호 작용하는 대화상자로 토스트, 날짜 또는 시간 입력, 커스텀 다이얼로그 등이 있다.
### 토스트 메시지(Toast Message)
토스트는 화면 아래쪽에 잠깐 보였다가 사라지는 문자열을 의미하며 사용자에게 간단한 메시지로 특정한 상황을 알릴 때 사용한다.
<br>
<br>
토스트를 사용하는 대표적인 예가 뒤로가기 버튼을 눌렀을 때 종료 확인을 받을 때를 들 수 있다.
<br>
<br>
토스트는 Toast의 makeTest() 함수로 만든다.
<pre>
open static fun makeText(context: Context!, text: CharSquence!, duration: Int): Toast!
open static fun makeText(context: Context!, resId: Int, duration: Int): Toast!
</pre>
makeText() 함수의 두 번째 매개변수가 출력할 문자열이다.
<br>
세 번째 매개변수는 토스트가 화면에 출력되는 시간이다. 보통 하기의 상수를 사용한다.
<ul>
<li>val LENGTH_LONG: Int</li>
<li>val LENGTH_SHORT: Int</li>
</ul>
Toast.LENGTH_SHORT는 일반적으로 3초 정도, Toast.LENGTH_LONG은 5초 정도의 시간을 의미한다.
<br>
토스트가 화면에 출력된 이후 시간이 지나면 자동으로 사라진다.
<pre>
// 토스트 출력 예제
val toast = Toast.makeText(this, "종료하려면 한 번 더 누르세요", Toast.LENGTH_SHORT)
toast.show()
</pre>
토스트는 makeText() 함수외에 하기의 세터 함수로도 만들 수 있다.
<pre>
open fun setDuration(duration: Int): Unit
open fun setGravity(gravity: Int, xOffset: Int, yOffset: Int): Unit
open fun setMargin(horizontalMargin: Float, verticalMargin: Float): Unit
open fun setText(resId: Int): Unit
</pre>
setDuration(), setText() 함수를 이용하면 문자열이나 화면에 보이는 시간을 설정할 수 있다.
<br>
setGravity()나 setMargin() 함수를 이용하면 토스트가 뜨는 위치를 지정할 수 있다.

### 토스트 콜백 함수
토스트가 화면에 보이거나 사라지는 순간을 콜백으로 감지해 특정 로직을 수행할 수 있다.
<br>
이 콜백 기능은 API 레벨 30버전에서 추가되어 이후 버전부터 사용할 수 있다.
<pre>
// 콜백 기능 이용하기
@RequiresApi(Build.VERSION_CODES.R)    // API 레벨 호환성 애너테이션
fun showToast() {
    val toast = Toast.makeText(this, "종료하려면 한 번 더 누르세요", Toast.LENGTH_SHORT)
    toast.addCallback(object: Toast.Callback() {
        override fun onToastHidden() {
            super.onToastHidden()       // Toast가 사라질 때
            Log.d("Toast", "Toast Hidden")
        }
        
        override fun onToastShown() {     // 토스타가 나타날 때
            super.OnToastShown()
            Log.d("Toast", "Toast Show")
        }
    })
    toast.show()
}
</pre>
Toast.Callback 타입의 객체를 토스트 객체의 addCalback() 함수로 등록해주면 된다.
<br>
이렇게 하면 onToastShown(), oNToastHidden() 함수가 자동으로 호출된다.

### 날짜 또는 시간 입력받기
앱에서 사용자에게 날짜나 시간을 입력받는데 사용하는 다이얼로그를 <b>피커(Picker) 다이얼로그</b> 라고 한다.
<br>
날짜를 입력 받을 때는 <b>데이트 피커 다이얼로그(DatePickerDialog)</b>를,
<br>
시간을 입력받을 때는 <b>타임 피커 다이얼로그(TimePickerDialog)</b> 를 사용한다.
<pre>
// 데이트 피커 다이얼로그 생성자
DatePickerDialog(context: Context, listener: DatePickerDialog.OnDateSetListener?,
                 year: Int, month: Int, dayOfMonth: Int)
</pre>
두 번째 매개변수로 DatePickerDialog.OnDateSetListener 구현 객체를 등록하면 다이얼로그에서
<br>
사용자가 설정한 날짜를 콜백 함수로 얻을 수 있다.
<br>
<br>
나머지 Int 타입의 매개변수느 처음에 보이는 날짜로, month 값은 0부터 11까지, 0은 1월을 의미한다.
<pre>
// 데이트 피커 다이얼로그 사용 예
DatePickerDialog(this, object: DatePickerDialog.OnDateSetListener {
    override fun onDateSet(p0: DatePicker?, p1: Int, p2: Int, p3: Int) {
        Log.d("Date", "year : $p1, month : ${p2+1}, dayOfMonth : $p3")
    }
}, 2020, 8, 21).show()
</pre>
[실행 결과]
<br>
<br>
<img src="https://user-images.githubusercontent.com/87363461/189915286-621d1333-98e2-4178-a7c0-76d1b77bfa2c.JPG" width="200" height="400">
<br>
<pre>
// 타임 피커 다이얼로그 생성자
TimePickerDialog(context: Context!, listener: TimePickerDialog.OnTimeSetListener!,
                 hourOfDay: Int, minute: Int, is24HourView: Boolean)
</pre>
두 번째 매개변수로 TimePickerDialog.OnTimeSetListener를 구현한 객체를 지정하면 사용자가<br>
다이얼로그에서 설정한 시간을 얻을 수 있으며, 처음에 보일 시간을 Int 타입으로 설정할 수 있다.
<br>
<br>
마지막 매개변수로 시간을 24시간과 12시간 형태 중에 어떤 것으로 출력할 것인지를 지정한다.
<br>
false로 지정할 경우 12시간 형태로 출력해 오전/오후 형태를 선택하는 부분이 보인다.
<br>
true로 지정할 경우 24시간 형태로 출력해 오전/오후 형태를 선택하는 부분이 없다.
<pre>
// 타임 피커 다이얼로그 사용 예
TimePickerDialog(this, object: TimePickerDialog.OnTimeSetListener {
    override fun onTimeSet(p0: TimePicker?, p1: Int, p2: Int) {
        Log.d("Time", "time : $p1, minute : $p2")
    }
}, 15, 0, true).show()
</pre>
[실행 결과]
<br>
<br>
<img src="https://user-images.githubusercontent.com/87363461/189916129-83ed9d8c-e9f3-434a-ab8b-cd76a937ce79.JPG" width="200" height="400">
<br>
<br>
### 알림 창 띄우기
안드로이드 다이얼로그의 기본은 AlertDialog로 단순한 메시지만 출력할 수도 있고, 다양한 화면을 출력할 수도 있다.
<br>
데이트 피커와 타임 피커도 AlertDialog의 하위 클래스로 각각의 화면에 데이트 피커와 타임 피커를 출력한 다이얼로그이다.
<br>
<br>
알림 창은 크게 3가지 영역으로, 제목, 내용, 버튼 영역이 있다.
<br>
<br>
<img src="https://user-images.githubusercontent.com/87363461/189917022-00075d09-39e7-4e4a-b159-54b9af77424b.JPG" width="300" height="150">
<br>
<br>
이 3가지의 영역은 항상 보이는 것은 아니며, 알림 창을 설정할 때 제목과 버튼 정보를 지정하지 않으면 내용 영역만 출력된다.
<br>
<br>
알림 창의 생성자는 접근 제한자가 protected로 선언돼서 객체를 직접 생성할 수 없다.
<br>
대신 AlertDialog.Builder를 제공하므로 이 빌더를 이용해서 알림 창을 만든다.
<br>
먼저 AlertDialog.Builder를 생성하고 빌더의 세터 함수로 알림 창의 정보를 지정한다.
<pre>
// 알림 창 빌더
AlertDialog.Builder(context: Context!)

// 알림 창에 아이콘 제목과 내용을 지정하는 함수들
open fun setIcon(iconId: Int): AlertDialog.Builder!
open fun setTitle(title: CharSequence!): AlertDialog.Builder!
open fun setMessage(message: CharSequence!): AlertDialog.Builder!
</pre>
setIcon() 함수는 제목 영역에 아이콘을 출력한다.
<br>
<br>
setTitle() 함수는 제목 문자열을 출력한다.
<br>
<br>
setMessage() 함수는 내용 영역에 간단한 문자열을 출력할 때 사용한다.
<pre>
// 알림 창에 버튼을 지정하는 함수
open fun setPositiveButton(text: CharSequence!, listener: DialogInterface.OnclickListener!): AlertDialog.Builder!
open fun setNegativeButton(text: CharSequence!, listener: DialogInterface.OnclickListener!): AlertDialog.Builder!
open fun setNeturalButton(text: CharSequence!, listener: DialogInterface.OnclickListener!): AlertDialog.Builder!
</pre>
각 함수의 첫 번째 매개변수는 버튼의 문자열이다.
<br>
<br>
두 번째 매개변수는 사용자가 버튼을 클릭했을 때 처리할 이벤트 핸들러이다.
<br>
만약 버튼을 클릭했을 때 처리할 내용이 없다면 두 번째 매개변수에 null을 대입한다. 그러면 클릭 후 창이 닫힌다.
<br>
<br>
알림 창의 버튼은 최대 3개까지만 추가할 수 있으며, 함수를 중복 호출하면 버튼은 중복되어 하나만 나타난다.
<pre>
// 알림 창 띄우기
AlertDialog.Builder(this).run {
    setTitle("test dialog")
    setIcon(android.R.drawable.ic_dialog_info)
    setMessage("정말 종료하시겠습니까?")
    setPositiveButton("OK", null)
    setNegativeButton("Cancel", null)
    setNeturalButton("More", null)
    setPositiveButton("YES", null)    // 중복 호출해도 마지막 함수만 호출됨
    setNegativeButton("NO", null)     // 중복 호출해도 마지막 함수만 호출됨
    show()
}
</pre>
<img src="https://user-images.githubusercontent.com/87363461/189919216-56d3b05b-0a81-4fbb-a7e3-6cea381ac3f9.JPG" width="300" height="150">
<br>
<br>
버튼 함수를 3개로 구분하는 이유는 이벤트 핸들러에서 어떤 버튼이 클릭되었는지 구분하기 위해서이다.
<br>
각 이벤트에 해당하는 이벤트 핸들러를 따로 만들 수도 있지만 한 알림 창의 버튼 이벤트를 하나의
<br>
이벤트 핸들러에서 모두 처리할 수 있다.
<br>
<br>
이때 어떤 버튼이 클릭되었는지를 구분해야 하는데, 셋 중 어떤 함수를 사용했는지에 따라 이벤트 핸들러에
<br>
전달되는 매개변수값이 달라서 그 값으로 구분한다.
<pre>
// 버튼의 이벤트 핸들러 등록
val eventHandler = object : DialogInterface.OnClickListener {
    override fun onClick(p): DialogInterface?, p1: Int) {
        if (p1 == DialogInterface.BUTTON_POSITIVE) {
            Log.d("Button", "Positive Button Click")
        }
        else {
            Log.d("Button", "Negative Button Click")
        }
    }
}

setPositiveButton("OK", eventHandler)     // 이벤트 핸들러 등록
setNegativeButton("OK", eventHandler)     // 이벤트 핸들러 등록
</pre>
알림 창을 클릭했을 때 호출되는 OnClick() 함수의 두 번째 매개변수가 이벤트가 발생한 버튼을 알려준다.
<br>
setPositiveButton() 함수로 만든 버튼은 이벤트 구분자가 DialogInterface.BUTTON_POSITIVE로 지정된다.
<br>
setNegativeButton() 함수로 만든 버튼은 이벤트 구분자가 DialogInterface.BUTTON_NEGATIVE로 지정된다.
<br>
따라서 이 값으로 버튼을 구분해 적절하게 처리해 주면 된다.
<br>
<br>
알림 창의 내용을 출력하는 setMessage() 함수 말고도 다양한 함수가 있다.
<pre>
// 알림 창의 내용을 출력하는 함수
open fun setItems(items: Array<CharSequence>!, listener: DialogInterface.OnClickListener!): AlertDialog.Builder!
open fun setMultiChoiceItems(items: Array<CharSequence>!, checkedItems: BooleanArray!, 
                             listener:DialogInterface.OnMultiChoiceClickListener!): AlertDialog.Builder!
open fun setSingleChoiceItems(items: Array<CharSequence>!, checkedItem: Int, listener: DialogInterface.OnClickListener!)
                              : AlertDialog.Builder!
</pre>
위 함수는 목록을 제공하고 이 중 하나를 선택받는 알림 창이다.
<br>
<br>
setItems() 함수의 두 번째 매개변수는 항목을 선택할 때의 이벤트 핸들러이며,
<br>
사용자가 항목을 선택하면 onClick() 함수가 자동으로 호출된다.
<br>
사용자가 선택한 항목의 인덱스는 onClick() 함수의 두 번째 매개변수로 전달된다.
<pre>
// setItems() 예제
val items = arrayOf<String>("사과", "복숭아", "수박", "딸기")
AlertDialog.Builder(this).run {
    setTitle("items test")
    setIcon(android.R.drawable.ic_dialog_info)
    setItems(items, object: DialogInterface.OnClickListener {
        override fun onClick(p0: DialogInterface?, p1: Int) {
            Log.d("setItems", "선택한 과일 : ${items[p1]}")
        }
    })
    setPositiveButton("닫기", null)
    show()
}
</pre>
<img src="https://user-images.githubusercontent.com/87363461/189921756-adc03490-de91-4fbb-8660-366cd476c807.JPG" width="200" height="200">
<br>
<br>
setMultiChoiceItems() 함수는 다중 선택을 위한 체크박스가 함께 출력되는 항목을 만들어 준다.
<br>
세 번째 매개변수가 항목을 선택할 때의 이벤트 핸들러이며 사용자가 항목을 선택하는 순간 onClick() 함수가 자동으로 호출된다.
<br>
onClick() 함수의 두 번째 매개변수로 선택한 항목의 인덱스가 전달되고,
<br>
세 번째 매개변수로 체크 상태가 전달된다.
<pre>
// setMultiChoiceItems() 예제
setMultiChoiceItems(items, booleanArrayOf(true, false, true, false), object: DialogInterface.OnMultiChoiceClickListener {
    override fun onClick(p0: DialogInterface?, p1: Int, p2: Boolean) {
        Log.d("setMultiChoiceItems", "${items[p1]} 이 ${if(p2) "선택되었습니다." else "선택 해제되었습니다."}")
    }
})
</pre>
<img src="https://user-images.githubusercontent.com/87363461/189922717-78691d4e-ae17-41e3-9abd-f8dec9b7334a.JPG" width="200" height="200">
<br>
<br>
setSingleChoiceItems() 함수는 하나만 선택할 수 있는 라디오 버튼으로 구성된 항목을 만들어 준다.
<br>
두 번째 매개변수로 처음 선택할 항목을 지정한다.
<pre>
// setSingleChoiceItems() 예제
setSingleChoiceItems(items, 1, object: DialogInterface.OnClickListener {
    override fun onClick(p0: DialogInterface?, p1: Int) {
        Log.d("setSingleChoiceItems", "${items[p1]} 이 선택되었습니다.")
    }
})
</pre>
<img src="https://user-images.githubusercontent.com/87363461/189923319-1c9b6bb8-e3d6-41c5-a271-385a38ff5877.JPG" width="200" height="200">
<br>
<br>
알림 창의 제목, 내용, 버튼을 구성하는 함수 이외에 속성을 설정하는 함수를 사용할 수 있다.
<pre>
open fun setCancelable(cancelable: Boolean): AlertDialog.Builder!
open fun setCanceldOnTouchOutside(cancel: Boolean): Unit
</pre>
두 함수 모두 사용자의 행동에 따라 알림 창을 닫을 것인지를 설정한다.
<br>
setCancelable() 함수는 사용자가 기기의 뒤로가기 버튼을 눌렀을 때,
<br>
<br>
setCanceldOnTouchOutside() 함수는 알림 창의 바깥 영역을 터치 했을 때 매개변수가 true이면 닫고 false 이면 닫지 않는다.
<br>
기본 값은 true이다.
<pre>
// 알림 창을 닫는 설정 예제
AlertDialog.Builder(this).run {
    setTitle("items Test")
    setIcon(android.R.drawable.ic_dialog_info)
    setItems(items, object: DialogInterface.OnClickListener {
        override fun onClick(p0: DialogInterface?, p1: Int) {
            Log.d("Alert", "선택한 과일 : ${items[p1]")
        }
    })
    setCancelable(false)
    setPositiveButton("닫기", null)
    show()
}.setCanceldOnTouchOutside(false)
</pre>

### 커스텀 다이얼로그
커스텀 다이얼로그는 개발자가 원하는 형태로 창을 구성한 다이얼로그로 커스텀 다이얼로그도 AlertDialog를 이용한다.
<br>
<br>
커스텀 다이얼로그를 만들기 위해 LayoutInflater라는 클래스를 이용해야 한다.
<br>
LayoutInfalter 클래스는 레이아웃 XML 파일을 코드에서 초기화(전개)하는 기능을 제공한다.
<br>
여기서 초기화란 XML 파일에 선언한 뷰를 코드에서 이용하고자 생성하는 작업을 말한다.
<br>
XML 파일은 텍스트 파일이라 코드에서 이용하려면 XML에 선언한 대로 객채를 생성해 메모리에 할당해야 한다.
<br>
이 작업을 LayoutInflater가 해준다.
<br>
<br>
LayoutInflater로 레이아웃 XML 파일을 초기화하는 작업은 getSystemService() 함수로 LayoutInflater를 얻는다.
<br>
그리고 inflate() 함수를 호출하면서 초기화할 레이아웃 XML 파일 정보를 매개변수로 전달한다.
<pre>
// XML 파일 초기화
val inflater = getSystemService(Context.LAYOUT_INFLATER_SERVICE) as LayoutInflater
val rootView = inflater.inflate(R.layout.activity_one, null)
</pre>
inflate() 함수의 반환값은 초기화된 XML의 루트 태그에 해당하는 객체이다.
<br>
만약 XML 파일의 루트 태그가 <LinearLayout>이라면 LinearLayout 객체를 반환한다.
<br>
<br>
그런데 뷰 바인딩 기법을 이용한다면 XML 초기화 코드를 조금 더 쉽게 작성할 수 있다.
<pre>
// 뷰 바인딩을 적용한 XML 파일 초기화
val binding = ActivityOneBinding.inflate(layoutInflater)
val rootView = binding.root
</pre>
초기화할 XML에 해당하는 바인딩 클래스의 inflate() 함수를 호출하면서,
<br>
매개변수로 layoutInflater 객체를 전달만 해주면 자동으로 초기화되고 루트 뷰 객체를 얻을 수 있다.
<br>
<br>
[커스텀 다이얼로그 예제]
<br>
<br>
<img src="https://user-images.githubusercontent.com/87363461/190126789-ae4bd9c4-0a9f-433b-9435-ecf4aaefcd0e.JPG" width="200" height="400">

## 소리 알림
사용자에게 짧은 소리로 특정한 상황을 알릴 때가 있는데, 예를 들어 문자나 카카오톡 앱이다.
<br>
이런 짧은 소리를 소리 알림음 이라고 한다.
<br>
<br>
알림음은 카카오톡처럼 자체 녹음한 음원을 쓸 수도 있고 안드로이드 시스템에 등록된 소리를 사용할 수 있다.
<br>
시스템에 등록된 소리를 이용하는 방법은 알림(NOTIFICATION), 알람(ALARM), 벨소리(RINGTONE) 등이 있으며,
<br>
RingtonManager 클래스로 얻을 수 있다.
<pre>
// 소리 얻기
val notification: Uri = RingtoneManager.getDefaultUri(RingtoneManager.TYPE_NOTIFICATION)
val ringtone = RingtoneManager.getRingtone(applicationContext, notification)
ringtone.play()
</pre>
RingtoneManager.getDefaultUri() 함수를 이용해 소리의 식별값을 얻는다.
<br>
이 식별값은 Uri 객체이며 이 값을 RingtoneManager.getRingtone() 함수의 두 번째 매개변수로 전달한다.
<br>
그러면 Ringtone 객체를 얻으며. 이 객체의 play() 함수를 호출하면 소리가 재생된다.

### 자체 음원 재생 방법
음원 파일은 리소스로 등록해서 이용해야 하는데 음원 리소스 디렉터리는 res/raw이다.
<br>
<br>
음원을 재생하는 클래스는 MediaPlayer로 이 클래스에 리소스 정보를 지정하고 start() 함수를 호출하면 음원이 재생된다.
<pre>
// 음원 재생하기
val player: MediaPlayer = MediaPlayer.create(this, R.raw.fallbackring)
player.start()
</pre>

## 진동 알림
진동도 사용자 알림 효과로 많이 이용하며, 앱에서 진동이 울리게 하려면 매니페스트 파일에 퍼미션을 얻어야한다.
<br>
진동은 Vibrator 클래스를 이용하는데, Vibrator 객체를 얻는 방법이 API 레벨 31부터 변경되었다.
31 이전 버전에서는 VIBRATOR_SERVICE로 식별되는 시스템 서비스를 이용했지만,
<br>
31 버전부터는 VIBRATOR_MANAGER_SERVICE로 식별되는 VibratorManager라는 시스템 서비스를 얻고 
<br>
이 시스템에서 Vibartor를 이용해야 한다.
<pre>
// 진동 객체 얻기
val vibrator = if (Build.VERSION_SDK_INT >= Build.VERSION_CODES.S) {
    val vibratorManager = this.getSystemService(Context.VIBRATOR_MANAGER_SERVICE) as Viabrator
                          vibratorManager.defaultVibrator;                          
}
    else {
        getSystemService(VIBRATOR_SERVICE) as Vibrator
    }
</pre>

### 시간과 패턴을 지정해 진동 울리기 (API 레벨 1부터 제공)
이 함수는 API 레벨 1부터 제공하던 함수였으나, 26 버전에서 새로운 함수를 제공하면서 deprecated 되었다
<br>
deprecated는 사용을 보장할 수 없으니 더는 사용하지 말라는 의미이다.
<br>
<br>
이 API를 사용하기 위해서 레벨 호환성을 고려해야 한다.
<br>
<pre>
open fun vibrate(milliseconds: Long): Unit
open fun vibrate(pattern: LongArray!, repeat: Int): Unit
</pre>
첫 번째 함수의 매개변수는 Long 타입 하나로, 이 매개변수는 진동이 울리는 시간을 의미한다(500일 경우 0.5초 ms 단위)
<br>
<br>
두 번째 함수는 매개변수가 2개인데, 진동을 반복해서 울리는 함수이다.
<br>
첫 번째 매개변수에는 진동 패턴을 배열로 지정한다. 
<br>
예를 들어 500, 100, 1500의 배열값을 전달하면 0.5초 울리고 1초 쉬고 1.5초 울린다.
<br>
두 번째 매개변수는 이 패턴을 얼마나 반복할지 지정한다.
<br>
만약 -1로 지정하면 반복하지 않고 패턴대로 한 번만 울리고, 0으로 지정하면 코드에서 cancel() 함수로 끄지 않는 한 계속 울린다.

### 진동의 세기까지 지정해 진동 울리기 (API 레벨 26부터 제공하는 함수)
API 레벨 26부터는 진동 정보를 VibratorEffect 객체로 지정할 수 있는 함수를 제공한다.
<br>
VibrationEffect 객체로는 진동이 울리는 시간 이외에 진동의 세기까지 제어할 수 있다.
<pre>
open fun vibrate(vibe: VibrationEffect!): Unit
</pre>
vibrate() 함수의 매개변수에 VibrationEffect 객체를 지정한다.
<br>
VibrationEffect는 진동 정보를 지정하는 함수를 제공한다.
<pre>
open static fun createOneShot(milliseconds: Long, amplitude: Int): VibrationEffect!
</pre>
이 함수로 만든 VibrationEffect 객체를 vibrate() 함수에 대입하면서 첫 번째 매개변수의 시간 동안 울린다(ms초)
<br>
그리고 두 번째 매개변수를 이용해 진동의 세기를 지정할 수 있다.
<br>
진동의 세기는 0 ~ 255 사이의 숫자로 표현하며, 0이면 진동이 울리지 않고 255면 기기에서 지원하는 가장 센 진동으로 울린다.
<br>
<br>
이렇게 숫자를 직접 대입해도 되고 VibrationEffect.DEFAULT_AMPLITUDE 처럼 상수를 지정할 수도 있다.
<pre>
// 기본 세기로 진동 울리기
if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.0) {
    vibrator.vibrate(VibrationEffect.createOneShot(500, VibrationEffect
}
</pre>

반복해서 진동을 울리는 createWaveform() 함수도 있다.
<pre>
// createWaveform() 함수 생성자
open static fun createWaveform(timings: LongArray!, amplitudes: IntArray!, repeat: Int): VibrationEffect!
</pre>
첫 번째 매개변수는 진동이 울리는 시간의 패턴 배열, 두 번째 매개변수는 진동 세기의 패턴 배열, 세 번째 매개변수는 횟수이다.
<pre>
// 패턴대로 반복해서 울리기
if(Build.VERSION.SDK_INT >= Build.VERSION_CODES.0) {
    vibrator.vibrate(VibrationEffect.createWaveform(longArrayOf(500, 1000, 500, 2000),
    intARrayOf(0, 50, 0, 200), -1))
}
else {
    vibrator.vibrate(longArrayOf(500, 1000, 500, 2000), -1)
}
</pre>
createWaveform() 함수의 첫 번째와 두 번째 매개변수를 보면 각각 4개의 숫자 배열을 대입 했습니다.
<br>
이렇게 하면 처음 0.5초간은 진동이 울리지 않다가 1초간 50만큼의 세기로 올리고, 다시 0.5초간 울리지 않다가
<br>
마지막 2초간 200만큼의 세기로 울린다.

## 알림 띄우기
상대 바는 화면 상단의 한 줄을 의미하며 이곳에 배터리, 네트워크, 시간 등 시스템의 상태 정보가 출력된다.
<br>
이 상태 바에 앱의 정보를 출력하는 것을 알림(notification) 이라고 한다.
<br>
<br>
원래 상태 바는 시스템에서 관리하는 곳이며 앱이 직접 제어할 수 없다.
<br>
그런데 앱에서 시스템에 의뢰하면 시스템에서 관리하는 상태 바에 앱의 알림을 출력할 수 있다.
<br>
따라서 앱의 화면을 구성하거나 사용자 이벤트를 처리하는 프로그래밍과는 구조가 다르고,
<br>
알림을 위해 제공하는 API 함수를 이용해야 한다.
<br>
<br>
<img src="https://user-images.githubusercontent.com/87363461/190139528-e1879903-2bae-48a2-95d2-9c87f9832499.JPG" width="600" height="300">
<br>
<br>
알림은 NotificationManager의 notify() 함수로 발생한다.
<br>
notify() 함수에는 NotificationCompat.Builder가 만들어 주는 Notification 객체를 대입하며 이 객체에는 알림 정보가 저장된다.
<br>
그런데 NotificationCompat.Builder를 만들 때 NotificationChannel 정보를 대입해 주어야 한다.
<br>
<br>
정리하자면 NotificationChannel로 알림 채널을 만들고,
<br>
이 채널 정보를 대입해 NotificationCompat.Builder를 만든 다음,
<br>
이 빌더로 Notification 객체를 만들고,
<br>
Notification 객체를 NotificationManager의 notify() 함수에 대입하는 구조이다.
<br>
<br>
Notification을 만들려면 NotificationCompat.Builder가 필요한데 빌더를 만드는 방법이
<br>
API 레벨 26(Android 8) 버전부터 변경 되었다.
<pre>
// API 레벨 26 버전 이전
Builder(context: Context!)
// API 레벨 26 이상
Builder(context: Context!, channelld: String!)
</pre>
API 레벨 26 버전에서 부터 채널이라는 개념이 추가되었는데, 이는 앱의 알림을 채널로 구분하겠다는 의도이다.
<br>
사용자가 환경 설정에서 어떤 앱의 알림을 받을지 말지를 설정할 수 있다.
<pre>
// 알림 채널 생성자
NotificationChannel(id: String!, name: CharSequence!, importance: Int)
</pre>
매개변수로 채널의 식별값과 설정 화면에 표시할 채널 이름을 문자열로 지정한다.
<br>
세 번째 매개변수는 이 채널에서 발생하는 알림의 중요도이며 하기의 상수로 지정한다.
<ul>
<li><b>NotificationManager.IMPORTANCE_HIGH</b> : 긴급 상황으로 알림음이 울리며 헤드업으로 표시</li>
<li><b>NotificationManager.IMPORTANCE_DEFAULT</b> : 높은 중요도이며 알림음이 울림</li>
<li><b>NotificationManager.IMPORTANCE_LOW</b> : 중간 중요도이며 알림음이 울리지 않음</li>
<li><b>NotificationManager.IMPORTANCE_MIN</b> : 낮은 중요도이며 알림음도 없고 상태 바에도 표시되지 않음</li>
</ul>
<br>
채널의 각종 정보는 함수나 프로퍼티로 설정할 수 있다.
<pre>
fun setDescription(description: String!): Unit : 채널의 설명 문자열
fun setShowBadge(showBadge: Boolean): Unit: 홈 화면의 아이콘에 배지 아이콘 출력 여부
fun setSound(sound: Uri!, audioAttributes: AudioAttributes!): Unit : 알림음 재생
fun enableLights(ligghts: Boolean): Unit : 불빛 표시 여부
fun setLightColor(argb: Int): Unit : 불빛이 표시된다면 불빛의 색상
fun enableVibration(vibration: Boolean): Unit : 진동을 울릴지 여부
fun setVibrationPattern(vibrationPattern: LongArray!): Unit : 진동을 울린다면 진동의 패턴
</pre>
setDescription() 함수에 전달하는 문자열은 설정 화면에서 채널을 설명하는 곳에 보인다.
<br>
그리고 setShowBadge(true0)로 설정하면 홈 화면의 앱 아이콘에 확인하지 않은 알림 개수가 표시된 배지 아이콘이 보인다.
<br>
<br>
<img src="https://user-images.githubusercontent.com/87363461/190141994-594c0ba0-b6f4-4425-bb12-7b8895e1e366.JPG" width="200" height="200">
<br>
<br>
배지 아이콘에 표시되는 숫자는 코드에서 지정하는 값이 아니며 사용자가 확인하지 않은 알림 개수가 자동으로 표시된다.
<br>
아이콘은 코드에서 setShowBadge(true)로 지정했더라도 사용자가 설정에서 변경하면 배지가 출력되지 않는다.
<br>
<br>
enableVibration() 함수를 이용해 알림이 발생할 때 진동이 울리게 할 수 있으며, 만약 특정 패턴으로 울려야 한다면
<br>
setVibrationPattern() 함수를 함께 이용해 진동 패턴을 설정한다.
<pre>
// 알림 빌더 예제
val manager = getSystemService(NOTIFICATION_SERVICE) as NotificationManager
val builder: NotificationCompat.Builder

if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.0) {
    val channelId = "one-channel"
    val channelName = "My Channel One"
    val channel = NotificationChannel(channelId, channelName, NotificationManager.IMPORTANCE_HIGH)
    
    // 채널에 다양한 정보 설정
    channel.description = "My Channel One Description"
    channel.setShowBadge(true)
    
    val uri: Uri = RingtoneManager.getDefaultUri(RingtoneManager.TYPE_NOTIFICATION)
    val audioAttributes = AudioAttributes.Builder()
                          .setContentType(AudioAttributes.CONTENT_TYPE_SONIFICATION)
                          .setUsage(AudioAttributes.USAGE_ALARM)
                          .build()
    channel.setSound(uri, audioAttributes)
    channel.enableLights(true)
    channel.lightColor = Color.RED
    channel.enableVibration(true)
    channel.vibrationPattern = longArrayOf(100, 200, 100, 200)
    
    // 채널을 NotificationManager의 등록
    manager.createNotificationChannel(channel)
    
    // 채널을 이용해 빌더 생성
    builder = NotificationCOmpat.Builder(this, channelId)
}
else {
    builder = NotificationCompat.Builder(this)
}
</pre>

### 알림 객체
알림 빌더를 만든 후 이 빌더를 이용해 Notification 객체를 만들어야 한다.
<br>
이 객체에 출력할 이미지, 문자열 등의 정보를 담고 알림이 발생하면 상태 바에 스몰 아이콘이 출력된다.
<br>
<br>
<img src="https://user-images.githubusercontent.com/87363461/190145080-9e128192-2b54-41ef-9495-fcb901754c73.JPG"  width="400" height="100">
<img src="https://user-images.githubusercontent.com/87363461/190145087-82f09d59-4b62-410d-a884-ed8d51e60c55.JPG"  width="400" height="300">
<br>
<br>
알림은 스몰 아이콘과 발생 시각, 제목, 내용 등으로 구성되며 이러한 정보를 Notification 객체에 설정해야 한다.
<pre>
// 알림 객체 설정
builder.setSmallIcon(android.R.drawble.ic_notification_overlay)
builder.setWhen(System.currentTimeMillis())
builder.setContentTitle("Content Title")
builder.setCOntentText("Content Message")
</pre>
빌더의 세터 함수를 이용해 알림의 구성 정보를 설정할 수 있다.
<br>
여기까지 작성했다면 NotificationManager 클래스의 notify() 함수를 이용해 알림을 띄운다.
<pre>
// 알림 발생
manager.notify(11, builder.build())
</pre>
builder.build() 함수가 Notification 객체를 만들고 이로써 알림이 발생한다.
<br>
첫 번째 매개변숫값은 알림을 식별하는 데 사용하는 숫자이며 개발자가 임의로 지정한다.
<br>
이 식별값은 사용자 폰에 발생한 알림을 코드에서 취소할 때 사용하며 이때 cancel() 함수를 이용한다.
<pre>
// 알림 취소
manager.cancel(11)
</pre>
사용자가 알림을 터치하면 이벤트가 발생할 수 있으며 이때 알림은 화면에서 자동으로 사라진다.
<br>
또한 사용자가 스와이프로 취소할 수 있다.
<br>
<br>
스와이프나 터치로 알림이 사라지지 않게 하려면 빌더의 세터 함수로 지정해야 한다.
<pre>
// 알림 취소 막기
builder.setAutoCancel(false)
builder.setOngoing(true)
</pre>
setAutoCancel(false)로 지정 시 알림을 터치할 때 이벤트는 발생하지만 알림은 사라지지 않는다.
<br>
setOngoing(true)로 지정하면 사용자가 알림을 스와이프해도 사라지지 않는다.
<br>
<br>
만약 2가지를 모두 설정했다면 사용자가 알림을 취소할 수 없으며 특정 순간에 cancel() 함수로 취소해야 한다.

### 알림 터치 이벤트
알림은 사용자에게 앱의 상태를 간단하게 알려 주는 기능을 하는데 , 사용자가 더 많은 정보를 요구할 수 있다.
<br>
그래서 대부분 앱은 사용자가 알림을 터치했을 때 앱의 액티비티 화면을 실행한다.
<br>
이렇게 구현하기 위해 알림의 터치 이벤트를 구현해야 한다.
<br>
<br>
그런데 알림은 앱이 관할하는 화면이 아니며 시스템에서 관리하는 상태 바에 출력하는 정보이다.
<br>
그러므로 이 알림에서 발생한 터치 이벤트는 앱의 터치 이벤트로 처리할 수 없다.
<br>
<br>
결국 앱에서는 사용자가 알림을 터치했을 때 실행해야 하는 정보를 Notification 객체에 담아 두고,
<br>
실제 이벤트가 발생하면 Notification 객체에 등록된 이벤트 처리 내용을 시스템이 실행하는 구조로 처리한다.
<br>
<br>
<img src="https://user-images.githubusercontent.com/87363461/190147059-3fb43d34-ee43-4df9-b97e-7c4dce3ec24f.JPG" width="400" height="300">
<br>
<br>
사용자가 알림을 터치하면 앱의 액티비티 또는 브로드캐스트 리시버를 실힝해야 하는데,
<br>
이를 실행하기 위해 인텐트(Intent)를 이용해야 한다.
<br>
<br>
이 인텐트는 앱의 코드에서 준비하지만 실제 컴포넌트를 실행하는 시점은 앱에서 정할 수 없다.
<br>
그래서 인텐트를 준비한 후 Notification 객체에 담아 이벤트 발생 시 시스템에 실행해 달라고 의뢰한다.
<br>
<br>
이러한 의뢰는 PendingIntent 클래스를 이용하며 컴포넌트별 실행 의뢰 함수를 제공한다.
<pre>
static fun getActivity(context: Context!, requestCode: Int, intent: Intent!, flags: Int): PendingIntent!
static fun getBroadcast(context: Context!, requestCode: Int, intent: Intent!, flags: Int): PendingIntent!
static fun getService(context: Context!, requestCode: Int, intent: Intent!, flags: Int): PendingIntent!
</pre>
각 함수의 세 번째 매개변수에 인텐트 정보를 등록한다.
<br>
이 함수들의 네 번째 매개변수는 flag값으로 똑같은 알림이 발생했을 때 어떻게 처리해야 하는지를 나타낸다.
<br>
여기에 입력할 상수 변수로는 하기 하나를 지정한다.
<ul>
<li>FLAG_IMMUTABLE</li>
<li>FLAG_CANCEL_CURRENT</li>
<li>FLAG_MUTABLE</li>
<li>FLAG_NO_CREATE</li>
<li>FLAG_ONE_SHOW</li>
<li>FLAG_UPDATE_CURRENT</li>
</ul>
<br>
API 레벨이 31을 대상으로 한다면 FLAG_MUTABLE과 FLAG_IMMUTABLE 중 하나를 지정해 주어야 한다.
<br>
<br>
알림을 터치했을 때 DetailActivity라는 액티비티의 실행 정보를
<br>
Notification 객체에 등록하고 터치 이벤트 등록은 빌더의 setContentIntent() 함수를 이용한다.
<pre>
// 알림 객체에 액티비티 실행 정보 등록
val intent = Intent(this, DetailActivity::class.java)
val pendingIntent = PendingIntent.getActivity(this, 10, intent, PendingIntent.FLAG_IMMUTABLE)
builder.setContentIntent(pendingIntent) // 터치 이벤트 등록
</pre>

### 액션
알림에는 터치 이벤트 이외에도 액션을 최대 3개까지 추가할 수 있다.
<br>
알림에서 간단한 이벤트는 액션으로 처리하며 앱의 알람 취소, 전화 앱의 수신이나 거부 등이 대표적인 예이다.
<br>
<br>
<img src="https://user-images.githubusercontent.com/87363461/190149805-28dbe4f9-20fc-4848-bcc2-2e2b5d71a95b.JPG" width="300" height="200">
<br>
<br>
액션도 이벤트 처리가 목적으로 액션을 터치할 때 인텐트 정보를 PendingIntent로 구성해 등록해야 한다.
<br>
실제 사용자가 액션을 터치하면 등록된 인텐트가 시스템에서 실행되어 이벤트가 처리되는 구조이다.
<pre>
// 액션 등록 함수
open fun addAction(action: Notification.Action!): Notification.Builder
</pre>
매개변수로 액션의 정보를 담는 Action 객체를 전달한다.
<pre>
// 액션 빌더 생성자
</pre>
액션 빌더의 생성자에 아이콘 정보와 액션 문자열, 사용자가 액션을 클릭했을 때 이벤트 PendingIntent 객체를 전달한다.
<pre>
// 액션 등록하기
val actionIntent = Intent(this, OneReceiver::class.java)
val actionPendingIntent = PendingIntent.getBroadcast(this, 20, actionIntent, PendingIntent.FLAG_IMMUTABLE)
builder.addAction(NotificationCompat.Action.Builder(
                  android.R.drawble.stat_notify_more, "Action", actionPendingIntent).build())
</pre>
<img src="https://user-images.githubusercontent.com/87363461/190151319-3a45b3c0-5508-4c27-91bd-af9272b055a5.JPG" width="300" height="200">

### 원격 입력(RemoteInput)
원격 입력이란 알림에서 사용자 입력을 직접 받는 기법이다.
<br>
에디트 텍스트 같은 사용자 입력 뷰를 사용하지 않고 원격으로 액션에서 직접 받아 처리할 수 있다.
<br>
<br>
<img src="https://user-images.githubusercontent.com/87363461/190151729-9c071118-6def-48df-af8d-dc99d6a59ef0.JPG" width="300" height="150">
<br>
<br>
원격 입력도 액션의 한 종류로 RemoteInput에 사용자 입력을 받는 정보를 설정한 후 액션에 추가하는 구조이다.
<pre>
val KEY_TEXT_REPLY = "key_text_reply"
val replyLabel: String = "답장"
var remoteInput: RemoteInput = RemoteInput.Builder(KEY_TEXT_REPLY).run {
    setLabel(replyLabel)
    build()
}
</pre>
RemoteInput 빌더에 들어가는 정보는 입력을 식별하는 값과 입력란에 출력되는 힌트 문자열이다.
<br>
식별값은 개발자가 임의로 작성할 수 있으며 사용자가 입력한 글을 가져올 때 사용한다.
<br>
<br>
그런데 RemoteInput은 API 레벨 20에서 추가되었으므로 minSdk를 20 아래로 설정했다면 API 레벨 호환성을 고려해야 한다.
<br>
앱의 API 레벨로 if~else 문을 작성해도 되지만, 호환성을 돕는 라이브러리가 있다.
<pre>
androidx.core.app.RemoteInput
</pre>
RemoteInput도 액션이므로 액션의 터치 이벤트를 처리하기 위한 PendingIntent를 준비해야 한다.
<pre>
// 액션 인텐트 준비
val replyIntent = Intent(this, ReplyReceiver::class.java)
val replyPendingIntent = PendingIntent.getBroadcast(this, 30, replyIntent, PendingIntent.FLAG_MUTABLE)

// 원격 입력 액션 등록하기
builder.addAction(NotificationCompat.Action.Builder(
                  R.drawble.send, "답장", replyPendingIntent).addRemoteInput(remoteInput).build))
</pre>
<br>
<img src="https://user-images.githubusercontent.com/87363461/190153049-6d29bf7a-8357-4246-b145-29df4da91087.JPG" width="600" height="150">

### 프로그레스(Progress)
앱에서 어떤 작업이 이루어지는 데 시간이 걸리는 것을 알람을 이용해 진행 상황을 프로그레스로 알려준다.
<br>
알림의 프로그레스 바는 화면을 따로 준비하지 않고 빌더에 setProgress() 함수만 추가해주면 자동으로 나온다.
<pre>
// 포로그레스 바 생성 함수
open fun setProgress(max: Int, progress: Int, indeterminate: Boolean): Notification.Builder
</pre>
첫 번째 매개변수가 프로그레스 바의 최댓값이며 두 번째 매개변수가 진행값이다.
<br>
처음에 현잿값을 지정한 후 스레드같은 프로그램을 사용해 진행값을 계속 바꾸면서 상황을 알려 주면 된다.
<br>
그리고 만약 세 번째 매개변숫값이 true면 프로그레스 바는 왼쪽에서 오른쪽으로 계속 흘러가듯이 표현된다.
<br>
<pre>
// 스레드을 이용해 10초 동안 프로그레스 바의 진행값을 증가시키는 예제


builder.setProgress(100, 0, false)
manager.notify(11, builder.build())

thread {
    for(i in 1..100) {
        builder.setProgress(100, i, false)
        manager.notify(11, builder.build())
        SystemClock.sleep(100)
    }
}
</pre>
<br>
<br>
<img src="https://user-images.githubusercontent.com/87363461/190154768-f1e623be-d15a-4abc-9d4d-fc15beb92a22.JPG" width="300" height="100">

## 알림 스타일
알림에 보이는 정보에 문자열 이외에 다양한 콘텐츠로 알림을 구성할 수 있다.
### 큰 이미지 스타일
알림에 큰 이미지를 출력할 때는 BigPictureStyle을 이용한다.
<pre>
// 큰 이미지 스타일
val pigPicture = BitmapFactory.decodeResource(resources, R.drawble.test)
val bigStyle = NotificationCompat.BigPictureStyle()
bigStyle.bigPicture(big.Picture)
builder.setStyle(bigStyle)
</pre>
<img src="https://user-images.githubusercontent.com/87363461/190157405-ab7b236c-d71a-4a4f-9467-777e6cab21c9.JPG" width="200" height="150">
<br>
<br>
BigPictureStyle 객체의 bigPicture 프로퍼티에 출력할 이미지를 비트맵 형식으로 지정한다.

### 긴 텍스트 스타일
알림에 긴 문자열을 출력해 사용자가 앱을 실행하지 않아도 많은 정보를 알 수 있게 한다.
<br>
대표적으로 이메일 앱은 알림과 제목, 발신자, 일부 내용도 보여준다.
<br>
긴 문자열 알림은 BigTextStyle을 이용한다.
<pre>
// 긴 텍스트 스타일
val bigTextStyle = NotificationCompat.BigTextStyle()
bigTextStyle.bigText(resources.getString(R.string.long_text))
builder.setStyle(bigTextStyle)
</pre>
<img src="https://user-images.githubusercontent.com/87363461/190158268-d2477b91-c56a-4387-85a9-f77ce048a4de.JPG" width="200" height="200">
