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
