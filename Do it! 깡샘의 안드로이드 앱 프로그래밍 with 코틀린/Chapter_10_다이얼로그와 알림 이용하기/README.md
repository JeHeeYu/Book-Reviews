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
<img src="https://
-images.githubusercontent.com/87363461/189881461-0bbbb59f-be89-4eb3-8c63-18f6f2cee481.JPG" width="400" height="200">
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
