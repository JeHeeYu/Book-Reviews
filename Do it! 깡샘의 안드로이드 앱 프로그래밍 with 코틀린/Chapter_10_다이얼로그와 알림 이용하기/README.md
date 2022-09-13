# Chapter_9_다이얼로그와 알림 이용하기 정리 내용
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
