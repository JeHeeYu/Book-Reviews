# 플러터 내부 구조 살펴보기 정리 내용

## 플러터 폴더 구조

![image](https://user-images.githubusercontent.com/87363461/233820154-99cde4b6-2e7d-4e8c-9a9f-23beaa048df9.png)

<br>
먼저 lib 폴더에는 플러터 앱의 소스가 작성된 main.dart 파일이 있다.
<br>
그리고 android와 ios 폴더에는 각 운영체제에서 프로젝트를 시작할 때 필요한 파일이 있다.
<br>
<br>
test 폴더에는 다트 코드로 함수를 테스트할 때 사용하는 파일이 있다.
<br>
build 폴더 등은 앱의 설정값을 바꾸면 자동으로 변경되므로 직접 수정하지 않아도 된다.

#### 플러터 프로젝트 주요 폴더

|폴더|내용|비고|
|:---:|:---|:---|
|android|안드로이드 프로젝트 관련 파일|안드로이드 스튜디오로 실행 가능|
|ios|iOS 프로젝트 관련 파일|엑스코드로 실행 가능(맥 전용)|
|lib|플러터 앱 개발을 위한 다트 파일|플러터 SDK 설치 필요|
|test|플러터 앱 개발 중 테스트 파일|테스트 편의성 제공|


루트에도 여러 파일이 있지만 중요한 파일은 pubspec.yaml 파일이다.
<br>
pubspec.yaml 파일은 플러터에서 다양한 패키지와 이미지, 폰트 등을 사용할 수 있게 해준다.

## 메인 소스 파일 구조

### main() 함수
다음은 main 함수이다. 

```
void main() {
  void main() {
  runApp(const MyApp());
}
```

main() 함수에서는 runApp() 함수를 호출한다.
<br>
runApp() 함수는 binding.dart 클래스에 정의되어 있으며 플러터 앱을 시작하는 역할을 한다.
<br>
이 함수에 플러터 앱을 시작하면서 화면에 표시할 위젯을 전달한다.

### MyApp 클래스
MyApp은 runApp() 함수로 플러터 앱을 실행할 때 화면에 표시할 위젯으로 전달한 MyApp 클래스를 정의한 부분이다.


## 위젯 구분
플러터 앱을 구성하는 위젯은 스테이트리스(Stateless)와 스테이트풀(Statefull) 두 가지로 구분할 수 있다.
<br>
이러한 구분으 상태 연결과 관련이 있다.
<br>
<br>
예를 들어, 내용을 갱신할 필요가 없는 위젯은 화면에 보이기 전에 모든 로딩을 마친다.
<br>
예를 들어 앱을 처음 실행하면 사용자에게 정적인 도움말을 보여줄 수 있다.
<br>
이런 도움말 페이지는 갱신할 필요가 없으므로, 앱이 위젯의 상태를 감시하고 있을 필요가 없다.
<br>
<br>
이처럼 상태를 연결할 필요가 없는 위젯을 스테이트리스 위젯이라고 하며, StatelessWidget 클래스를 상속받아 만든다.


![image](https://user-images.githubusercontent.com/87363461/233820900-baa38395-96ef-40fd-b7cf-09f418801b6f.png)

<br>

반면에 내용을 갱신해야 할 때도 있다.
<br>
예를 들어 계산기 앱에서 숫자를 입력하고 계산 버튼을 누르면 결괏값이 화면에 출력되어야 한다.
<br>
이때 버튼을 누를 때마다 화면에 누른 숫자가 반영되어야 한다.
<br>
<br>
즉, 앱이 위젯의 상태를 감시하다가 위젯이 특정 상태가 되면 알맞은 처리를 수행하여야 한다.
<br>
이처럼 상태가 연결된 동적인 위젯을 스테이트 풀 위젯이라고 하며 StateFulWidget 클래스를 상속받아 만든다.

![image](https://user-images.githubusercontent.com/87363461/233820937-51da1b03-f8a5-426b-80ee-046c7e4917e9.png)

<br>
스테이트풀 위젯은 언제든 상태가 변경되면 특정한 처리를 수행해야 하므로 항시 지켜봐야 한다.
<br>
그만큼 메모리나 CPU 등 자원을 많이 소비한다.

## 위젯의 생명 주기
위젯의 생명주기를 알면 언제 데이터를 주고 받을지, 그리고 화면이 사라질 때 어떤 로직을 처리해야 할지를 정리해서 넣을 수 있다.

### 스테이트 풀 위젯의 생명주기
스테이트리스 위젯은 한 번 만들어지면 갱신할 수 없으므로 **생명주기가 없다.** 
<br>
즉, 다른 화면으로 넘어가면 모든 로직이 종료된다.
<br>
<br>
그러나 스테이트풀 위젯은 10단계로 구분하는 생명주기가 있다.

#### 1. 상태를 생성하는 createState() 함수
먼저 StatefulWidget 클래스를 상속받는 클래스는 반드시 createState() 함수를 호출해야 한다.
<br>
이 함수는 다른 생명주기 함수들이 포함된 State 클래스를 반환한다.
<br>
<br>
즉, 위젯의 상태를 생성하는 함수로 생각할 수 있다.

```
class MyHomePage extends StatefulWidget {
  @override
  _MyHomePageState createState() => new _MyHomePageState();
}
```

#### 2. 위젯을 화면에 장착하면 mounted == true
createState() 함수가 호출되어 상태가 생성되면 곧바로 mounted 속성이 true로 변경된다.
<br>
mounted 속성이 true라는 것은 위젯을 제어할 수 있는 buildContext 클래스에 접긓날 수 있다는 의미이다.
<br>
buildContext가 활성화되어야 비로소 setState() 함수를 이용할 수 있다.

```
if(mounted) {
  setState()
}
```

#### 3. 위젯을 초기화하는 initState() 함수
inteState() 함수는 위젯을 초기화할 때 한 번만 호출된다.
<br>
주로 데이터 목록을 만들거나 처음 필요한 데이터를 주고받을 때 호출한다.

```
@override
initState() {
  super.initState();
  _getJsonData();
}
```

initState() 함수를 호출할 때 내부에서 _getJsonData() 함수를 호출해 서버에서 받아온 데이터를 화면에 출력하게 만들 수 있다.
<br>
만약 네트워크 통신이 안되거나 데이터가 이상하다면 화면에 표시하기 전에 미리 알아서 적절하게 대응해야 하므로 위젯을 초기화하는 initState() 함수에서 데이터를 준비해 놓는게 좋다.

#### 4. 의존성이 변경되면 호출하는 didChangeDependencies() 함수
위젯을 초기화하는 initState() 함수가 호출된 후에 이어서 바로 호출되는 함수가 didChangeDependencies()이다.
<br>
이 함수는 데이터에 의존하는 위젯이라면 화면에 표시하기 전에 꼭 호출해야 한다.
<br>
<br>
주로 상속받은 위젯을 사용할 때 피상속자가 변경하면 호출한다.

#### 5. 화면에 표시하는 build() 함수
build() 함수는 Widget을 반환한다.
<br>
즉, 위젯을 화면에 렌더링한다.
<br>
build() 함수에서 위젯을 만들고 반환하면 비로소 화면에 표시된다.

```
@override
Widget build(BuildContext context) {
  return MaterialApp(
    title: 'Flutter Demo',
    theme: ThemeData(
      primarySwatch: Colors.blue,
    ),
  );
}
```

#### 6. 위젯을 갱신하는 didUpdateWidget() 함수
부모 위젯이나 데이터가 변경되어 위젯을 갱신해야 할 때 호출한다.
<br>
만약 initState()에서 특정 이벤트에 의해 위젯이 변경되면 didUpdateWidget() 함수를 호출할 때 위젯을 갱신할 수 있다.
<br>
<br>
initState() 함수는 위젯을 초기화할 때 한 번만 호출되므로 위젯이 변경되었을 때 호출하는 didUpdateWidget() 같은 함수가 필요하다.

```
@override
void didUpdateWidget(Widget oldWidget) {
  if(oldWidget.importantProperty != widget.importantProperty) {
    _init();
  }
}
```

#### 7. 위젯의 상태를 갱신하는 setSate() 함수
setState() 함수를 이용하면 데이터가 변경되었다는 것을 알려주고 변경된 데이터를 이용해 화면의 UI를 변경할 수 있도록 한다.
<br>
앱의 화면을 구성하는 함수이므로 제일 많이 호출하는 함수이다.

```
void updateProfile(String name) {
  setState(() => this.name = name);
}
```

#### 8. 위젯의 상태 관리를 중지하는 deactivate() 함수
deactivate() 함수는 State 객체가 플러터의 구성 트리로부터 제거될 때 호출된다.
<br>
다만 State 객체가 제거됐다고 해서 해당 메모리까지 지워지지는 않는다.
<br>
<br>
deactivate() 함수가 호출되더라도 dispoes() 함수를 호출하기 전까지는 State 객체를 사용할 수 있다.

#### 9. 위젯의 상태 관리를 완전히 끝내는 dispose() 함수
State 객체를 영구적으로 소멸할 때 호출한다.
<br>
이 함수를 호출한다는 것은 이제 해당 위젯을 종료한다는 뜻이다.
<br>
<br>
예를 들어 네트워크 통신을 하거나 스트림 통신을 하다가 dispose() 함수를 호출하면 데이터 전송을 중지한다.
<br>
그리고 위젯을 소멸할 때 꼭 호출해야 하는 함수라면 dispose() 함수 안에서 호출해야 한다.
<br>
<br>
deactivate 함수 호출로 State 객체를 트리에서 제거한 후에 같은 State를 다시 다른 트리에 재사용할 경우 dispose() 함수가 호출되지 않을 수 있다.


#### 10. 위젯을 화면에서 제거하면 mounted == false
State 객체가 소멸하면 마지막으로 mounted 속성이 false로 되면서 생명주기가 끝난다.
<br>
mounted 속성이 false가 되었다는 것은 이 State는 재사용할 수 없다는 의미이다.
<br>
<br>
그러므로 setState() 함수를 호출하면 오류가 발생한다.

## [예제 코드](https://github.com/JeHeeYu/Book-Reviews/blob/main/Application/Do%20it!%20%ED%94%8C%EB%9F%AC%ED%84%B0%20%EC%95%B1%20%ED%94%84%EB%A1%9C%EA%B7%B8%EB%9E%98%EB%B0%8D/Chapter%203.%20%ED%94%8C%EB%9F%AC%ED%84%B0%20%EB%82%B4%EB%B6%80%20%EA%B5%AC%EC%A1%B0%20%EC%82%B4%ED%8E%B4%EB%B3%B4%EA%B8%B0/ButtonClickChange.dart)

- 버튼 클릭 시 String, Color가 바뀌는 예제

![image](https://user-images.githubusercontent.com/87363461/233821705-56ccd037-6fe8-4449-816c-26ae3fd91823.png)

![image](https://user-images.githubusercontent.com/87363461/233821710-29119dd2-64a2-4230-a01c-de5a9da911ee.png)


