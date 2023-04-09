# Chapter 1. 플러터 시작하기 정리 내용


## 1. 플러터의 등장 배경

## 리액트 네이티브와 플러터
페이스북에서 만든 리액트 네이티브(React Native)는 여러 운영체제에서 동적으로 앱을 개발할 수 있는 크로스 플랫폼 앱 개발  프레임워크이다.
<br>
특히 웹 개발자에게 익숙한 자바스크립트를 사용하므로 웹 개발자가 새로운 언어를 배우지 않고서도 앱을 개발할 수 있는 장점이 있다.
<br>
<br>
또한 네이티브 언어로 앱을 개발할 때 UI가 변경될 때마다 다시 빌드해야 하지만, 리액트 네이티브는 화면에 바로 표시되므로 개발 효율도 좋다.
<br>
<br>
반면 플러터는 똑같은 크로스 플랫폼 앱 개발 프레임워크지만, 구글에서 만든 다트(Dart)라는 언어를 사용한다.
<br>
따라서 자바나 C#과 같은 컴파일 언어가 가진 특징을 활용해 앱을 개발할 수 있다.
<br>
플러터는 크게 프레임워크와 엔진, 임베더 계층으로 구성되어 있다.
<br>
프레임워크 게층에는 다트 언어로 개발된 여러 가지 클래스가 있으며, 이러한 클래스를 이용해 앱을 개발한다.
<br>
<br>
그리고 엔진 계층은 플러터의 코어를 담당하며 이 부분의 대부분은 C와 C++ 언어로 만들어졌다.
<br>
여기서는 데이터 통신, 다트 컴파일, 렌더링, 그리고 시스템 이벤트 등을 처리한다.

![image](https://user-images.githubusercontent.com/87363461/230757676-7d247d45-98d1-46b2-a8c6-e0686e9b8a08.png)

<br>

![image](https://user-images.githubusercontent.com/87363461/230757685-ed090381-be4d-4e48-abf0-8e4d3ef37597.png)


<br>


## 3. 개발 환경 준비

### Android Studio 다운로드
> Android Studio 다운로드 링크 : https://developer.android.com/studio

Android Studio를 다운로드 하기 위해 다운로드 홈페이지로 접속한다.
<br>
아래 이미지에서 초록색 다운로드 버튼을 클릭하여 다운로드 한다.

![image](https://user-images.githubusercontent.com/87363461/227704117-25d800f2-c0aa-4aa0-bd5e-3f8ec391ded1.png)

<br>

Android Studio 다운로드가 완료되면 Android Studio의 설정을 진행해야 한다.
<br>
먼저 Insall Type이 나오면 Custom을 클릭하여 진행한다.

<br>

![image](https://user-images.githubusercontent.com/87363461/227704446-9d09e708-9e45-41ec-badf-bd0bbadeebad.png)

<br>

다음 Next를 하다 보면 SDK Components Setup 항목이 나온다.
<br>
아래 이미지와 같이 설정하고 Next를 클릭한다.
> 여기서 맨 아래 항목인 Android Virtual Device 가 unavailable로 나오고 클릭이 안되는데, 상관 없음

![image](https://user-images.githubusercontent.com/87363461/227704613-abdeb64c-050c-444b-82d9-5b8c7140d9dd.png)

<br>

설치가 완료되면 안드로이드 스튜디오에서 플러터를 실행할 수 있도록 플러그인을 다운로드 해야한다.
<br>
Plugin을 실행하고, 검색 창에 flutter를 입력한다.

<br>
맨 위에 나오는 Flutter 플러그인을 Install 한다.

<br>

![image](https://user-images.githubusercontent.com/87363461/230758286-bf56754c-75e0-4c13-bc6b-01f03145ab5a.png)

<br>

이후 상단에 New Flutter Project가 생긴 것을 볼 수 있다.

<br>

![image](https://user-images.githubusercontent.com/87363461/230758398-d487b052-89de-4610-95b3-43e4473b81cf.png)


<br>

### 플러터 SDK 다운로드
> 플러터 SDK 다운로드 링크 : https://docs.flutter.dev/get-started/install

플러터 SDK 다운로드 링크로 접속한 다음, 원하는 OS를 클릭해 다운로드 한다.

<br>

![image](https://user-images.githubusercontent.com/87363461/230757857-ee2bf7f5-ddcc-4ac9-ac78-c9f9856f61b0.png)

<br>

![image](https://user-images.githubusercontent.com/87363461/230757868-f2deac89-d876-4508-8da1-07d31cea7204.png)


<br>

내려 받은 파일의 압축을 해제한다.
<br>
여기서는 D:W에 압축을 해제하였고, 그러면 flutter라는 디렉터리가 생성된다.

![image](https://user-images.githubusercontent.com/87363461/230758091-6bc2bdbc-dd83-42a1-b0a1-88289cdb495b.png)

<br>

이후 플러터가 정상 설치되었는지 확인하기 위해 프롬프트를 실행하고 SDK가 설치된 곳의 bin 디렉터리로 이동한다.
<br>
그리고 다음 명령어를 실행한다.

```
flutter doctor
```

<br>

![image](https://user-images.githubusercontent.com/87363461/230758137-7925467a-1497-49e0-b266-b187c04666df.png)

![image](https://user-images.githubusercontent.com/87363461/230758163-b56d69b5-7998-4d9f-815c-f2e300f71368.png)

<br>

현재는 두 번째 줄의 [v] Flutter 칸만 보면 된다.



## 4. 프로젝트 만들기

Android Studio를 실행하고 Create New Flutter Project를 선택한다.

<br>

![image](https://user-images.githubusercontent.com/87363461/230758401-91c076a8-db88-4681-bdda-2834d44d6f7a.png)

<br>

먼저 Flutter SDK Path를 다운로드 받았던 파일 내 flutter 폴더를 지정한다.

<br>

![image](https://user-images.githubusercontent.com/87363461/230758431-5d6d8c11-cf1f-4caf-8cd8-491fc268f827.png)

그리고 New Project에서 파일 이름을 지정한다.
<br>
파일 이름은 소문자와 언더바만 사용할 수 있다.

![image](https://user-images.githubusercontent.com/87363461/230758531-5970a320-c2a6-4425-aee1-4b16c656e927.png)
