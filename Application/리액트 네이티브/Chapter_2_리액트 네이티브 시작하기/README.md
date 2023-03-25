# Chapter_2_리액트 네이티브 시작하기 정리 내용

## 리액트 네이티브 개발 환경
> 윈도우 환경 기준
리액트 네이티브를 사용하기 위해 Node.js, JDK, 안드로이드 스튜디오, 파이썬이 필요하다.
<br>
이외에 맥은 왓치맨(Watchman), Xcode 등이 필요하다.

<br>

### Node.js 다운로드
> Node.js 다운로드 링크 : https://nodejs.org/ko

Node.js를 다운로드 하기 위해 아래 이미지에서 왼쪽, LTS 버튼을 클릭하여 다운로드 한다.

![image](https://user-images.githubusercontent.com/87363461/227703532-b71a5a95-1e14-4957-afd0-85f7cfb2ca74.png)

<br>

### Python 다운로드
> Python 다운로드 링크 : https://www.python.org/downloads/windows/

리액트 네이티브에서는 Python 2 버전을 사용하기 때문에, Python 2 버전을 설치해야 한다.
<br>
여기서는 Python 2.7.18 버전을 다운로드 한다.

<br>
<br>

아래 이미지에서 4번 째 항목인 Download Windows x86-64 MSI installer 을 클릭하여 다운로드 한다.

![image](https://user-images.githubusercontent.com/87363461/227703620-c129284c-c900-4918-a6a5-bbfc08a6382c.png)

<br>

### JDK 설치
> JDK 설치 다운로드 링크 : https://www.oracle.com/kr/java/technologies/downloads/#jdk20-windows

JDK 다운로드 홈페이지에서 Windows - x64 Installer 항목을 클릭하여 다운로드 한다.

![image](https://user-images.githubusercontent.com/87363461/227703675-2f21c8a0-ac9f-4a07-8639-7fbb2cd6b3f8.png)

<br>

JDK 다운로드가 완료되면 환경 변수를 설정해야 한다.
<br>
시스템 환경 변수 편집 -> 환경 변수 -> 시스템 변수 -> 새로 만들기에 진입하여 시스템 변수를 추가한다.

![image](https://user-images.githubusercontent.com/87363461/227703840-5375ead5-4878-44a3-bb41-51d11ac27ecb.png)

<br>

![image](https://user-images.githubusercontent.com/87363461/227703906-5e983988-30f9-425b-9644-e052fb588cda.png)

이렇게 JAVA_HOME 추가가 완료되면 시스템 변수에서 Path를 선택하고 다음과 같이 수정한다.
<br>
시스템 변수 -> Path -> 편집 -> 새로 만들기를 하여 추가한다.

![image](https://user-images.githubusercontent.com/87363461/227703983-81afcaea-b31a-40e1-80e0-f34ff5d827ff.png)

여기까지 완료가 되었으면 프롬프트 창을 켜서 정상적으로 다운로드가 되었는지 확인한다.
```
java -version
javac -version
```

![image](https://user-images.githubusercontent.com/87363461/227704015-91f17945-34fe-4903-8c7a-68f41a75a42c.png)

<br>

### Android Studio 다운로드
> Android Studio 다운로드 링크 : https://developer.android.com/studio

Android Studio를 다운로드 하기 위해 다운로드 홈페이지로 접속한다.
<br>
아래 이미지에서 초록색 다운로드 버튼을 클릭하여 다운로드 한다.

![image](https://user-images.githubusercontent.com/87363461/227704117-25d800f2-c0aa-4aa0-bd5e-3f8ec391ded1.png)

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

이후 계속 Next를 클릭하여 설정을 진행한다.
<br>
<br>
설정이 완료되면 아래와 같이 완료 화면이 나타난다.
<br>
More Actions - SDK Manager를 실행한다.

<br>

![image](https://user-images.githubusercontent.com/87363461/227704791-42f44b16-b9fb-447f-905b-21b5f03c7671.png)

<br>

Android SDK 탭에서 SDK Platforms 필요 항목들을 설치한다.
> 우측 하단 Show Packages Details를 클릭하여 전체 항목을 볼 수 있음

- Android 11.0 (R)
  - Android SDK Platform 30
  - Google APIs Intel x86 Atom System Image

<br>

![image](https://user-images.githubusercontent.com/87363461/227705033-a839c9e7-719a-4d29-bb9a-122b1897f262.png)

<br>

- Android 10.0 (Q)
  - Android SDK Platform 29
  - Intel x86_64 Atom System Image
  - Google APIs ARM 64 v8a System Image

- ![image](https://user-images.githubusercontent.com/87363461/227705062-5fb3d266-725b-464d-845c-1e223fcedda2.png)

이후 바로 오른쪽 탭인 SDK Tools 를 클릭하여 전체 항목을 확인한다.
<br>
그 다음 Android SDK Buiild-Tools 아래 항목에서 29.0.2를 클릭하여 다운로드를 진행한다.

<br>

![image](https://user-images.githubusercontent.com/87363461/227705147-f4e03c02-3ae9-4b97-8548-4e8a9576a30b.png)

<br>

여기까지 Android Studio 설정이 완료 되었으면 시스템 환경 변수 설정을 해야한다.
<br>
시스템 환경 변수 편집 -> 환경 변수를 클릭하여 사용자 변수에 새로 만들기를 통해 다음과 같이 추가한다.

- 변수 이름 : ANDROID_HOME
- 변수 값 : %LOCALAPPDATA%\Android\Sdk

여기서 변수 값에는 Android SDK 위치를 정확하게 입력해야 한다.
<br>
이 값은 SDK Manager 메뉴 - Android SDK Locations에서 확인할 수 있다.

<br>

![image](https://user-images.githubusercontent.com/87363461/227705479-037e61cc-869d-4fea-ba8c-eeb27be661e8.png)

<br>

![image](https://user-images.githubusercontent.com/87363461/227705504-6ce67f4b-bf3c-48bc-8584-37b6d83ede1c.png)


<br>

사용자 변수가 추가되면, 다시 시스템 변수에 있는 Path를 추가해야 한다.
<br>
시스템 변수 -> Path -> 새로 만들기를 추가하여 다음과 같이 추가한다.

- 변수 값 + platform-tools

<br>

![image](https://user-images.githubusercontent.com/87363461/227705567-61902caa-938f-4795-bd6e-76000b69116e.png)

<br>

여기 까지 작업을 완료하고 정상적으로 되었는지 확인하기 위해 프롬프트를 실행하고 아래 명령어를 실행한다.
```
adb --version
```

![image](https://user-images.githubusercontent.com/87363461/227705620-d1ef7f98-6678-4b68-8e94-a85589826329.png)

<br>

### 에뮬레이터 설정

에뮬레이터를 설정하기 위해 먼저 Android Studio를 실행한다.
<br>
그리고 More Actions - Virtual Device Manager 항목을 클릭한다.
<br>
<br>
생성한 디바이스가 없으므로 Create virtual device를 클릭한다.

<br>

![image](https://user-images.githubusercontent.com/87363461/227705722-9e625cf8-0816-40f6-9e1b-a8d38f8dbe3d.png)

<br>

먼저 Phone 탭에서 Pixcel 3을 클릭하고 Next 버튼을 클릭한다.

<br>

![image](https://user-images.githubusercontent.com/87363461/227705774-d7e7fca5-40d3-4bc8-93b2-fbee1fb6baa0.png)


<br>

이후 Q를 선택하고 Next -> Finish를 선택하면 최종적으로 설정이 완료된다.

![image](https://user-images.githubusercontent.com/87363461/227705936-545344ec-fe10-49a8-b288-370d6bfd174a.png)


<br>

여기서 만약 아래 그림과 같이 에러가 발생하면 Release Name 옆에 다운로드 버튼을 클릭하면 된다.

<br>

![image](https://user-images.githubusercontent.com/87363461/227705862-ef555474-6d0f-4166-a0ba-1dd9b86ffc4c.png)


<br>

### 에디터 다운로드
> Visual Studio Code 다운로드 링크 : https://code.visualstudio.com/download

에디터는 Visual Studio Code를 사용한다.
<br>
아래 이미지에서 Windows 버튼을 클릭하여 다운로드를 진행한다.

<br>

![image](https://user-images.githubusercontent.com/87363461/227705993-6a88975d-715b-4fe7-95d1-6dfa8e487ae7.png)

<br>


## 리액트 네이티브 프로젝트 만들기
리액트 네이티브 프로젝트를 만들기 위한 방법으로 2가지로 Expo를 이용하는 방법과 리액트 네이티브 CLI를 이용하는 방법이 있다.

<br>

### Expo
Expo는 쉽게 말해 리액트 네이티브를 편하게 사용할 수 있도록 미리 여러 가지 설정이 되어 있는 툴이라고 생각하면 된다.
<br>
Expo를 사용하기 위해 Expo 사이트에서 회원가입 후 Expo 프로젝트를 진행한다.
> Expo 사이트 링크 : https://expo.dev/

<br>

Expo를 이용하여 리액트 네이티브를 생성하기 위해 터미널에서 아래와 같이 명령어를 입력한다.
```
npm install --global expo-cli
```

![image](https://user-images.githubusercontent.com/87363461/227706626-24f8cb7e-cca8-4460-805d-a4622008680c.png)

<br>

다운로드가 완료되면 아래 명령어로 Expo 프로젝트를 생성한다.
```
expo init [Project Name]
```
명령어를 입력하면 어떤 프로젝트를 생성할 것인지 선택할 수 있다.
<br>
여기선 balnk를 선택한다.

<br>

![image](https://user-images.githubusercontent.com/87363461/227706691-fe4d10cf-c5cc-4521-bd97-ee6a3f03d5f9.png)

<br>

프로젝트 생성이 완료되면 생성된 프로젝트 폴더로 이동하여 프로젝트를 실행한다.
```
cd [Project Name]
npm start
```
![image](https://user-images.githubusercontent.com/87363461/227706790-7ec0df62-9169-4425-ad75-23460150896a.png)

<br>

이후 안드로이드에서 실행하기 위해 실제 핸드폰 또는 에뮬레이터에서 expo를 설치해야 한다.
<br>
키보드 a를 눌러 안드로이드 에뮬레이터를 실행한다.
<br>
에뮬레이터에서 Google Play Store - Expo를 다운로드 후 실행한다.

<br>

![image](https://user-images.githubusercontent.com/87363461/227707112-e502de79-3c96-46df-b78d-ed6936eb15e5.png)


<br>

정상적으로 실행된 것을 볼 수 있다.

![image](https://user-images.githubusercontent.com/87363461/227707238-3a780b8e-a9eb-440a-85cf-27eb96ece904.png)
