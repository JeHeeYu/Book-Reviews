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

