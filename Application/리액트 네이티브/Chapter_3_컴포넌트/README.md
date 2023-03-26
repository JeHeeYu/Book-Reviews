# Chapter_3_컴포넌트 정리 내용
 
## 컴포넌트(Component)
컴포넌트란 재사용이 가능한 블럭과 같은 것을 말한다.
<br>
화면에 나타나는 것들도 UI 컴포넌트 요소이며, App.js 파일도 App 이라는 컴포넌트이다.
<br>
<br>
단순히 UI 역할만 하는 것이 아니라 부모로부터 받은 속성(Property)이나 자신의 상태(State)에 따라 표현이 달라지고 다양한 기능을 수행핟나.
<br>
리액트 네이티브는 데이터와 UI 요소의 집합체라고 할 수 있는 컴포넌트를 이용하여 화면을 구성한다.

<br>

### 내장 컴포넌트(Core Component)
리액트 네이티브에서는 다양한 내장 컴포넌트들이 제공된다.
<br>
먼저 가장  대표적인 내장 컴포넌트로 View 컴포넌트, Text 컴포넌트, StyleSheet 등이 있다.
> 리액트 네이티브 컴포넌트는 https://reactnative.dev/docs/components-and-apis 링크에서 확인할 수 있음
<br>
<br>
또한 Button 컴포넌트도 있으며 리액트 네이티브 공식 홈페이지에서 여러 속성들 및 예제를 확인할 수 있다.
> Button 컴포넌트 링크 : https://reactnative.dev/docs/button

버튼 컴포넌트를 사용하면 title, onPress 속성 등을 지정할 수 있다.

- title : 버튼 내부에 출력되는 텍스트
- onPress : 버튼이 눌렀을 때 호출되는 함수 지정

<br>

### [예제 코드](https://github.com/JeHeeYu/Book-Reviews/blob/main/Application/%EB%A6%AC%EC%95%A1%ED%8A%B8%20%EB%84%A4%EC%9D%B4%ED%8B%B0%EB%B8%8C/Chapter_3_%EC%BB%B4%ED%8F%AC%EB%84%8C%ED%8A%B8/Click_Alert.js)

<br>

위 예제 코드는 버튼 클릭 시 Alert가 발생하는 예제 코드이다.

![image](https://user-images.githubusercontent.com/87363461/227753780-78a1cd4f-aa01-47d2-8a0e-43f5e20652f3.png)

<br>

![image](https://user-images.githubusercontent.com/87363461/227753783-17704c36-28c4-4e0d-94e8-bd2a70e37751.png)


<br>

## JSX
JSX는 자바스크립트 내에서 HTML을 작성한 것과 같은 코드들을 말한다.
<br>
JSX는 객체 생성과 함수 호출을 위한 문법적 편의를 제공하기 위해 만들어진 확장 기능으로, 리액트 프로젝트에서 사용된다.
<br>
JSX는 가독성이 높고 작성하기도 쉬우며 XML과 유사하다는 점에서 <b>중첩된 구조를 잘 나타낼 수 있는 장점</b>이 있다.

<br>

### 하나의 부모
JSX에서 여러 개의 요소를 표현할 경우 반드시 <b>하나의 부모로 감싸야</b> 한다.
<br>
예를 들어 아래와 같이 Text 컴포넌트의 경우 View 컴포넌트로 감싸져있는 것을 볼 수 있다.
```
export default function App() {
  return (
    <View style={styles.container}>
      <Text>Open up App.js to start working on your app!</Text>
      <StatusBar style="auto" />
    </View>
  );
}
```

이 코드에서 Text 태그를 부모로 감싸지 않는다면, 에러가 발생하는 것을 볼 수 있다.
```
export default function App() {
  return (
      <Text>Open up App.js to start working on your app!</Text>
      <StatusBar style="auto" />
  );
}
```

![image](https://user-images.githubusercontent.com/87363461/227752651-0852001b-f6af-45a9-9389-a3d3c0536cc6.png)

<br>

이처럼 JSX에서는 반드시 하나의 부모로 나머지 요소들을 감싸서 반환해야 한다.
<br>
<br>
View 컴포넌트는 UI를 구성하는 가장 기본적인 요소로 웹 프로그래밍에서 div와 비슷한 역할을 하는 컴포넌트이다.
<br>
컴포넌트를 반환할 때 View 컴포넌트처럼 특정 역할을 하는 컴포넌트로 감싸지 않고 여러 개의 컴포넌트를 반환하고 싶은 경우는 Fragment 컴포넌트를 사용한다.
```
import { StatusBar } from 'expo-status-bar';
import { Text } from 'react-native';
import React, { Fragment } from 'react';

export default function App() {
  return (
    <Fragment>
      <Text>Open up App.js to start working on your app!</Text>
      <StatusBar style="auto" />
    </Fragment>
  );
}
```
Fragment 컴포넌트를 사용하기 위해 import를 이용하여 불러오고 View 컴포넌트 대신 Framgent를 사용할 수 있다.
<br>
이러한 Fragment 컴포넌트는 단축 문법을 제공하는데, 아래와 같이 제공한다.
```
import { StatusBar } from 'expo-status-bar';
import { Text } from 'react-native';
import React from 'react';

export default function App() {
  return (
    <>
      <Text>Open up App.js to start working on your app!</Text>
      <StatusBar style="auto" />
    </>
  );
}
```

<br>

### JSX의 변수
JSX는 내부에서 자바스크립트의 변수를 전달하여 이용할 수 있다.
<br>
변수 값은 중괄호를 이용하여 사용한다.
```
const App = () => {
    const name = "YuJeHee";
    return (
        <View style={styles.container}>
            <Text style={styles.text}>My name is {name}</Text>
            <StatusBar style="auto" />
        </View>
    );
};
```

![image](https://user-images.githubusercontent.com/87363461/227752945-38645c23-3466-4754-b496-849e0dc57dfb.png)

<br>

### JSX의 조건문
JSX는 내부에서 if문을 사용할 수 있지만, if문을 즉시실행함수 형태로 작성해야 한다.
```
const App = () => {
    const name = "A";
    return (
        <View style={styles.container}>
            <Text style={styles.text}>My name is 
            {(() => {
                if(name === "A") return " return A";
                else if(name === "B") return " return B";
                else return " return else";
            })()}
            </Text>
            <StatusBar style="auto" />
        </View>
    );
};
```

![image](https://user-images.githubusercontent.com/87363461/227753025-84550031-7685-494e-912e-1c33d3e325e9.png)


<br>

### JSX의 삼항 연산자
JSX는 내부에서 if 조건문 이외에도 삼항 연산자를 사용할 수 있다.
```
const App = () => {
    const name = "C";
    return (
        <View style={styles.container}>
            <Text style={styles.text}>
                My name is {name === "A" ? "A" : "B"}
            </Text>
            <StatusBar style="auto" />
        </View>
    );
};
```

![image](https://user-images.githubusercontent.com/87363461/227753058-f81f4241-9af5-47cb-9b53-31cf19d389dc.png)


<br>

### JSX의 AND OR 연산자
JSX는 내부에서 AND와 OR 연산자를 모두 이용할 수 있으며, 잘 이용 시 컴포넌트의 렌더링 여부를 결정할 수 있다.
```
const App = () => {
    const name = "A";
    return (
        <View style={styles.container}>
            {name === "A" && (
                <Text style={styles.text}>My name is A</Text>
            )}
            {name !== "A" && (
                <Text style={styles.text}>My name is Not A</Text>
            )}
            <StatusBar style="auto" />
        </View>
    );
};
```

![image](https://user-images.githubusercontent.com/87363461/227753134-d11b3e49-ba0c-4c4b-9f93-c13e5ee79d3a.png)

<br>

결과 확인 시 두 개의 내용 중 한 가지만 나타나는 것을 확인할 수 있다.
<br>
JSX에서 false는 렌더링되지 않기 때문에 AND 연산자 앞의 조건이 참일 때 뒤의 내용, 거짓인 경우 나타나지 않는다.
<br>
<br>
AND 연산자를 반대로 하면 OR 연산자를 사용할 수 있다.

<br>

### null과 undefined
JSX에서는 조건에 따라 출력하는 값을 변경하다 보면 컴포넌트가 null이나 undefined를 반환하는 경우가 있다.
<br>
JSX의 경우는 null은 허용하지만, undefined는 오류가 발생한다.
```
const App = () => {
    return null;
};

const App = () => {
    return undefined;
};
```
위 코드에서 null은 화면에 아무것도 나타나지 않지만, undefined의 경우 에러가 발생하는 것을 볼 수 있다.

<br>

![image](https://user-images.githubusercontent.com/87363461/227753229-85bb5233-b614-4411-8942-61f6dd98c1dc.png)





