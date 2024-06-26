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

### 커스텀 컴포넌트(Custom Component)
커스텀 컴포넌트는 여러 컴포넌트을 조합하여 새로운 컴포넌트를 제작해서 사용하는 것을 말한다.
<br>
컴포넌트는 src 폴더 아래 components 폴더를 만들고 이 폴더에서 관리한다.

<br>

![image](https://user-images.githubusercontent.com/87363461/227754146-92d91a71-614a-45aa-bf48-c2fe10c50859.png)

<br>

여기선 커스텀 버튼을 만드는 예제이다.
<br>
커스텀 버튼을 만들기 위해 TouchableOpacity 컴포넌트를 사용한다.
<br>
<br>
components 폴더 아래 js 파일을 만들고, 커스텀 버튼을 정의한다.

<br>

### [예제 코드](https://github.com/JeHeeYu/Book-Reviews/blob/main/Application/%EB%A6%AC%EC%95%A1%ED%8A%B8%20%EB%84%A4%EC%9D%B4%ED%8B%B0%EB%B8%8C/Chapter_3_%EC%BB%B4%ED%8F%AC%EB%84%8C%ED%8A%B8/Custom_Button/src/components/CButton.js)

<br>

![image](https://user-images.githubusercontent.com/87363461/227755232-83c37e4c-03e1-40ef-a82f-4e535c302ff0.png)


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



<br>

## props
먼저 props란 properties의 줄임말로, <b>부모 컴포넌트로부터 전달된 속성값 혹은 상속받은 속성값</b>을 말한다.
<br>
부모 컴포넌트가 자식 컴포넌트의 props를 설정하면 자식 컴포넌트에서는 해당 props를 사용할 수 있지만, 변경은 불가능하다.
<br>
<br>
만약 props의 변경이 필요한 경우 props를 설정 및 전달한 부모 컴포넌트에서 변경해야 한다.
<br>

### props 전달하고 사용하기
부모 컴포넌트에서 props를 전달하기 위해서는 컴포넌트 내 속성을 지정해야 한다.
```
// App.js

<CButton title="Button" />
```
<br>
위 예시 코드처럼 Button 컴포넌트에 title 속성을 지정하면 Button 컴포넌트의 props로 title이 전달된다.
<br>
<br>
자식 컴포넌트인 CButton에서 로그를 출력하면 props 값이 전달되는 것을 볼 수 있다.

```
// CButton.js

const CButton = props => {
    console.log(props);
    return...
}

// 실행 결과

{"title": "Button"}
```
이렇게 전달된 props의 값을 이용할 수도 있다.
<br>
아래 코드는 전달된 props의 title값으로 버튼에 출력되는 문자열을 변경하는 코드이다.
```
// CButton,js

<Text style={{ color: 'white', fontSize: 24}}>{props.title}</Text>
```

<br>

이렇게 props를 전달하는 방법 이외에도 태그 사이에 값을 입력해서 전달할 수도 있다.
```
// App.js

<CButton title="Button">Children Props</CButton>
```
컴포넌트의 태그 사이에 전달된 값은 자식 컴포넌트의 props - children으로 전달된다.
```
// CButton.js

<Text style={{ color: 'white', fontSize: 24}}>{props.children || props.title}</Text>
```

![image](https://user-images.githubusercontent.com/87363461/227755794-1d4e1f60-6bd9-459c-8227-1975d4ec0b1a.png)


<br>

### defaultProps
만약 props를 지정하지 않으면 렌더링 시 어떤 값도 전달되지 않기 때문에 빈 값이 나타난다.
<br>

![image](https://user-images.githubusercontent.com/87363461/227755823-ce1fd607-3e1c-440c-9d50-8b604640a4a6.png)

<br>

여러 사람과 함께 개발을 하다 보면 컴포넌트를 공용해서 사용하는 경우가 많다.
<br>
이런 상황에서 전달되어야 할 중요한 값이 전달되지 않으면 문제가 생기는데, 이러한 경우에 defaultProps를 지정한다.
<br>
즉, 빈 값이 나타나는 상황을 방지할 수 있다.
```
// CButton.js

const CButton = props => { ... };

CButton.defaultProps = {
    title: 'default Button',
};
```
props를 지정하지 않더라도 defaultProps가 들어가는 것을 볼 수 있다.

<br>

![image](https://user-images.githubusercontent.com/87363461/227755917-beced892-071f-4558-8317-bc54facbd900.png)

<br>

### propTypes
propTypes는 컴포넌트에 props를 잘못 전달하는 것을 방지하기 위한 방법이다.
<br>
props가 잘못 전달되었다는 것을 경고 메시지를 통해 알리는 방법이다.
<br>
<br>
propsTypes을 사용하기 위해서는 prop-types 라이브러리를 다운로드 해야한다.
> prop-types Link : https://github.com/facebook/prop-types
> 
```
npm install prop-types
```
<br>
propsTypes를 이용하면 컴포넌트에서 전달받아야 하는 타입과 필수 여부를 저장할 수 있다.
<br>
저장하는 방법은 아래와 같이 popTypes에 설정한다.

```
// CButton.js

import PropTypes from 'prop-types';

CButton.prototype = {
    title: PropTypes.number,
};
```
title에는 문자열(string)이 넘어오지만, PropTypes를 이용해 title에 전달되어야 하는 값의 타입은 숫자(number)로 지정했다.
<br>
이런 경우 아래 이미지와 같이 경고 메시지가 발생하는 것을 볼 수 있다.

<br>

![image](https://user-images.githubusercontent.com/87363461/227759386-52a87e68-39ad-46a7-ac77-f7fd25752d35.png)

<br>

또한 PropTypes를 이용하여 경고가 아닌 필수 여부도 지정할 수 있다.
<br>
필수 여부는 선언된 타입 뒤에 isRequired만 붙여주면 된다.

```
// CButton.js

import PropTypes from 'prop-types';

CButton.propTypes = {
    title: PropTypes.number.isRequired,
};
```

이렇게 PropTypes에는 문자열 또는 숫자 이외에도 함수(funct), 객체(object), 배열(array)등 다양한 타입을 지정할 수 있다.

<br>

### [props 예제 코드](https://github.com/JeHeeYu/Book-Reviews/blob/main/Application/%EB%A6%AC%EC%95%A1%ED%8A%B8%20%EB%84%A4%EC%9D%B4%ED%8B%B0%EB%B8%8C/Chapter_3_%EC%BB%B4%ED%8F%AC%EB%84%8C%ED%8A%B8/PropTypes_Example/src/App.js)

<br>

위 예제 코드는 props 값을 가져오는 여러 가지 방법이며, 각 버튼 클릭 시 props의 값을 확인할 수 있다.

<br>

![image](https://user-images.githubusercontent.com/87363461/227761494-747b3d0d-27fb-44d3-b413-7020740a033f.png)

<br>

## state
props는 부모 컴포넌트에서 받은 값으로 변경할 수 없는 반면, state는 컴포넌트 내부에서 생성되고 값을 변경할 수 있다.
<br>
이를 이용해 컴포넌트 상태를 관리한다.
<br>
<br>
<b>상태(state)란 컴포넌트에서 변화할 수 있는 값을 나타내며, 상태가 변하면 컴포넌트는 리렌더링(re-rendering)</b>된다.
<br>
<br>
과거 리액트 네이티브의 경우 함수형 컴포넌트에서 상태 관리를 할 수 없었으나
<br>
리액트 16.8 버전 이후 버전을 사용하는 리액트 네이티브 0.59 버전부터 Hooks라는 것을 사용해 함수형 컴포넌트에서도 상태관리를 할 수 있게 되었다.
> Hooks FAQ : https://bit.ly/hooks-faq

### useSate
useState는 함수형 컴포넌트에서 상태를 관리할 수 있도록 도와주는 Hooks 중 하나이다.
<br>
사용법은 아래와 같이 사용한다.

```
const [state, setState] = useState(initialState);
```

useState는 상태를 관리하는 변수와 그 변수를 변경할 수 있는 세터(setter) 함수를 배열로 반환한다.
<br>
상태 변수는 직접 변경하는 것이 아니라 useState 함수에서 반환한 세터 함수를 이용해야 한다.
<br>
useState 함수를 호출할 때 파라미터에 생성되는 상태의 초깃값을 전달할 수 있고, 전달하지 않으면 undefined로 설정되어 에러가 발생할 수 있으므로 <b>항상 초깃값을 설정하는 것이 좋다</b>.
<br>
<br>
useState 예제를 실행하기 위해 components 폴더 아래 Counter.js 파일을 생성하고 아래와 같이 코딩한다.

```
// Counter.js

import React, { useState } from 'react';
import { View, Text } from 'react-native';
import CButton from './CButton';

const Counter = () => {
    const [count, setCount] = useState(0);
    return (
        <View style={{ alignItems: 'center'}}>
            <Text style={{ fontSize: 30, margin: 10}}>{count}</Text>
            <CButton title="+1" onPress={() => setCount(count + 1)} />
            <CButton title="-1" onPress={() => setCount(count - 1)} />
        </View>
    );
};

export default Counter;
```

useState 함수를 이용해 숫자의 상태를 관리할 count 변수와 변수 상태를 변경할 수 있는 setCount 세터 함수를 만들었다.
<br>
그리고 useState(0)과 같이 초깃값을 0으로 설정하였다.
<br>
<br>
그리고 +1 버튼과 -1씩 버튼을 두어 버튼 클릭 시 count의 값이 변한다.

```
// App.js
import { View, Text } from 'react-native';
import React from 'react';
import CButton from './components/CButton';
import Counter from './components/Counter';

const App = () => {
    return (
        <View
            style={{
                flex: 1,
                backgroundColor: '#fff',
                alignItems: 'center',
                justifyContent: 'center',
            }}
        >
            <Counter />
        </View>
    );
};

export default App;
```

<br>

![image](https://user-images.githubusercontent.com/87363461/227761947-0bf61b9f-3a9c-4b98-a1bb-f80a493e6c67.png)

### 여러 개의 useState
컴포넌트에서 관리해야 할 상태가 여러 개라면 useState를 여러 번 사용하여 관리할 수도 있다.
<br>
관리해야 하는 상태의 수만큼 useState를 사용하면 된다.

```
import React, { useState } from 'react';
import { View, Text } from 'react-native';
import CButton from './CButton';

const Counter = () => {
    const [count, setCount] = useState(0);
    const [double, setDouble] = useState(0);

    return (
        <View style={{ alignItems: 'center'}}>
            <Text style={{ fontSize: 30, margin: 10}}>count: {count}</Text>
            <Text style={{ fontSize: 30, margin: 10}}>double: {double}</Text>
            <CButton title="+" onPress={() => {
                setCount(count + 1)
                setDouble(double + 2)
            }} />
            <CButton title="-" onPress={() => {
                setCount(count - 1)
                setDouble(double - 2)
            }} />
        </View>
    );
};

export default Counter;
```

[useState 예제 코드](https://github.com/JeHeeYu/Book-Reviews/blob/main/Application/%EB%A6%AC%EC%95%A1%ED%8A%B8%20%EB%84%A4%EC%9D%B4%ED%8B%B0%EB%B8%8C/Chapter_3_%EC%BB%B4%ED%8F%AC%EB%84%8C%ED%8A%B8/useState_Example/src/App.js)

![image](https://user-images.githubusercontent.com/87363461/227762071-2d53a80d-7212-4b76-bf19-42acf04e4c67.png)


## 이벤트(Event)
리액트 네이티브는 사용자의 행동에 따라 상호 작용하는 이벤트를 다양하게 제공한다.
<br>
많은 이벤트가 있으며, 대표적으로 press, changed 등이 있다.

### press Event
press 이벤트는 사용자가 특정 버튼 등을눌렀을 때 발생하는 이벤트이다.
<br>
예를 들어 TouchableOpacity 컴포넌트에서 설정할 수 있는 press 이벤트의 종류는 총 4가지이다.

- onPressIn : 터치가 시작될 때 항상 호출
- onPressOut : 터치가 해제될 때 항상 호출
- onPress : 터치가 해제될 때 onPressOut 이후 호출
- onLongPress : 터치가 일정 시간 이상 지속되면 호출

<br>

### [Press 이벤트 예제 코드](https://github.com/JeHeeYu/Book-Reviews/blob/main/Application/%EB%A6%AC%EC%95%A1%ED%8A%B8%20%EB%84%A4%EC%9D%B4%ED%8B%B0%EB%B8%8C/Chapter_3_%EC%BB%B4%ED%8F%AC%EB%84%8C%ED%8A%B8/Press_Event/src/App.js)

<br>

### 실행 결과

![image](https://user-images.githubusercontent.com/87363461/227762522-7c106242-9a7d-4b5f-a540-a963b10f68df.png)

<br>

![image](https://user-images.githubusercontent.com/87363461/227762530-bc0616a9-027c-4e3b-b6f9-ffaa13644347.png)


### change Event
change Event는 변화를 감지하는 이벤트로 값을 입력하는 TextInput 컴포넌트에서 많이 사용된다.
<br>
TextInput 컴포넌트에 있는 onChange 속성은 TextInput 컴포넌트에 입력된 텍스트가 변경될 때 호출된다.
<br>
그리고 호출되는 함수에 다음과 같은 형태로 인자를 전달한다.

```
{
    ...,
    "nativeEvent": {
        "eventCount": ...,
        "target": ...,
        "text": ...,
    },
    ...
}
```

onChange를 통해 전달되는 내용 중 필요한건 변화된 텍스트 뿐이므로, 이때 onChangeText는 조금 더 간편하게 사용할 수 있다.
<br>
onChangeText는 컴포넌트의 텍스트가 변경되었을 때 변경된 텍스트의 문자열만 인수로 전달하며 호출된다.

<br>

### [Change Event 예제 코드](https://github.com/JeHeeYu/Book-Reviews/blob/main/Application/%EB%A6%AC%EC%95%A1%ED%8A%B8%20%EB%84%A4%EC%9D%B4%ED%8B%B0%EB%B8%8C/Chapter_3_%EC%BB%B4%ED%8F%AC%EB%84%8C%ED%8A%B8/Change_Event/src/App.js)

<br>

![image](https://user-images.githubusercontent.com/87363461/227763951-4cd4e179-ee89-4c62-b866-82123697f143.png)

