# Chapter_4_스타일링 정리 내용

## 스타일링
리액트 네이티브는 자바스크립트를 이용해 스타일링하며 컴포넌트에는 style 속성이 있다.
<br>
이 style 속성에 인라인 스타일을 적용하는 방식과 스탙일시트(StyleSheet)에 정의된 스타일을 사용하는 방법이 있다.

### 인라인 스타일링
인라인 스타일은 HTML의 인라인 스타일링처럼 컴포넌트에 직접 스타일을 입력하는 방식을 말한다.
<br>
HTML과의 차이점은 HTML에서는 문자열 형태로 스타일을 입력하지만, 리액트 네이티브에서는 객체 형태로 전달해야 하는 차이점이 있다.
<br>
<br>
아래의 예제와 같이 객체의 형태로 전달한다.

```
// 

import { View, Text } from 'react-native';
import React from 'react';

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
            <Text
                style={{
                    padding: 10,
                    fontSize: 26,
                    fontWeight: '600',
                    color: 'black',
                }}
            >
                React Native Style
            </Text>
            <Text
                style={{
                    padding: 10,
                    fontSize: 26,
                    fontWeight: '400',
                    color: 'red',
                }}
            >
                Inline Style - Error
            </Text>
        </View>
    );
};

export default App;
```

![image](https://user-images.githubusercontent.com/87363461/227765400-c4473779-48b4-4af3-9be9-625e2c045e83.png)

<br>

이렇게 인라인 스타일링은 어떤 스타일이 적용되는지 잘 보인다는 장점이 있다.
<br>
하지만 단점으로 두 Text 컴포넌트처럼 비슷한 역할을 하는 컴포넌트에 동일한 코드가 반복된다는 점이다.
<br>
또 어떤 이유로 해당 스타일이 적용 되었는지 코드만으로는 명확하게 어렵다는 단점도 존재한다.

<br>

### 클래스 스타일링
클래스트 스타일링은 컴포넌트의 태그에 직접 입력하는 방식이 아닌 스타일시트에 정의된 스타일을 사용하는 방법이다.
<br>
스타일시트에 스타일을 정의하고 컴포넌트에서는 정의된 스타일의 이름으로 적용하는 방식으로, 웹 프로그래밍의 CSS 방식과 유사하다.
<br>

```
// App.js

import { View, Text, StyleSheet } from 'react-native';
import React from 'react';

const App = () => {
    return (
        <View style={styles.container}>
            <Text style={styles.text}>Text</Text>
            <Text style={styles.error}>Error</Text>
        </View>
    );
};

const styles = StyleSheet.create({
    container: {
        flex: 1,
        backgroundColor: '#fff',
        alignItems: 'center',
        justifyContent: 'center',
    },
    text: {
        padding: 10,
        fontSize: 26,
        fontWeight: '600',
        color: 'black',
    },
    error: {
        padding: 10,
        fontSize: 26,
        fontWeight: '400',
        color: 'red',
    },
});

export default App;
```

![image](https://user-images.githubusercontent.com/87363461/227766038-7fd06be0-02cd-4707-98be-99657590a6c3.png)

<br>

위의 예제를 보면 인라인 스타일을 적용했을 때보다 조금 더 깔끔해진 것을 볼 수 있다.
<br>
또한 전체적인 스타일 관리에도 클래스 스타일링이 더 쉽다.

<br>

### 여러 개의 스타일 적용
위 예제에서 text, error 등은 중복된 스타일이 많으며 두 스타일 모두 Text 컴포넌트에 적용되는 스타일이라는 공통점이 있다.
<br>
이렇게 중복된 코드를 제거하다 보면 스타일을 덮어쓰거나 하나의 컴포넌트에 여러 개의 스타일을 적용할 때가 있다.
<br>
이렇게 여러 개의 스타일을 적용할 경우는 배열을 이용하여 style 속성을 여러 개의 스타일로 지정하면 된다.

```
// App.js

import { View, Text, StyleSheet } from 'react-native';
import React from 'react';

const App = () => {
    return (
        <View style={styles.container}>
            <Text style={styles.text}>Text</Text>
            <Text style={[styles.text, styles.error]}>Error</Text>
        </View>
    );
};

const styles = StyleSheet.create({
    container: {
        flex: 1,
        backgroundColor: '#fff',
        alignItems: 'center',
        justifyContent: 'center',
    },
    text: {
        padding: 10,
        fontSize: 26,
        fontWeight: '600',
        color: 'black',
    },
    error: {
        fontWeight: '400',
        color: 'red',
    },
});

export default App;
```

![image](https://user-images.githubusercontent.com/87363461/227766038-7fd06be0-02cd-4707-98be-99657590a6c3.png)

<br>

error style을 보면 fontSize와 padding이 정의되어 있지 않은데, error Text에는 적용된 것을 볼 수 있다.
<br>
여기서 중요한 점은 스타일의 순서로, <b>뒤에 오는 스타일이 앞에 있는 스타일을 덮는다</b>는 것이다.
<br>
<br>
또한 아래와 같이 인라인 스타일과 클래스 스타일 방식을 혼용해서 사용할 수도 있다.

```
const App = () => {
    return (
        <View style={styles.container}>
            <Text style={[styles.text, { color: 'green' }]}>Text</Text>
            <Text style={[styles.text, styles.error]}>Error</Text>
        </View>
    );
};
```

### 외부 스타일 
외부 스타일은 외부 파일에 스타일링을 정의하고 이 스타일링을 여러 파일에서 공통으로 사용하는 것을 말한다.
<br>

아래 예제는 스타일 파일과 App.js 파일을 나눈 예제이며, 위 동작들과 같은 동작을 한다.

### [외부 스타일 예제 코드]()

<br>

### 실행 결과

<br>

![image](https://user-images.githubusercontent.com/87363461/227766796-ba2c7a0e-472c-4e25-8a54-ca498ded0794.png)
