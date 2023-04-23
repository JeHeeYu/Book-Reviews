# 다트를 알면 플러터가 보인다 정리 내용

## 다트 언어의 9가지 특징
1. 다트는 main() 함수로 시작함
2. 다트는 어디에서나 변수를 선언하고 사용할 수 있음
3. 다트에서는 모든 변수가 객체이며 모든 객체는 Object 클래스를 상속받음
4. 다트는 자료형이 엄격한 언어이며, 다른 유형의 값을 저장하면 오류가 발생함. 만약 여러 자료형을 정의하려면 dynamic 타입 이용
5. 다트는 제네릭 타입을 이용하여 개발 가능. List<int> 처럼 int, List<dynamic> 등 다양한 데이터를 넣을 수 있음
6. 다트는 public, protected 같은 키워드가 없음. 외부로 노출하고 싶지 않다면 변수나 함수 이름 앞에 언더스코어(_)를 이용해 표시
7. 변수나 함수의 시작은 언더스코어(_) 또는 문자열로 시작하고 그 이후에 숫자 입력 가능
8. 삼항 연산자 사용 가능
9. Null Safety 지원

## 다트의 특징 이해하기
예를 들어 아래와 같이 다트 코드가 있다.

```
printIntegger(int aNumber) {
  print('The number is $aNumber.');
}

void main() {
  var number = 42;
  printIntegger(number);
}

// 실행 결과
The number is 42.
```

number라는 이름의 변수를 선언하고 숫자 42를 넣는다.
<br>
여기서 var 키워드는 자료형을 특정하지 않고 변수를 선언할 때 사용한다.
> var 키워드는 변수를 선언하면 해당 변수에 저장되는 값의 유형에 따라 자료형이 정해짐. 이것을 자료형 추론(Type Inference)라고 함
<br>
<br>
그리고 문자열을 표현할 때 따옴표를 사용하며, 따옴표 안에 ${표현식}과 같은 형태로 사용하면 표현식에 변수를 직접 넣을 수 있다.

## Null Safety
새롭게 추가된 Null Safety를 사용하려면 pubspec.yaml 파일의 SDK 환경을 변경해야 한다.

```
environment:
  sdk: ">=2.12.0 <3.0.0"
```

Null Safety는 변수를 선언할 때 사용하는 것으로, 자료형 다음에 ?를 붙이면 Null이 가능, 붙이지 않으면 부가능하다.
<br>
그리고 식 다음에 !를 붙이면 Null이 아님을 직접 표시한다.

```
// null을 넣을 수 있음
int? couldReturnNullButDoesnt() => -3;

void main() {
  // null로 변경 가능
  int? couldBeNullButIsnt = 1;
  
  // List의 int에 null값 포함 가능
  List<int?> listThatCouldHoldNulls = [2, null ,4];
  
  // List 자체가 null일 수 있음
  List<int>? nullsList;
  
  // null을 넣으면 오류
  int a = couldBeNullButIsnt;
  
  // int b는 ?가 없으므로 오류
  int b = listThatCouldHoldNulls.first;
  
  // null이 아님을 직접 표시
  int b = listThatCouldHoldNulls.first!;
  
  // null일 수도 있으므로 abs()에서 오류
  int c = couldReturnNullButDoesnt().abs();
  
  // null이 아님을 직접 표시
  int c = couldReturnNullButDoesnt()!.abs();
}

```

Null Safety를 사용하는 이유는 프로그램 실행 중 널 예외가 발생하면 프로그램이 중지되는데, 이를 코드 단계에서 구분하여 작성할 수 있도록 하기 위해서이다.

## 비동기(Asynchronous) 처리 방식
다트는 비동기 처리를 지원하는 언어이다.
<br>
여기서 비동기란 언제 끝날지 모르는 작업을 기다리지 않고 다음 작업을 처리하게 하는 것을 의미한다.
<br>
<br>
만약 비동기를 지원하지 않고 동기로만 처리한다면 어떤 작업이 오래 걸릴 경우 사용자는 실행이 멈춘 것으로 생각하고 프로그램을 종료할 수 있다.
<br>
<br>
일반적으로 네트워크에서 데이터를 가져오거나 데이터베이스 쓰기, 파일 읽기 등의 작업은 상황에 따라 언제 끝날지 알 수 없으므로 비동기로 처리해야 한다.

<br

아래 그림은 첫 번째는 동기 방식을 표현, 두 번째는 비동기 방식을 표현한 그림이다.

![image](https://user-images.githubusercontent.com/87363461/233819175-74d4410f-be19-4bd0-a1ca-1b74841e178e.png)

<br>

다트는 async와 await 키워드를 이용해 비동기 처리를 구현한다.
<br>
구현 방법은 아래와 같다.

1. 함수 이름 뒤, 본문이 시작하는 중괄호 { 앞에 async 키워드를 붙여 비동기로 만듦
2. 비동기 함수 안에서 언제 끝날지 모르는 작업 앞에 await 키워드를 붙임
3. 2번 작업을 마친 결과를 받기 위해 비동기 함수 이름 앞에 Feature(값이 여러 개면 Stream) 클래스를 지정

아래 코드는 비동기 처리를 구현한 코드이다.

```
void main() {
  checkVersion();
  print('end process');
}

Future checkVersion() async {
  var version = await lookUpVersion();
  print(version);
}

int lookUpVersion() {
  return 12;
}

// 실행 결과
end process
12
```

결과가 나온 이유는 먼저 checkVersion() 함수를 보면 이름 앞뒤로 Future와 async가 붙었다.
<br>
이렇게 하면 checkVersion 함수를 비동기 함수로 만들겠다는 의미이다.
<br>
<br>
즉 checkVersion() 함수 안에 await가 붙은 함수를 따로(비동기) 처리한 다음 그 결과는 Future 클래스에 저장해 둘 테니 먼저 checkVersion() 함수를 호출한 main() 함수의 나머지 코드를 모두 실행하라 라는 의미이다.
<br>
<br>
그리고 main 함수를 모두 실행했으면 그때 Future 클래스에 저장해 둔 결과를 이용해서 checkVersion() 함수의 나머지 코드를 실행한다.
<br>
<br>
위 코드에서는 lookUpVersion() 함수 앞에 await 키워드가 붙었다.
<br>
await 키워드는 처리를 완료하고 결과를 반환할 때까지 이후 코드의 처리를 멈춘다.
<br>
<br>
따라서 lookUpVersion() 함수를 호출해 version 변수에 12가 저장된 다음에야 비로소 print(version) 문으로 이를 출력한다.
<br>
<br>
이처럼 비동기 함수에서 어떤 결괏값이 필요하다면 해당 코드를 await으로 지정한다.
<br>
그러면 네트워크 지연 등으로 제대로 된 값을 반환받지 못한 채 이후 과정이 실행되는 것을 방지할 수 있다.

## 비동기 함수가 반환하는 값 활용하기
비동기 함수가 반환하는 값을 처리하려면 then() 함수를 이용한다.

```void main() async {
  await getVersionName().then((value) => {
    print(value)
  });
  print('end process');
}

Future<String> getVersionName() async {
  var versionName = await lookUpVersionName();
  return versionName;
}

String lookUpVersionName() {
  return 'Android Q';
}

// 실행 결과
Android Q
end process
```

위 코드를 보면 Future<String> 이라는 반환값을 정해 놓은 getVersionName()이라는 함수가 있다.
<br>
이 함수는 async 키워드가 붙었으므로 비동기 함수이다.
<br>
이처럼 비동기 함수가 데이터를 석옹적으로 반환하면 호출하는 쪽에서 then() 함수를 이용해 처리할 수 있다.
<br>
<br>
then() 이외에 error() 함수도 이용할 수 있는데, error() 함수는 실행 과정에서 오류가 발생했을 때 호출되므로 이를 이용해 예외를 처리할 수 있다.

## 다트와 스레드
다트는 하나의 스레드(Thread)로 동작하는 프로그래밍 언어이므로 await 키워드를 잘 사용해야 한다.
<br>
예를 들어 아래와 같은 코드가 있다.

```
void main() {
  printOne();
  printTwo();
  printThree();
}

void printOne() {
  print('One');
}

void printThree() {
  print('Three');
}

void printTwo() async {
  Future.delayed(Duration(seconds: 1), () {
    print('Future!');
  });
  print('Two');
}

// 실행 결과
One
Two
Three
Future!
```

One 출력 이후 printTwo() 함수에 진입하면 Future를 1초 지연했으므로 async로 정의한 비동기 함수의 특징에 따라 Two가 먼저 출력된다.
<br>
그리고 Three를 출력하고 Future가 가장 늦게 출력된다.
<br>
<br>
여기서 만약 printTwo() 함수를 아래와 같이 바꾸어 출력하면 다른 결과가 나온다.

```
void printTwo() async {
  await Future.delayed(Duration(seconds: 2), () {
    print('Future!');
  });
  print('Two');
}

// 실행 결과
One
Three
Future!
Two
```

Future.delayed() 코드 앞에 await 키워드가 붙었으므로, 2초를 기다린 후 Future를 먼저 출력하고 나중에 Two가 출력된다.
<br>
<br>
이처럼 await 키워드를 이용하면 await가 속한 함수를 호출한 쪽의 프로세스가 끝날 때까지 기다리기 때문에 이를 잘 고려해야 한다.

## JSON 주고받기
JSON을 사용하려면 코드에 covert라는 라이브러리를 포함해야 한다.

```
import 'dart:convert';

void main() {
  var jsonString = '''
    [
      {"score": 40},
      {"score: 80"}
    ]
  ''';
  
  var scores = jsonDecode(jsonString);
  
  // true 출력
  print(scores is List);
  
  var firstScore = scores[0];
  
  // true 출력
  print(firstScore is Map);
  
  // true 출력
  print(firstScore['score'] == 40);
}

```

위 코드에서 먼저 convert라는 라이브러리를 임포트 한다.
<br>
이후 jsonDecode() 함수에 전달한 후 그 결과를 scores 변수에 저장한다.
<br>
<br>
jsonDecode() 함수는 JSON 형태의 데이터를 dynamic 형식의 리스트로 변환해서 반환해 준다.
<br>
scores 변수가 리스트인지는 True False로 확인할 수 있다.

<br>
<br>
그리고 scores는 Map 형태이기 때문에 true, score의 value가 40이므로 모두 true를 나타낸다.
<br>
<br>
또한 서버로 보내는 형태도 있는데, 이때는 jsonEncode() 함수를 이용한다.

```
import 'dart:convert';

void main() {
  var jsonString = '''
    [
      {"score": 40},
      {"score: 80"}
    ]
  ''';
  
  var scores = jsonEncode(jsonString);  
}

```

## 스트림 통신하기

애플리케이션을 개발하다 보면 데이터를 순서대로 주고받아야 할 때가 있다.
<br>
데이터의 순서를 보장받고 싶을 때는 스트림(Stream)을 이용한다.
<br>
스트림은 처음에 넣은 데이터가 꺼낼 때도 가장 먼저 나오는 데이터 구조로 생각할 수 있다.

<br>
<br>
예를 들어 아래와 같은 코드가 있다.

```
import 'dart:async';

Future<int> sumStream(Stream<int> stream) async {
  var sum = 0;
  await for(var value in stream) {
    print('sumStream : $value');
    sum += value;
  }
  
  return sum;
}

Stream<int> countStream(int to) async* {
  for(int i = 1; i <= to; i++) {
    print('countStream : $i');
    yield i;
  }
}

void main() async {
  var stream = countStream(10);
  var sum = await sumStream(stream);
  
  // 55
  print(sum);
}


// 실행 결과
countStream : 1
sumStream : 1
countStream : 2
sumStream : 2
countStream : 3
sumStream : 3
countStream : 4
sumStream : 4
countStream : 5
sumStream : 5
countStream : 6
sumStream : 6
countStream : 7
sumStream : 7
countStream : 8
sumStream : 8
countStream : 9
sumStream : 9
countStream : 10
sumStream : 10
55
```

main() 함수를 보면 먼저 countStream(10) 함수를 호출한다.
<br>
이 함수는 async*와 yield 키워드를 이용해 비동기 함수로 만들었다.
<br>
함수 안에서 for문을 이용해 1부터 int형 매개변수 to로 전달받은 숫자까지 반복한다.
<br>
<br>
async* 명령어는 앞으로 yield를 이용해 지속적으로 데이터를 전달하겠다는 의미이다.
<br>
<br>
위 코드에서 yield는 int형 i를 반환하는데 return은 한 번 반환하면 함수가 끝나지만, yield는 반환 후에도 계속 함수를 유지한다.
<br>
<br>
이렇게 받은 yield 값을 인자로 sumStream() 함수를 호출하면 이 값이 전달될 때마다 sum 변수에 누적해서 반환해준다.
<br>
