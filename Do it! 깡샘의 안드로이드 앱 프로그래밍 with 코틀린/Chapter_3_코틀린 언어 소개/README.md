# Chapter_3_코틀린 언어 소개 정리 내용
## 코틀린의 역사
코틀린은 젯브레인스(JetBrains)에서 오픈소스 그룹을 만들어 개발한 프로그래밍 언어이다.
<br>
코틀린은 2011년 처음 공개되었으며 2017년 구글에서 안드로이드 공식 언어로 지정했다.
<br>
코틀린도 안드로이드 앱 개발할 수 있는 자바와 같이 JVM 머신에 기반을 둔 언어이다.
<br>
코틀린은 러시아 섬 이름애서 유래되었다.
## 코틀린의 확장자
자바는 '.java' 확장자를 이용하듯 코틀린은 '.kt' 확장자를 이용한다.
<br>
코틀린 컴파일러가 .kt 파일을 컴파일하면, 자바 바이트 코드가 만들어진다.
<br>
## 코틀린의 장점
### 표현력과 간결함(Expressive And Concise)
코틀린은 최신 언어 기법을 이용한 만큼, 다른 언어에 비해 훨씬 간결한 구문으로 프로그램을 작성할 수 있다.
<br>
같은 로직으로 자바와 비교 시 코틀린이 훨씬 간결하다.
### 안전한 코드(Safer Code)
코틀린은 널 안정성(NULL Safety)를 지원하는 언어이다.
<br>
코틀린은 다른 언어에서 발생하는 런타임 오류를 런타임 오류를 사전에 잡을 수 있다.
<br>
<br>
변수에 널 허용(NULLable)과 널 불허용(Not NULL)을 이용한다.
<br>
NULL과 관련된 오류를 사전에 잡을 수 있기 때문에, 보다 안전한 코드를 작성할 수 있다.
### 상호 운용성(Interoperable)
코틀린과 자바는 100% 호환된다. 따라서 코틀린으로 프로그램을 작성할 때 자바 클래스나 라이브러리를 이용할 수 있다.
<br>
뿐만 아니라 하나의 앱을 개발할 때, 자바와 코틀린을 혼용해서 사용할 수 있다.
### 구조화 동시성(Structured Concurrency)
코틀린 언어가 제공하는 코루틴(Coroutines)이라는 기법을 이용하면 비동기 프로그래밍을 간소화할 수 있다.
<br>
네트워크 연동이나 데이터베이스 갱신과 같은 작업 시 프로그램을 간단하고 효율적으로 작성할 수 있다.
## 코틀린의 파일 구성
코틀린은 '.kt'  확장자를 가지며, package와 import 구문이 있다.
<pre>
package com.example.test3
</pre>
package 구문은 이 파일을 컴파일했을 때 만들어지는 클래스 파일의 위치를 나타낸다.
<br>
package 구문은 맨 처음 한 줄로 선언되며, 이 경우 com/example/test3 폴더에 생성된다.
<br>
<br>
package 이름은 kt 파일의 위치와 상관없는 별도의 이름으로도 선언할 수 있다.
<br>
예를 들어, User.kt 파일이 com/example/test3에 있더라도, package ch3 처럼 선언할 수 있다.
<br>
물론 이렇게 선언했을 때 컴파일된 클래스 파일은 ch3 폴더에 생성된다.
<br>
<pre>
import java.text.SimpleDateFormat
import java.util.*
</pre>
import 구문은 package 구문 아래에 여러 줄 작성할 수 있다.
<br>
그리고 어떤 파일에 선언한 멤버(변수, 함수, 클래스)를 다른 코틀린 파일에서 참조할 때
<br>
두 파일을 같은 package로 선언했다면 import 구문 없이 사용할 수 있다.
## 코틀린의 변수
코틀린에서 변수는 <b>val</b> 키워드와 <b>var</b> 키워드를 사용해 선언한다.
<br>
val은 value의 줄임말로, <b>초깃값이 할당되면 바꿀 수 없는 변수</b>를 선언할 때 사용한다. (const 키워드)
<br>
var은 variable의 줄임말로, <b>초깃값이 할당된 후에도 바꿀 수 있는 변수</b>를 선언할 때 사용한다.
<pre>
val data1 = 10
var data2 = 10

fun main() {
  data = 20   // 오류!
  data = 20   // 성공!
}
</pre>
### 타입 지정과 타입 추론
코틀린에서 변수명 뒤에 콜론(:)을 추가해 타입을 명시할 수 있다.
<br>
대입하는 값에 따라 타입을 유추(타입 추론)할 수 있을 때는 생략할 수 있다.
<pre>
val data1: Int = 10   // 명시적으로 Int 타입 선언
val data2 = 10        // 할당 값이 Int 타입이므로 자동으로 Int 타입 설정
</pre>
### 초깃값 할당
코틀린에서 최상위에 선언한 변수나 클래스의 멤버 변수는 선언과 동시에 초깃값을 할당해야 한다.
<br>
함수 내부에서 선언한 변수는 선언과 동시에 초깃값을 할당하지 않아도 된다.
<br>
대신 변수를 이용하려고 하면 값을 할당하고 이용해야 한다.
<pre>
val data1: Int      // 오류!
val data2 = 10      // 성공!

fun someFun() {
  val data3: Int
  println("data3 : $data3")     // 오류!
  data3 = 10
  println("data3 : $data3")     // 성공!
}

class User {
  val data4: Int        // 오류!
  val data5: Int = 10   // 성공!

}
</pre>
### 초기화 미루기
변수를 선언하고 초깃값을 할당할 수 없는 경우에 사용하며, <b>lateinit</b> 키워드와 <b>lazy</b> 키워드를 사용한다.
lateinit 키워드는 <b>초깃값을 할당할 것임을 명시적으로 선언</b> 할 때 사용한다.
<pre>
lateinit var data1: Int      // 오류!
lateinit val data2: String   // 오류!
lateinit var data3: String   // 성공!
</pre>
lateinit 키워드로 선언한 변수는 동시에 초깃값을 할당하지 않아도 되지만, 2가지의 규칙을 지켜야 한다.
<br>
　　1. lateinit은 <b>var 키워드로 선언한 변수</b>에만 사용할 수 있다.
<br>
　　2. Int, Long, Short, Double, Float, Boolean, Byte 타입에는 사용할 수 없다.
<br>
<br>
lazy 키워드는 by lazy { } 형식으로 선언한다.
<br>
소스에서 변수가 최초로 이용되는 순간 중괄호로 묶은 부분이 자동으로 실행되어, 그 결괏값이 변수의 초깃값으로 할당된다.
<br>
lazy 문의 중괄호 부분을 여러 줄로 작성하면 마지막 줄의 실행 결과가 변수의 초깃값이 된다.
<pre>
val data4: Int by lazy {
  println("in lazy...")
  10
}

fun main() {
  println("in main...")
  println(data4 + 10)
  println(data4 + 10)
}

===== 실행 결과 =====
in main...
in lazy...
20
20
</pre>
## 데이터 타입
<b>코틀린의 모든 변수는 객체</b>이다. 따라서 코틀린의 모든 타입은 <b>객체 타입</b>이다.
<br>
예를 들어, 정수 타입인 Int 타입은 기초 데이터 타입이 아닌 <b>클래스</b>이다.
<pre>
fun someFun() {
  var data1: Int = 10
  var data2: Int? = null    // null 대입 가능
  
  data1 = data + 10
  data1 = data1.plus(10)    // 객체의 메서드 이용 가능
}
</pre>
만약 Int 타입이 기초 데이터 타입이고, 클래스가 아니었다면 null을 대입할 수 없을 뿐만 아니라, 메소드를 이용할 수 없을 것이다.
### 코틀린의 문자와 문자열
Char는 코틀린에서 문자를 표현하는 타입으로, 작은따옴표(')로 감싸서 표현하고, Number 타입으로 표현할 수 없다.
<pre>
val a: Char = 'a'   // 성공!
if(a == 1)          // 오류!
</pre>
문자열은 String 타입으로 표현하고, 큰따옴표("")나 삼중 따옴표(""")로 감싸서 표현한다.
<br>
삼중 따옴표는 키보드로 입력한 줄 바꿈이나 들여쓰기가 그대로 반영된다.(삼중 따옴표를 안쓸 거면 역슬래시(\)나 이스케이프 시퀀스 사용)
<br>
<br>
코틀린에는 문자열 템플릿(String Template)가 있다.
<br>
문자열 템플릿이란 String 타입의 데이터에 변숫값이나 어떤 연산식의 결괏값을 포함하는 것을 말하며, $ 기호를 이용한다.
<pre>
fun main() {
  fun sum(no: Int): Int {
    var sum = 0
    for (i in 1..no) {
      sum += i
  }
  
  val name: String = "kkang"
  println("name : $name, sum : ${sum(10)}, plus : ${10 + 20}")
}

===== 실행 결과 =====
name : kkang, sum : 55, plus : 30
</pre>
### Any (모든 타입 가능)
Any는 코틀린에서 최상위 클래스이다. 모든 코틀린의 클래스는 Any의 하위 클래스이다.
<br>
따라서 Any 타입으로 선언한 변수에는 모든 타입의 데이터를 할당할 수 있다.
<pre>
val data1: Any = 10
val data2: Any = "hello"

class User
val data3: Any = User()   // 성공!
</pre>
### Unit - 반환문이 없는 함수
Unit은 다른 타입과 다르게 데이터의 형식이 아닌, 특수환 상황을 표현하려는 목적으로 사용한다.
<br>
Unit 타입으로 선언한 변수에는 Unit 객체만 대입할 수 있다. 따라서 Unit 타입으로 변수는 선언할 수 있지만 의미가 없다
<br>
<br>
Unit은 주로 함수의 반환 타입으로 사용한다.
<br>
함수에서 반환문이 없을 경우 명시적으로 나타낼 때 Unit을 사용한다.
<br>
함수를 선언할 때 반환 타입을 생략하면 자동으로 Unit이 적용된다.
<pre>
val data1: Unit = Unit

fun some(): Unit {
  println(10 + 20)
}


fun some() {
  println(10 + 20)      // Unit 타입이 자동으로 적용
}
</pre>
### Nothing - null이나 예외를 반환하는 함수
Nothing도 특수한 상황을 표현하는 타입이다.
Nothing으로 선언한 변수에는 null만 데이터를 댕비할 수 있고, 변수로서의 데이터는 의미가 없다.
<br>
<br>
Nothing도 Unit과 마찬가지로 함수 반환 타입에 사용된다.
<br>
Nothing은 반환 값이 null 또는 예외만 반환할 수 있다.
<pre>
val data1: Nothing? = null

fun some1(): Nothing? {
  return null
}

fun some2(): Nothing {
  throw Exception()
}
</pre>
### 널 허용과 불허용
코틀린의 모든 타입은 객체이므로 변수에 null을 대입하여 사용할 수 있다. 
<br
코틀린에서 null을 대입할 수 있는 변수(널 허용, nullable)인지, 대입할 수 없는(널 불허용, not null)인지 명확하게 구분해서 선언해야 한다.
<br>
<br>
null 구분 변수는 타입 뒤에 물음표(?)로 표시한다.
<br>
<b>타입 뒤에 물음표를 추가하면 널 허용</b> <b>타입 뒤에 물음표를 추가하지 않으면 널 불허용</b>이다.
<pre>
var data1: Int = 10
data1 = null        // 오류!

var data2: Int? = 10
data2 = null        // 성공!
</pre>
