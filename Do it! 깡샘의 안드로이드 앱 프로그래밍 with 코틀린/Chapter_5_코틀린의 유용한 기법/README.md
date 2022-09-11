# Chapter_5_코틀린의 유능한 기법 정리 내용

## 람다 함수
람다 함수는 많은 프로그래밍 언어에서 제공하는 익명 함수 정의 기법이다.
<br>
람다 함수는 주로 함수를 간단하게 정의할 때 이용하며, 람다식이라고도 한다.
<br>
<br>
고차 함수는 매개변수나 반환값으로 함수를 이용하는데, 람다 함수는 주고받을 함수를 간단하게 정의할 때 사용한다.
### 람다 함수 선언과 호출
람다 함수는 fun 키워드를 이용하지 않으며, 함수 이름이 없다.
<pre>
// 일반 함수 선언
fun 함수명 (매개변수) { 함수 본문 }

// 람다 함수 선언
{ 매개변수 -> 함수 본문 }
</pre>
코틀린에서 람다 함수 규칙은 3가지가 있다.
　　1. 람다 함수는 { }로 표현한다.
<br>
　　2. { }안에 화살표(->)가 있으며 화살표 왼쪽은 매개변수, 오른쪽은 함수 본문이다.
<br>
　　3. 함수의 반환값은 함수 본문의 마지막 표현식이다.
<br>
### 일반 함수와 람다 함수 비교
// 일반 함수
<pre>
fun sum(no1: Int, no2: Int): Int {
    return no1 + no2
}

// 람다 함수
val sum = {no1: Int, no2: Int -> no1 + no2}
</pre>
중괄호 안의 화살표를 기준으로 왼쪽에는 Int 타입의 매개변수를, 오른쪽에는 실행문이 존재한다.
<br>
람다 함수는 마지막 실행문의 결과가 함수의 반환값이므로 no1 + no2의 결과가 반환값이 된다.
<br>
<br>
그런데 일반 함수와 다르게 람다 함수는 이름이 없으므로 함수명으로 호출할 수 없다.
<br>
그래서 보통은 람다 함수를 변수에 대입하여 사용한다.
<br>
람다 함수를 호출할 때는 아래와 같이 호출한다.
<pre>
sum(10, 20)
</pre>
람다 함수를 꼭 변수에 대입해서 사용해야 하는 것은 아니다.
<br>
람다 함수를 선언하자마자 호출할 수도 있다.
<pre>
{no1: Int, no2: Int -> no1 + no2} (10, 20)
</pre>
### 매개변수 없는 람다 함수
함수에 매개변수가 항상 있어야 하는 것은 아니다.
<br>
매개변수가 없을 경우, 왼쪽 매개 변수를 비울 수도 있고, 매개변수가없을 때는 화살표까지 생략할 수 있다.
<pre>
// 매개변수가 없음
{-> println("function call")}
// 화살표까지 생략
{println("function call")}	
</pre>
### 매개변수가 1개인 람다 함수
<pre>
fun main() {
    // 2개 다 같은 결과
    val some = {no: Int -> println(no)}
    val some: (Int) -> Unit = {println(it)}
    some(10)
}

===== 실행 결과 =====
10
</pre>
람다 함수 앞에 (Int) -> Unit이 매개변수가 1개인 람다 함수임을 알려준다.
<br>
매개변수가 1개일 때는 중괄호 안에서 매개변수 선언을 생략하고, it 키워드로 매개변수를 이용할 수 있다.
<br>
여기서 중요한 점은, it을 이용해 매개변수를 사용하는 것은 매개변수의 타입을 식별할 수 있을 때만 가능하다.
### 람다 함수의 반환
람다 함수에서는 return 문을 사용할 수 없으므로 마지막 실행문이 결과값이 된다.
<pre>
fun main() {
    val some = {no1: Int, no2: Int ->
    	println("in lambda function")
    	no1 * no2	// 본문에서 마지막 실행 줄로 실행 결과
    }
    
    println("result : ${some(10, 20)}")
}

===== 실행 결과 =====
in lambda function
result : 200
</pre>
## 함수 타입과
함수 타입이란 함수를 선언할 때 나타내는 매개변수와 반환 타입을 의미한다.
<br>
변수에 함수를 대입하려면 변수를 함수 타입으로 선언해야 한다.
<br>
<pre>
// Int형 매개변수 2개를 받아서 Int 형으로 반환
fun some(no1: Int, no2: Int): Int {
    return no1 + no2
}
</pre>
이를 (Int, Int) -> Int로 표현할 수 있다.
<br>
함수를 대입할 변수를 선언할 때 이러한 함수 타입을 선언하고 그에 맞는 함수를 대입해야 한다.
<pre>     // 함수 타입                // 함수 내용
val some: (Int, Int) -> Int = { no1: Int, no2: Int -> no1 + no2}
</pre>
### typealias - 타입 별칭
typealias는 타입의 별칭을 선언하는 키워드로, 함수 타입을 typealias를 이용해 선언할 수 있다.
<br>
함수 타입뿐만 아니라 데이터 타입을 선언할 때도 사용한다.
<br>
하지만 타입 별칭은 주로 함수 타입을 선언하는 데 사용한다.
<pre>
typealias MyInt = Int    // 변수 타입 별칭
typealias MyFuncType = (Int, Int) > Booealean // 함수 타입 별칭
</pre>
람다 함수 정의 시 타입 별칭으로 매개 변수의 타입을 유추할 수 있다면 매개변수의 타입 선언을 생략할 수 있다.
<pre>
typealias MyFuncType = (Int, Int) > Booealean
val someFun: MyFunType = { no1, no2 -> no1 > no2 }
</pre>
## 고차 함수(High Order Function)
고차 함수란 함수를 매개변수로 전달받거나 반환하는 함수를 의미한다.
<br>
## 널 안정성
널이란 객체가 선언되었지만 초기화되지 않은 상태를 의미한다.
<pre>
val data1: String = "hello"
val data2: String? = null
</pre>
위 코드에서 data1 변수에는 "hello"라는 데이터를 저장했다.
<br>
실제로는 "hello"라는 무자열 데이터가 저장된 주소가 대입되며, 그 주소로 문자열 데이터를 이용한다.
<br>
<br>
data2 변수에는 null을 대입했고, 아직 주솟값을 가지지 못한다.
<br>
즉, 변수가 선언 되었지만 이용할 수 없는 상태이다.
<br>
<br>
이처럼 null 상태의 객체를 이용하면 <b>널 포인트 예외</b>가 발생한다.
<br>
널 포인트 예외는 널인 상태의 객체를 이용할 수 없다는 오류이다.
<br>
이때 널 안정성이란 널 포인트 예외가 발생하지 않도록 코드를 작성하는 것을 말한다.
<pre>
fun main() {
    var data: String? = null
    println("data length : ${data?.length?: 0}")
}

===== 실행 결과 =====
data length : 0
</pre>
data가 null이면 0을 반환하고, null이 아니면 length를 이용해 문자열의 개수를 얻는 예제이다.
<br>
null 점검 코드를 작성하지 않아도, 자동으로 널 안정성이 확보된다.
### 널 허용 - ? 연산자
널 허용 변수에는 타입 뒤에 물음표(?)를 붙여 선언한다. 널을 허용했기에 널을 대입할 수 있다.
<pre>
var data1: String = "Kkang"
data1 = null	// 오류!
var data2: String? = "kkang"
data2 = null	// 성공!
</pre>
### 널 안정성 호출 - ?. 연산자
널 허용으로 선언한 변수에는 널 포인트 예외가 발생할 수 있다. 따라서 널인 상황을 고려해야 한다.
<br>
이때 널 허용으로 선언한 변수에는 <b>반드시 ?. 연산자를 이용해야 한다.</b>
<br>
<br>
?. 연산자는 변수가 널이 아니면 멤버에 접근하지만, 널이면 멤버에 접근하지 않고 널을 반환한다.
### 엘비스 - ?: 연산자
엘비스 연산자란 ?: 기호를 말한다. 이 연산자는 변수가 널이면 반환한다.
<br>
변수가 널일 때 대입해야 하는 값이나 실행해야 하는 구문이 있을 때 사용한다.
<pre>
fun main() {
    var data: String? = "kkang"
    println("data = $data : ${data?.length ?: -1}")
    data = null
    println("data = $data : ${data?.length ?: -1}")
}

===== 실행 결과 =====
data = kkang : 5
data = null : -1
</pre>
### 예외 발생 - !! 연산자
!!는 객체가 널일 때 예외를 일으키는 연산자이다.
<pre>
fun some(data: String?): Int {
    return data!!.length  // 예외 발생
}

fun main() {
    println(some("kkang"))
    println(some(null))
}

===== 실행 결과 =====
5
Exception in thread "main" java.lang.NullPointerException
</pre>
