# Chapter_3_코틀린 객체지향 프로그래밍 내용 정리
## 클래스 선언
코틀린에서 클래스는 class 키워드를 사용해 선언한다.
<pre>
class User { }
</pre>
클래스의 멤버는 생성자, 변수, 함수, 클래스로 구성된다.
이 중에서 생성자는 constructor 키워드를 사용해 선언한다. 그리고 클래스 안에 다른 클래스를 선언할 수 있다.
<pre>
fun main() {
	class User {
        var name = "kkang"

        constructor(name: String) {       // 생성자
            this.name = name
        }

        fun someFun() {
            println("name : $name")
        }
        
        class SomeClass { ... }
    }
    
    var user = User("Kim")    // 생성자
    user.someFun()            // 클래스 메소드 
}

===== 실행 결과 =====
name : Kim
</pre>
클래스의 <b>생성자는 주 생성자와 보조 생성자로 구분</b>한다.
<br>
한 클래스 안에 주 생성자만 선언할 수도 있고, 보조 생성자만 선언할 수도 있고, 둘 다 선언할 수도 있다.
### 주 생성자
주 생성자는 constructor 키워드로 클래스 선언부에 선언한다.
<br>
주 생성자는 필수는 아니며, 한 클래스에 하나만 선언 가능하다.
<br>
또한 주 생성자를 선언할 때 constructor 키워드를 생략할 수 있다.
<pre>
class User constructor() { }   // 주 생성자
class User() { } // 키워드 생략 가능
</pre>
만약 클래스의 주 생성자를 선언하지 않으면 컴파일러가 
가 없는 주 생성자를 자동으로 생성한다.
### 주 생성자의 매개변수
주 생성자를 선언할 때 필요에 따라 매개변수를 선언할 수도 있다.
<br>
객체 생성 시 매개변수의 타입과 개수에 맞는 인자를 전달해야 한다.
<pre>
class User constructor(name: String, count: Int) { }   // 주 생성자
class User(name: String, count: Int) { } // 키워드 생략 가능

val user = User("kkang", 10)    // 객체 생성
</pre>
### 주 생성자의 본문 - init 영역
주 생성자를 이용해 객체를 생성할 때 특정한 로직을 수행할 수 있다.
<br>
대신 주 생성자의 실행 영역을 벗어나면 오류가 발생한다.
<pre>
class User(name: String, count: Int) {
      // 주생성자 본문
} {   // 클래스 본문으로 오류 발생
}
</pre>
주 생성자 클래스 영역에는 { } 를 추가할 수 없다. 이유는 주 생성자는 클래스 선언부에 있기 때문이다.
<br>
이럴 때는 init 키워드를 사용해 주 생성자의 본문을 구현할 수 있다.
<br>
<br>
코틀린의 클래스 안에서 init 키워드로 지정한 영역은 객체를 생성할 때 자동으로 실행된다.
<br>
클래스에서 init 영역은 꼭 선언할 필요는 없으므로, 주 생성자 본문을 구현하고 싶을 때 사용한다.
<br>
init 영역은 보조 생성자로 객체를 생성할 때 실행될 수도 있지만, 보조 생성자는 클래스 안에 선언하므로, 
<br>
{ } 를 본문을 이용해 지정할 수 있다.
<br>
<br>
따라서 init 영역은 일반적으로 <b>주 생성자의 본문을 구현하는 용도</b>로 사용한다.
<pre>
class User(name: String, count: Int) {
    init {
        println("init area")
    }
}

fun main() {
    val user = User("kkang", 10)
}

===== 실행 결과 =====
init area
</pre>
### 생성자의 매개변수를 클래스의 멤버 변수로 선언하는 방법
아래 오류 init과 같이 매개변수는 지역 변수이므로, 다른 함수에서 사용할 수 없다.
<br>
someFun()과 같이 다른 영역에서 선언할 경우, 멤버 변수에 대입후 사용해야 한다.
<pre>
class User(name: String, count: Int) {
    var name: String
    var count: Int   
    
    init {      // 오류 발생
        println("name : $name, count : $count")
    }
    
    init {      // 정상!
        this.name = name
        this.count = count
    }
    
    fun someFun() {
        println("name : $name, count : $count")
    }
}

fun main() {
    val user = User("kkang", 10)
    user.someFun()
}
</pre>
위 방법도 문법적으로 가능하지만, 더 간결하게 지정할 수 있는 방법이 있다.
<pre>
class User(val name: String, val count: Int) {
    fun someFun() {
        println("name : $name, count : $count")
    }
}

fun main() {
    val user = User("kkang", 10)
    user.someFun()
}

===== 실행 결과 =====
name : kkang, count : 10
</pre>
위와 같은 방법으로, 매개변수에 val 또는 var 키워드로 선언하면 클래스의 멤버 변수가 된다.
<br>
원래 함수 매개변수를 선언할 때 val, var 키워드를 사용할 수 없으나,
<br>
주 생성자에서만 유일하게 키워드를 사용하여 매개변수를 선언할 수 있다.
## 보조 생성자
보조 생성자는 클래스의 본문에 constructor 키워드를 사용해 선언하는 함수이다.
<br>
클래스 본문에 선언하므로, 여러 개를 추가할 수 있다.
<br>
<br>
보조 생성자도 생성자이므로 객체를 생성할 때 자동으로 호출된다.
<br>
보조 생성자는 클래스 본문에 선언하므로 중괄호로 묶어서 객체 생성과 동시에 실행할 영역을 지정할 수 있다.
<pre>
class User {
    constructor(name: String) {
        println("constructor(name: String) call...")
    }
    constructor(name: String, count: Int) {
        println("constructor(name: String, count: Int) call...")
	}
}

fun main() {
	val user1 = User("kkang")
    val user2 = User("kkang", 10)
}

===== 실행 결과 =====
constructor(name: String) call...
constructor(name: String, count: Int) call...
</pre>
### 보조 생성자에 주 생성자 연결
클래스를 선언할 때 둘 중 하나만 선언하면 문제가 없지만, 만약 모두 생성한다면,
<br>
주 생성자와 보조 생성자 끼리 반드시 연결해 주어야 한다.
<br>
아래 예는 클래스에 주 생성자와 보조 생성자를 모두 선언한 것으로, 오류가 발생한다.
<pre>
class User(name: String) {		// 주 생성자
    constructor(name: String, count: Int) {	// 보조 생성자, 오류 발생!
        println("constructor(name: String, count: Int) call...")
    }
}
</pre>
주 생성자가 없다면 문제가 생기지 않겠지만, 주 생성자가 있을 경우 <b>보조 생성자에서 주 생성자를 호출</b>해 주어야 한다.
<br>
보조 생성자는 객체를 호출할 때 호출되며, 이때 주 생성자가 있다면 <b>this() 구문으로 주 생성자를 호출해 주어야 한다.</b>
<pre>
class User(name: String) {		// 주 생성자
    constructor(name: String, count: Int): this(name) {	// 보조 생성자, 성공!
        println("constructor(name: String, count: Int) call...")
    }
}
</pre>
만약 주 생성자가 있는 경우에서 보조 생성자가 여러 개 있다면 모두 this()를 호출해 주어야 한다.
## 클래스 상속과 생성자
클래스를 선언할 때 다른 클래스를 참조해서 선언하는 것을 상속(Inhaeritance)라고 한다.
<br>
코틀린에서 어떤 클래스를 상속 받으려면 선언부에 콜론(:)과 함께 상속받을 클래스 이름을 입력한다.
<br>
또한 다른 클래스가 상속할 수 있게 선언하려면 open 키워드를 이용해 선언한다.
<pre>
open class Super { }	// 상속할 수 있게 open 키워드 이용
class Sub: Super() { }	// Super 클래스를 상속받아 Sub 클래스 선언
</pre>
상위 클래스를 상속 받은 하위 클래스의 생성자에서는 상위 클래스의 생성자를 호출해야 한다.
만약 매개변수가 있는 상위 클래스의 생성자를 호출할 때 매개변수 구성에 맞게 인자를 전달해야 한다.
<pre>
open class Super(name: String) { }	  // 상위 클래스의 생성자 name
class Sub(name: String): Super(name) { }  // 하위 클래스도 동일한 생성자
</pre>
상위 클래스의 생성자 호출문을 꼭 클래스 선언부에 작성할 필요는 없다.
<br>
만약 하위 클래스에 보조 생성자만 있다면 상위 클래스의 생성자를 아래와 같이 호출할 수 있다.

<pre>
open class Super(name: String) { }	  // 상위 클래스의 생성자 name
class Sub: Super { 
    constructor(name: String): super(name) // 하위 클래스도 동일한 생성자
}  
</pre>
### 오버라이딩 - 재정의
상속이 주는 최고의 이점은 상위 클래스에 정의된 멤버(변수, 함수)를 하위 클래스에서 자신의 멤버처럼 사용할 수 있는 것이다.
<pre>
open class Super {
    var superData = 10
    fun superFun() {
        println("i am superFun : $superData")
    }
}

class Sub: Super()

fun main() {
    val obj = Sub()
    obj.superData = 20
    obj.superFun()
}

===== 실행 결과 =====
i am superFun : 20
</pre>
Super 클래스를 상속받은 Sub 클래스의 객체가 superData()와 superFun()을 사용했다.
상위 클래스의 superData()와 같이 하위 클래스에서 다시 선언하여 재정의 하는 것을 오버라이딩(Overriding) 이라고 한다.
<br>
<br>
변수도 오버라이딩 기법으로 재정의할 수 있지만, 주로 함수를 재정의하는 데 사용한다.
<br>
함수는 본문의 실행 영역에 특정한 로직을 작성하는데, 하위 클래스에서 같은 함수명으로 새 로직을 추가할 때 사용한다.
<br>
<br>
오버 라이딩 규칙으로, <b>오버라이딩을 허용할 변수나 함수 선언 앞에 open 키워드를,</b>
<br>
<b>하위 클래스에서 재정의할 때 앞에 override 키워드를</b> 추가해야 한다.
<pre>
open class Super {
    open var superData = 10	// 오버라이딩 허용 변수
    open fun superFun() {	// 오버라이딩 허용 함수
        println("i am superFun : $superData")
    }
}

class Sub: Super() {
    override var superData = 20	  // 재정의 변수
    override fun superFun() {	  // 
        println("i am superFun : $superData")
    }
}

fun main() {
    val obj = Sub()
    obj.superFun()
}

===== 실행 결과 =====
i am superFun : 20
</pre>
## 접근 제한자(Visibility Modifier)
접근 제한자란 <b>클래스의 멤버를 외부의 어느 범위까지 이용하게 할 것</b>인지를 결정하는 키워드이다.
<br>
코틀린에서 제공하는 접근 제한자는 4가지로, <b>public, internal, protected, private</b>가 있다.
<br>
<img src="https://user-images.githubusercontent.com/87363461/189154568-34dff39c-a001-401e-a33a-183343199af8.JPG" width="600" height="200">
<br>
<b>public</b> : 접근 제한이 없음을 나타낸다.
<br>
즉, 원하는 곳 어디서든 접근 가능하며, 접근 제한자를 생략 시 public이 기본 적용된다.
<br>
<br>
<b>internal</b> : 같은 모듈 내에서 접근할 수 있다.
<br>
모듈은 그래들(Gradle), 메이븐(Maven)과 같은 빌드 도구에서 프로젝트 단위 또는 세트 단위를 가리킨다.
<br>
<br>
<b>protected</b> : 클래스의 멤버에서만 선언할 수 있고, 최상위에 선언되는 변수나 함수는 protected로 선언할 수 없다.
<br>
protected로 선언한 클래스의 멤버는 해당 클래스 내부와 그 클래스를 상속받는 하위 클래스에서 접근할 수 있다.
<br>
<br>
<b>private</b> : 클래스에서 이용할 때는 해당 클래스 내부에서만 접근할 수 있으며,
<br>
최상위에서 private로 선언하면 해당 파일 내부에서만 접근할 수 있다.
<pre>
open class Super {
    var publicData = 10
    protected var protectedData = 20
    private privateData = 30
}

class Sub: Super() {
    fun subFun() {
        publicData++		// 성공!
        protectedData++		// 성공!
        privateData++		// 오류!
    }
}

fun main() {
    var obj = Super()
    obj.publicData++		// 성공!
    protectedData++			// 오류!
    privateData++			// 오류!
}
</pre>
## 클래스의 종류
### 데이터 클래스
데이터 클래스는 data 키워드로 선언하며, 자주 사용하는 데이터를 객체로 묶어 준다.
<br>
데이터 클래스는 VO(Value-Object) 클래스를 편하게 이용할 수 있게 해준다.
<pre>
class NonDataClass(val name: String, val email: String, val age: Int) { }	// data 키워드가 없음
data class DataClass(val name: String, val email: String, val age: Int) { }	// data 키워드가 있음
</pre>

### equals()
equals() 함수는 객체의 데이터를 비교하는 함수이다.
<br>
VO 클래스는 데이터를 주요하게 다루므로 객체의 데이터가 서로 같은지 비교할 때가 많다.
<br>
이때 equals() 함수를 사용한다.
<pre>
class NonDataClass(val name: String, val email: String, val age: Int) { }
data class DataClass(val name: String, val email: String, val age: Int) { }

fun main() {
    val non1 = NonDataClass("kkang", "a@a.com", 10)
    val non2 = NonDataClass("kkang", "a@a.com", 10)
    
    val data1 = DataClass("kkang", "a@a.com", 10)
    val data2 = DataClass("kkang", "a@a.com", 10)
    
    println("non data class equals : ${non1.equals(non2)}")
    println("data class equals : ${data1.equals(data2)}")
}

===== 실행 결과 =====
non data class equals : false
data class equals : true
</pre>
equals() 함수로 일반 클래스의 객체를 비교하면 객체 자체를 비교하므로 결과는 false이다.
<br>
하지만 데이터 클래스의 객체를 비교하면 객체 자체가 아닌, 객체의 데이터를 비교하므로 true이다.
<br>
<br>
데이터 클래스는 데이터를 다루는 데 편리한 기능을 제공하는 것이 주 목적이므로, 
<br>
주 생성자에 val, var 키워드로 매개변수를 선언해 클래스의 멤버 변수로 활용하는 것이 일반적이다.
<br>
이때 equals() 함수는 주 생성자에 선언한 멤버 변수의 데이터만 비교 대상으로 삼는다.
<pre>
data class DataClass(val name: String, val email: String, val age: Int) {
    lateinit var address: String
    constructor(name: String, email: String, age: Int, address: String): this(name, email, age) {
        this.address = address
    }
}

fun main() {
	val obj1 = DataClass("kkang", "a@a.com", 10, "Seoul")
    val obj2 = DataClass("kkang", "a@a.com", 10, "Busan")
    
    println("obj1.equals(obj2) : ${obj1.equals(obj2)}")
}

===== 실행 결과 =====
obj1.equals(obj2) : true  // 매개 변수가 아닌 address를 비교했으므로 true 반환
</pre>
### toString()
toString() 함수는 객체의 데이터를 반환하는 함수이다.
<br>
데이터 클래스를 사용하면서 객체가 가지는 값을 확인해야할 때가 많은데,
<br>
이때 데이터 클래스와 일반 클래스의 toString() 함수의 반환값이 다르다.
<pre>
fun main() {
    class NonDataClass(val name: String, val email: String, val age: Int)
	data class DataClass(val name: String, val email: String, val age: Int)
    
    val non = NonDataClass("kkang", "a@a.com", 10)
    val data = DataClass("kkang", "a@a.com", 10)
    
    println("non data class toString : ${non.toString()}")
    println("data class toString : ${data.toString()}")
}

===== 실행 결과 =====
non data class toString : FileKt$main$NonDataClass@49c2faae
data class toString : DataClass(name=kkang, email=a@a.com, age=10)
</pre>
일반 클래스로 생성한 객체의 toString() 함수는 의미 있는 값이 아니다.
<br>
반면 데이터 클래스의 경우 <b>객체가 포함하는 멤버 변수의 데이터를 출력한다.</b>
<br>
따라서 객체의 데이터를 확인할 때 유용하게 사용할 수 있다. 하지만, 주 생성자의 매개변수에만 선언된 데이터만 출력 대상이다.
## 오브젝트 클래스
코틀린에서 오브젝트는 익명 클래스(Anonymous Class)를 만들 목적으로 사용한다. 
<br>
익명 클래스는 말 그대로 이름이 없는 클래스이다.
<br>
클래스의 이름이 없으므로 클래스를 선언과 동시에 객체를 생성해야 하며, 그렇지 않을 시 객체를 생성할 수 없다.
<br>
<br>
오브젝트 클래스는 선언과 동시에 객체를 생성한다는 의미로 object 키워드를 사용한다.
<pre>
val obj = object {
    var data = 10
    fun some() {
        println("data : $data")
    }
}

fun main() {
    obj.data = 20	// 오류!
    obj.some()		// 오류!
}
</pre>
여기서 오류가 발생하는 이유는 클래스의 타입 때문이다. object 키워드로 클래스를 선언했지만, 타입을 명시하지 않았으므로
<br>
최상위 타입인 Any로 취급되는데, Any 클래스에 data, some()이 없어서 오류가 발생한다.
<br>
그래서 object { } 형태로 익명 클래스를 선언할 수는 있지만 보통은 타입까지 함께 입력해서 선언한다.
<br>
<br>
오브젝트 클래스의 타입은 object 뒤에 콜론을 입력하고 그 뒤에 클래스의 상위 또는 인터페이스를 입력한다.
<pre>
open class Super {
    open var data = 10
    open fun some() {
        println("i am super some() : $data")
    }
}

val obj = object: Super() {
    override var data = 20
    override fun some() {
        println("i am object some() : $data")
    }
}

fun main() {
    obj.data = 30	// 성공!
    obj.some()		// 성공!
}

===== 실행 결과 =====
i am object some() : 30
</pre>

## 컴패니언 클래스
컴패니언 클래스는 멤버 변수나 함수를 클래스 이름으로 접근하고자 할 때 사용한다.
<br>
일반적으로 클래스의 멤버는 객체를 생성해서 접근하는데,  컴패니언 클래스는 객체를 생성하지 않더라도,
<br>
클래스 이름으로 특정 멤버를 이용할 수 있다.
<br>
<br>
클래스의 이름으로 접근할 수 있게 companion 이라는 키워드로 선언해야 한다.
<pre>
class MyClass {
    companion object {	// 컴패니언 클래스
    var data = 10
    fun some() {
        println(data)
    	}
    }
}

fun main() {
    MyClass.data = 20	// 오류!
    MyClass.some()		// 오류!
}
</pre>
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
