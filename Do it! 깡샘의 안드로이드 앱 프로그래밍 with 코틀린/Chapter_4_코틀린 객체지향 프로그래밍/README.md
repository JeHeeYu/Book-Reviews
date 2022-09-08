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
만약 클래스의 주 생성자를 선언하지 않으면 컴파일러가 매개변수가 없는 주 생성자를 자동으로 생성한다.
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
