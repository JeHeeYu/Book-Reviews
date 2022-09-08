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
