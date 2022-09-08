## Chapter_2_알아두어야 할 자바스크립트 정리 내용
## const, let의 차이
기존 자바스크립트에서 변수 선언 시 var로 변수를 선언했다면, ES2015+부터 let과 const가 대체한다.
<br>
const와 let의 공통점은 블록 스코프(범위)이다.
<pre>
if (true) {
  var x = 3;
}
console.log(x); // 3

if (true) {
  const y = 3;
}

console.log(y); // Uncaught ReferenceError: y is not defined
</pre>
var은 함수 스코프를 가지므로 if문의 블록과 관계없이 접근할 수 있다.
<br> 
반면 const와 let은 블록 스코프를 가지므로, 블록 밖에서는 변수에 접근할 수 없다.
<br>
함수 스코프 대신 블록 스코프를 사용하므로 호이스팅 같은 문제도 해결되고 코드 관리도 수월해졌다.
## 템플릿 문자열
ES2015에 새로 생긴 문자열 문법으로, 백틱(`)으로 감싸는 문법이다.
<br>
백틱은 문자열 안에 변수를 넣을 수 있는 문자열이다.
<pre>
// ES5 문법
var num1 = 1;
var num2 = 2;
var result = 3;

var string1 = num1 + '더하기' + num2 + '는 \'' + result + '\'';
console.log(string1); // 1더하기는 2는 '3'
</pre>
<pre>
// ES2015 문법
const num3 = 1;
const num4 = 2;
const reseult = 3;
const string2 = `${num3} 더하기 ${num4}는 '${result2}'`;
console.log(string2);   // 1 더하기 2는 '3'
</pre>
${변수} 형식을 사용하여 가독성이 올라가는 것을 볼 수 있다.
## 객체 리터럴
<pre>
var sayNode = function() {
  console.log('Node');
};
var es = 'ES';
var oldObject = {
  sayJS: function() {
    console.log('JS');
   },
   sayNode: sayNode,
};
oldObject[es + 6] = 'Fantastic';
oldObject.sayNode();  // Node
oldObject.sayJS();    // JS
console.log(oldObject.ES6); // Fantastic

// 위 코드를 아래와 같이 리터럴화 가능
const newObject = {
  sayJS() {
    console.log('JS');
  },
  sayNode,
  [es + 6]: 'Fantastic'
};
newObject.sayNode();    // Node
newObject.sayJS();      // JS
console.log(newObject.ES6);   // Fantastic
</pre>
oldObject와 newObject 비교 시 sayJS 같은 객체의 메서드에 함수를 연결할 때 더이상
<br>
콜론과 function을 붙이지 않아도 된다.
<br>
<pre>
{ name: name, age: age }
// 위를 아래와 같이 사용 가능
{ name, age}
</pre>
## 화살표 함수(Arrow Function)
화살표 함수는 기존 func()을 대체하는 문법으로, 기존 function()와 같은 동작을 한다.
<pre>
function add1(x, y) {
  return x + y;
}

const add2 = (x, y) => {
  return x + y;
}

const add3 = (x, y) => x + y;

const add4 = (x, y) => (x + y);

function not1(x) {
  return !x;
}

const not2 = x => !x;
</pre>
add1, add2, add3, add4는 모두 같은 기능을 하는 함수이다. 마찬가지로 not1과 not2도 같은 함수다.
<br>
화살표 함수 내부에 return 밖에 없는 경우는 return 문을 줄일 수 있다.
## 구조분해 할당
구조분해 할당을 사용하면 객체와 배열로부터 속성이나 요소를 쉽게 꺼낼 수 있다.
<pre>
// 객체의 속성을 같은 이름의 변수에 대입하는 코드

var candyMachine = {
  status: {
    name: 'node',
    count:5,
  },
  getCandy: function() {
    this.status.count--;
    
    return this.status.count;
  },
};
var getCandy = candyMachine.getCandy;
var count = candyMachine.status.count;

// 위 코드를 아래의 코드로 바꿀 수 있음

const candyMachine = {
  status: {
    name: 'node',
    count: 5,
   },
   getCadndy() {
    this.status.count--;
    
    return this.status.count;
  },
};
const { getCandy, status: { count } } = candyMachine;
</pre>
## 클래스
<pre>
var Human = function(type) {
  this.type = type || 'human';
};

Human.isHuman = function(human) {
  return human istanceof Human;
}

Human.prototype.breathe = function() {
  alert('h-a-a-a-m');
};

var Zero = function(type, firstName, lastName) {
  Human.apply(this, arguments);
  this.firstName = firstName;
  this.lastName = lastName;
};

Zero.prototype = Object.create(Human.prototype);
Zero.prototype.constructor = Zero; // 상속하는 부분
Zero.prototype.sayName = function() {
  alert(this.firstName + ' ' + this.lastName);
};

varOldZero = new Zero('human', 'Zero', 'Cho');
Human.isHuman(oldZero); // true

// 난해한 코드를 아래와 같이 클래스 기반 코드로 변경

class Human {
  constructor(type = 'human') {
  this.type = type;
  }
  
  static isHuman(human) {
    return human instanceof Human;
  }
  
  breathe() {
    alert('h-a-a-a-m');
  }
}

class Zero extends Human {
  constructor(type, firstName, lastName) {
    super(type);
    this.firstName = firstName;
    this.lastName = lastName;
  }
  
  sayName() {
    super.breathe();
    alert(`${this.firstName} ${this.lastName}`);
  }
}

const newZero = new Zero('human', 'Zero', 'Cho');
Human.isHuman(newZero);
</pre>
코드들이 전반적으로 class안에 그룹화 되었다.
<br>
생성자 함수는 contructor 안으로, Human.isHuman 같은 클래스 함수는 static 키워드로 전환되었다.
