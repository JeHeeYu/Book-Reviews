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
</pre>
