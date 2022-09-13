# Chapter_3_노드 기능 정리 내용
## 노드 모듈 만들기
<b>모듈이란 특정한 기능을 하는 함수나 변수들의 집합</b>이다.
<br>
보통 파일 하나가 모듈 하나가 되고, 파일별로 코드를 모듈화할 수 있어 관리하기 편리하다.
<pre>
<code>
// var.js
const odd = '홀수';
const even = '짝수'
module.exports = {
  odd, 
  even,
};
</code>
</pre>
var.js에서 변수 두 개를 선언하고, mdoule.exports에 변수들을 담은 객체를 대입한다.
<br>
이제 이렇게 한 파일은 모듈로서 기능을 사용할 수 있다.
<br>
다른 파일에서 module.exports에 대입된 값을 사용할 수 있는 것이다.
<pre>
<code>
// func.js
const { odd, even } = require('./var');
function checkOddOrEven(num) {
  if (num %2) { // 홀수일 경우
    return odd;
  }
  else {
    return even; // 짝수일 경우
  }
}

module.exports = checkOddOrEven;
</code>
</pre>
require 함수 안에 불러온 모듈의 경로를 적고 js나 json 같은 확장자는 생략한다.
<br>
var.js에 있는 변수를 불러온 뒤, 숫자의 홀짝을 판별하는 함수를 func.js에 선언했다.
<br>
그리고 func.js에서 다시 모듈화를 하여 다른 프로그램에서 사용할 수 있다.
<pre>
<code>
// index.js

const { odd, even } = require('./var');
const checkNumber = require('./func');

function checkStringOddOrEven(str) {
    if(str.length % 2) {  // 홀수일 경우
        return add;
    }
    else {
        return even;    // 짝수일 경우
    }
}

console.log(checkNumber(10));
console.log(checkStringOddOrEven('hello');

===== 실행 결과 =====
짝수
홀수
</code>
</pre>
index.js에서 var.js와 func.js를 모두 참조한다. 모듈 하나가 여러 개의 모듈을 사용할 수 있다.
<br>
이렇게 되면 여러 파일에 걸쳐 재사용되는 함수나 변수를 모듈로 만들어두면 편리하다.
## 노드 내장 객체
### global
global 객체는 바르우저의 window와 같은 전역 객체로 모든 파일에서 접근할 수 있다.
<br>
또한 window.open 메서드를 그냥 open으로 호출할 수 있는 것처럼 global도 생략할 수 있다.
<br>
require 함수 또한 global.require에서 global이 생략된 것이고, console도 마찬 가지이다.
### console
console도 global 객체 안에 들어있으며, 보통 디버깅을 위해 사용한다.
<br>
개발하면서 변수에 값이 제대로 들어 있는지 확인하기 위해 사용되고, 에러 발생 시 내용을 확인하기 위해 사용한다.
<br>
로깅 함수들은 아래와 같다.
<ul>
<li><b>console.time(레이블)</b> :console.timeEnd과 대응되어 같은 레이블을 가진 time과 timeEnd 사이의 시간을 측정</li>
<li><b>console.log(내용)</b> : 평범한 로그를 콘솔에 표시, 여러 내용 동시에 표시 가능</li>
<li><b>console.error(에러 내용)</b> : 에러를 콘솔에 표시</li>
<li><b>console.table(배열)</b> : 배열의 요소로 객체 리터럴을 넣으면 객체의 속성들이 테이블 형식으로 표시</li>
<li><b>console.dir(객체, 옵션)</b> : 객체를 콘솔에 표시할 때 사용</li>
<li><b>console.trace(레이블)</b> : 에러가 어디서 발생했는지 추적할 수 있게 해줌</li>
</ul>
### 타이머
