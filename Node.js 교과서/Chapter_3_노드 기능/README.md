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
타이머 기능을 제공하는 함수들도 노드에서 gloab 객체 안에 들어있다.
<br>
set 타이머 함수들은 모두 아이디를 반환하며, 이 아이디를 사용하여 타이머를 취소할 수 있다.
<pre>
setTimeout(콜백 함수, 밀리초) : 주어진 밀리초 이후에 콜백 함수를 실행한다.
setInterval(콜백 함수, 밀리초) : 주어진 미리초마다 콜백 함수를 실행한다.
setImmeidate(콜백 함수) : 콜백 함수를 즉시 실행한다

clearTimeout(아이디) : setTimeout을 취소한다.
clearInterval(아이디) : setInterval을 취소한다.
clearImmediate(아이디) : setImmediate를 취소한다.
</pre>

### __filename, __dirname
노드에서 파일 사이에 모듈 관계가 있는 경우가 많아 현재 파일의 경로나 파일명을 알아야 한다.
<br>
노드는 __filename, __dirname 키워드로 경로에 대한 정보를 제공한다.
<pre>
console.log(__filename);
console.log(__dirname);

===== 실행 결과 =====
C:\Users\zerocho\filename.js
C:\Users\zerocho
</pre>
__filename은 현재 파일의 위치를 나타내며, __dirname은 현재 파일의 디렉터리 위치를 나타낸다.

### require
require 함수는 모듈을 불러오는 함수이고, 함수는 객체이므로 require는 객체로서 몇 가지 속성을 갖고 있다.
<br>
require는 반드시 파일 최상단에 위치할 필요없이 아무 곳에서나 사용할 수 있다.
<br>
<br>
한 번 require 파일은 require.cache에 저장되므로 다음 번에 require할 때 새로 불러오지 않고 require.cache에 있는 것이 재사용 된다.
<br>
만약 새로 require 하길 원한다면 require.cache의 속성을 제거하면 된다.
<pre>
const express = require('express')
const fs = ('fs')
</pre>
### process
process 객체는 현재 실행되고 있는 노드 프로세스에 대한 정보를 담고 있다.
<ul>
<li><b>process.arch</b> : 프로세서의 아키텍처 정보</li>
<li><b>process.version</b> : 설치된 노드의 버전</li>
<li><b>process.pid</b> : 현재 프로세스의 아이디</li>
<li><b>process.platform</b> : 운영체제 플랫폼 정보</li>
<li><b>process.uptime()</b> : 프로세스가 시작된 후 흐른 시간으로 단위는 초</li>
<li><b>process.execPath</b> : node.exe 즉, 노드의 경로</li>
<li><b>process.cwd()</b> : 현재 프로세스가 실행되는 위치</li>
<li><b>process.cpuUsage()</b> : 현재 CPU 사용 량</li>
</ul>

## 노드 내장 모듈
### os
os 모듈에는 웹 브라우저에 사용되는 자바스크립트의 운영체제 정보를 볼 수 있다.
<ul>
<li><b>os.arch()</b> : 프로세서의 아키텍처 정보</li>
<li><b>os.platform()</b> : 운영체제 플랫폼 정보</li>
<li><b>os.type()</b> : 운영체제 정보</li>
<li><b>os.hostname()</b> : 컴퓨터의 이름</li>
<li><b>os.release()</b> : 운영체제 버전</li>
<li><b>os.homedir()</b> : 홈 디렉터리 경로</li>
<li><b>os.cpus()</b> : 컴퓨터의 코어 정보</li>
<li><b>os.tmpdir()</b> : 임시 파일 저장 경로</li>
<li><b>os.freemem()</b> : 사용 가능한 메모리(RAM)</li>
<li><b>os.totalmem()</b> : 전체 메모리 용량</li>
</ul>

### path
path는 폴더와 파일의 경로르 쉽게 조작할 수 있도록 도와주는 모듈이다.
<ul>
<li><b>path.sep</b> : 경로의 구분자로 윈도우는 \, POSIX는 /</li>
<li><b>path.delimiter</b> : 환경 변수의 구분자로 윈도우는 세미콜론(;), POSIX는 콜론(:)</li>
<li><b>path.dirname(경로)</b> : 파일이 위치한 경로</li>
<li><b>path.extname(경로)</b> : 파일의 확장자</li>
<li><b>path.basename(경로, 확장자)</b> : 파일의 이름(확장자 포함)을 표시, 파일의 이름만 표시하고 싶다면 2번째 매개변수에 확장자 추가</li>
<li><b>path.parse(경로)</b> : 파일 경로를 root, dir, base, ext, name으로 분리</li>
<li><b>path.format(객체)</b> : path.parse()한 객체를 파일 경로로 합침</li>
<li><b>path.normalize(경로)</b> : /나 \를 실수로 여러 번 사용했거나 혼용했을 때 정상적인 경로로 변환</li>
<li><b>path.isAbsolute(경로)</b> : 파일의 경로가 절대경로인지 상대경로인지 true나 false 반환</li>
<li><b>path.relative(기준경로, 비교경로)</b> : 경로를 두 개 넣으면 첫 번째 경로에서 두 번째 경로로 가는 법을 알려줌</li>
<li><b>path.join(경로, ...)</b> : 여러 인수를 넣으면 하나의 경로로 합침</li>
<li><b>path.resolve(경로, ...)</b> : path.join()과 비슷하지만 \를 만나면 절대 경로로 인식</li>
</ul>
