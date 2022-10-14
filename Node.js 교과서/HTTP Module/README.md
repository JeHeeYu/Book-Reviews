# HTTP Modeule 정리 내용

## 요청과 응답
클라이언트에서 서버로 요청(Request)을 보내고 서버에서 요청의 내요을 읽고 처리한 뒤 클라이언트로 응답(Response)를 보낸다.
<br>
따라서 서버에는 요청을 받는 부분과 응답을 보내는 부분이 존재해야 한다.
<br>
<br>
요청과 응답은 이벤트 방식으로, 클라이언트로부터 어떤 요청이 왔을 때 어떤 작업을 수행할지 이벤트 리스너를 등록해야 한다.
<pre>
// createServer.js

const http = require('http');

http.createServer((req, res) => {
    // 동작 정의
});
</pre>
http 서버가 있어야 웹 브라우저의 요청을 처리할 수 있으므로 http 모듈을 사용한다.
<br>
http의 createServer 메서드를 사용하여 인수로 요청에 대한 콜백 함수를 넣을 수 있으며 요청이 들어오면 콜백 함수가 실행된다.
<br>
<br>
콜백 안에 req와 res 매개변수가 있으며, req는 요청에 관한 정보를, res는 응답에 관한 정보를 담고 있다.
<br>
<pre>
// server1.js

const http = require('http');

http.createServer((req, res) => {
    res.writeHead(200, { 'Content-Type': 'text/html; charset=utf-8' });
    res.write('<h1>Hello Node!</h1>');
    res.end('<p>Hello Server!</p>');
}).listen(8080, () => {
    // 서버 접속 시 동작 정의
});
</pre>
createServer 메서드 뒤에 listen 메서드를 붙이고 클라이언트에 공개할 포트 번호(8080)와 포트 연결 후 실행될
<br>
콜백 함수를 넣는다. 그러면 서버는 8080 포트에서 포트에서 요청이 오기를 기다린다.
<br>
<br>
res 객체에는 res.writeHead와 res.wirte, res.end 메서드가 있다.
<br>
res.writeHead : 응답에 대한 정보를 기록하는 메서드로 첫 번째 인수로 성공적인 요청임을 의미하는 200,
<br>
두 번째 인수로 응답에 대한 정보를 보내는데, HTML 콘텐츠 형식을 알려준다.
<br>
이 정보가 기록되는 부분을 헤더라고 한다.
<br>
<br>
res.write : 메서드의 첫 번째 인수는 클라이언트로 보낼 데이터로, 데이터가 기록되는 부분을 본문(body)라고 한다.
<br>
<br>
res.end : 응답을 종료하는 메서드로 만약 인수가 있다면 그 데이터도 클라이언트로 보내고 응답을 종료한다.
<br>
<br>
listen 메서드에 콜백 함수를 넣는 대신 listening 이벤트 리스너를 붙여도 된다.
<pre>
// server1-1.js

const http = require('http');

http.createServer((req, res) => {
    res.writeHead(200, { 'Content-Type': 'text/html; charset=utf-8' });
    res.write('<h1>Hello Node!</h1>');
    res.end('<p>Hello Server!</p>');
});
listen(8080);

server.on('listening', () => {
    // 동작 정의
});
server.on('error', (error) => {
    // 동작 정의
});
</pre>
<br>
한 번에 여러 서버를 실행할 수도 있다.
<pre>
// server1-2.js

const http = require('http');

http.createServer((req, res) => {
    res.writeHead(200, { 'Content-Type': 'text/html; charset=utf-8' });
    res.write('<h1>Hello Node!</h1>');
    res.end('<p>Hello Server!</p>');
}).listen(8080, () => {
    // 서버 접속 시 동작 정의
});

http.createServer((req, res) => {
    res.writeHead(200, { 'Content-Type': 'text/html; charset=utf-8' });
    res.write('<h1>Hello Node!</h1>');
    res.end('<p>Hello Server!</p>');
}).listen(8081, () => {
    // 서버 접속 시 동작 정의
});
</pre>

## HTTP 상태 코드
200, 500과 같은 숫자는 HTP의 상태 코드로, 요청이 성공했는지 실패했는지를 판단한다.
<ul>
  <li>2XX : 성공을 알리는 코드로 대표적으로 200(성공), 201(작성됨)이 많이 사용</li>
  <li>3XX : 리다이렉션(다른 홈페이지로 이동)을 알리는 상태 코드로, 어떤 주소를 입력했는데 다른 주소의 페이지로 넘어갈 때 이 코드가 사용 대표적으로 301(영구 이동), 302(임시 이동)가 있으며 304(수정되지 않음)는 요청의 응답으로 캐시를 사용했다는 듯</li>
  <li>4XX : 요청 오류를 나타내며 요청 자체에 오류가 있을 때 표시, 대표적으로 400(잘못된 요청), 401(권한 없음), 403(금지됨), 404(찾을 수 없음)이 있음</li>
  <li>5XX : 서버 오류를 나타내며 요청은 제대로 왔지만 서버에 오류가 생겼을 때 발생, 대표적으로 500(내부 서버 오류), 502(불량 게이트웨이), 503(서비스를 사용할 수 없음)이 있음</li>
</ul>
HTTP 상태 코드는 반드시 전송해야 한다. 요청 처리 과정 중 에러가 발생 후 응답을 보내지 않으면 클라이언트는 응답을 기다리는데,
<br>
하염없이 응답을 기다리다 일정 시간 후 Timeout(시간 초과)가 발생한다.
