# 03-1. 기본 중에 기본 정리 내용

SELECT는 SQL 문에서 가장 많이 사용되는 문법으로, 데이터베이스에서 데이터를 구축한 후 그 내용들을 활용한다.
<br>
데이터를 아무리 완벽하게 준비해 놓았더라도 활용을 잘 못하면 의미가 없다.

<br>

## 시작하기 전에
SELECT 문은 구축이 완료된 테이블에서 데이터를 추출하는 기능을 한다.
<br>
그러므로 SELECT를 아무리 많이 사용해도 기존의 데이터가 변경되지는 않는다.
<br>
<br>
SELECT의 가장 기본 형식은 SELECT ~ FROM ~ WHERE 이다.
<br>
SELECT 바로 다음에는 열 이름이, FROM 다음에는 테이블 이름이 나온다.
<br>
<br>
WHERE 다음에는 조건식이 나오는데, 조건식을 다양하게 표현함으로써 데이터베이스에서 원하는 데이터를 뽑아낼 수 있다.
<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/219dc903-25c5-4141-90ab-cb9fa4d6b616)

<br>

## 실습용 데이터베이스 구축


### 실습용 데이터베이스 개요

데이터베이스를 실습하기 위해 아래 구조의 데이터베이스를 사용한다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/6b6482db-ae80-474f-9a3b-11b7371af6fc)

<br>

'인터넷 마켓 DB 구성도'는 인터넷 마켓에서 운영하는 데이터베이스를 단순화한 것이라고 생각하면 된다.
<br>
이 인터넷 마켓은 특별히 그룹으로 이루어진 가수만 가입되도록 한다고 가정한다.
<br>
<br>
가수 그룹의 리더는 물건을 사기 위해 회원가입을 한다.
<br>
입력한 회원 정보는 회원 테이블(member)에 입력된다.
<br>
<br>
물론, 더 많은 정보를 입력해야 하지만 간단히 아이디/이름/인원/주소/국번/전화번호/평균 키/데뷔 일자 등만 입력한다.
<br>
<br>
회원가입을 한 후에 인터넷 마켓에서 물건을 구입하면 회원이 구매한 정보는 구매 테이블에 기록된다.
<br>
그러면 인터넷 마켓의 배송 담당자는 구매 테이블(buy)을 통해서 회원이 주문한 물건을 준비하고, 회원 테이블(member)에서 구매 테이블(buy)의 아이디와 일치하는 회원의 아디이를 찾아서 그 행의 주소로 물품을 배송한다.
<br>
<br>
예를 들어, 배송 담당자는 구매 테이블(buy)에서 BLK라는 아이디를 가진 회원이 구매한 지갑 2개, 맥북 프로 1개, 청바지 3벌을 포장한 후 회원 테이블(member)에서 BLK라는 아이디를 찾는다.
<br>
<br>
그리고 포장박스에 이름은 '블랙핑크', 주소는 '경남', 연락처는 '055-222-2222'라고 적어서 배송하면 된다.

<br>

### 실습용 데이터베이스 만들기

1. market_db.sqml 다운로드



2. MySQL Workbench를 실행해서 열려 있는 쿼리 창은 모두 닫는다.<br>그리고 File - Open SQL Scription 메뉴를 선택하고 앞에서 다운로드한 'market_db.sql'을 선택한 후 열기 버튼을 클릭한다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/51a62b1c-1b32-4563-b314-a034c507906f)

<br>

3. Execute 아이콘을 클릭해서 SQL을 실행하면 Rsult Grid 창의 하단 member 1 탭을 클릭해서 회원 테이블(member)의 완성된 상태를 확인한다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/1b284750-70b7-4d57-b352-8101dde7a5a4)


<br>

4. 이번에는 Result Grid 창의 하단에서 buy 2 탭을 클릭 후 구매 테이블(buy)을 확인해 본다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/a4c8eb95-1c64-4b0a-bab0-7c7530edcfa9)

<br>

### market_db.sql 파일 내용 살펴보기


### 데이터베이스 만들기

```
DROP DATABASE IF EXISTS market_db;
```

DROP DATABASE는 market_db를 삭제하는 문장이다.
<br>
이는 market_db.sql을 처음 실행할 때는 필요 없다.
<br>
하지만 책을 학습하다 보면 다시 market_db.sql을 실행할 일이 있기 때문의 기존의 DB를 삭제하는 것이다.

<br>

```
CREATE DATABASE market_db;
```

데이터베이스를 새로 만든다.

<br>


### 회원 테이블(member) 만들기

```
USE market_db;
CREATE TABLE member -- 회원 테이블(member)
( mem_id	   CHAR(8) NOT NULL PRIMARY KEY, -- 회원 아이디(PK)
  mem_name	   VARCHAR(10) NOT NULL, -- 이름
  mem_number   INT NOT NULL, -- 인원수
  addr         CHAR(2) NOT NULL, -- 주소(경기, 서울, 경남, 식으로 2글자만 입력)
  phone1       CHAR(3), -- 연락처의 국번(02, 031, 055 등)
  phne2        CHAR(8), -- 연락처의 나머지 전화번호(하이픈 제외)
  height       SMALLINT,  -- 평균 키
  debut_date   DATE -- 데뷔 일자
);
```

```
USE market_db;
```
USE 문은 market_db 데이터베이스를 선택하는 문장이다.
<br>
MySQL Workbench의 SCHMEAS 패널에서 shop_db 데이터베이스를 더블 클릭 하는 것과 동일한 효과를 갖는다.

<br>

```
CREATE TABLE member -- 회원 테이블(member)
( mem_id	   CHAR(8) NOT NULL PRIMARY KEY, -- 회원 아이디(PK)
  mem_name	   VARCHAR(10) NOT NULL, -- 이름
  mem_number   INT NOT NULL, -- 인원수
  addr         CHAR(2) NOT NULL, -- 주소(경기, 서울, 경남, 식으로 2글자만 입력)
  phone1       CHAR(3), -- 연락처의 국번(02, 031, 055 등)
  phne2        CHAR(8), -- 연락처의 나머지 전화번호(하이픈 제외)
  height       SMALLINT,  -- 평균 키
  debut_date   DATE -- 데뷔 일자
);
```

member 테이블을 만드는 과정이다.
<br>
주석에는 해당하는 한글 이름이 표기되어 있다.

<br>

### 구매 테이블(buy) 만들기
```
CREATE TABLE buy -- 구매 테이블(buy)
(  num			INT AUTO_INCREMENT NOT NULL PRIMARY KEY, -- 순번(PK)
   mem_id   	CHAR(8) NOT NULL, -- 아이디(FK)
   prod_name	CHAR(6) NOT NULL, -- 제품 이름
   group_name   CHAR(4), -- 분류
   price		INT NOT NULL, -- 단가
   amount       SMALLINT NOT NULL, -- 수량
   FOREIGN KEY (mem_id) REFERENCES member(mem_id)
);
```

```
CREATE TABLE buy -- 구매 테이블(buy)
```

구매 테이블을 생성한다.

```
num			INT AUTO_INCREMENT NOT NULL PRIMARY KEY, -- 순번(PK)
```
AUTO_INCREMENT는 자동으로 숫자를 입력해준다는 의미이다.
<br>
즉, 순번은 직접 입력할 필요가 없이 1, 2, 3, ... 과 같은 방식으로 자동으로 증가한다.

<br>

### 데이터 입력하기
```
INSERT INTO member VALUES('TWC', '트와이스', '9', '서울', '02', '11111111', 167, '2015.10.19');
INSERT INTO buy VALUES(NULL, 'BLK', '지갑', NULL, 30, 2);
```


```
INSERT INTO member VALUES('TWC', '트와이스', '9', '서울', '02', '11111111', 167, '2015.10.19');
```
회원 테이블(member)에 값을 입력한다.
<br>
CHAR, VARCHAR, DATE 형은 작은따옴표로 값을 묶어주고, INT형은 그냥 넣어주면 된다.


```
INSERT INTO buy VALUES(NULL, 'BLK', '지갑', NULL, 30, 2);
```
구매 테이블(buy)의 첫 번째 열인 순번(num)은 자동으로 입력되므로 그 자리에는 NULL이라고 써주면 된다.
<br>
그러면 알아서 1, 2, 3 ... 으로 증가하면서 입력되고, 여기서는 처음이므로 1이 입력된다.

<br>

### 데이터 조회하기
market_db.sql 파일의 마지막 2개 행에서는 입력된 내용을 확인하기 위해서 SELECT로 조회한다.

```
SELECT * FROM member;
SELECT * FROM buy;
```


<br>


## 기본 조회하기: SELECT ~ FROM
