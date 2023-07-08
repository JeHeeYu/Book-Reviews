# 데이터베이스 시작부터 끝까지 정리 내용

## 시작하기 전에

데이터베이스는 데이터를 저장하는 공간이다.
<br>
MySQL을 설치한 후에는 가장 먼저 데이터베이스를 준비하고 데이터베이스 안에 테이블을 생성해야 한다.
<br>
<br>
테이블은 2차원의 표 형태로 이루어져 있으며, 각 열에 해당하는 데이터를 한 행씩 입력할 수 있다.
<br>
필욯다ㅏ면 행에 입력된 데이터를 수정하거나 삭제할 수도 있다.
<br>
<br>
마지막으로 입력이 완료된 데이터를 조회해서 활용할 수 있다.
<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/f61a8787-cb66-4b8b-a6c1-deeae207923f)

<br>

## DBMS 설치하기
데이터베이스를 구축하기 위해 DBMS를 설치해야 한다.
<br>
앞선 1장에서 MySQL을 설치하였으니 DBMS까지 완료된 상태이다.
<br>
<br>
아직 DBMS 내부에 사용할 쇼핑몰 데이터베이스는 없는 상태이다.
<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/ab39bf70-a208-4644-87d4-00c04529c564)

<br>


## 데이터베이스 만들기
이제 DBMS 안에 데이터베이스를 만들 차례로, 이번 과목을 지나면 다음과 같이 비어 있는 '쇼핑몰 데이터베이스'가 생성된다.
<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/ec32806c-3694-446a-af8d-891c07d7a14a)

<br>

1. 먼저 MySQL Worbech에서 'Local Instance MySQL'을 클릭하고, 비밀번호(0000)를 입력한 후 OK 클릭

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/6fa3ad69-903a-4817-a61a-84bcb0160bdd)

<br>

2. 좌측 하단에 [Administration]과 [Schemas]가 탭으로 구분되어 있는데, [Schemas] 탭을 클릭하면 MySQL에 기본적으로 들어 있는 3개의 데이터베이스가 있으며 지금은 무시

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/40a45475-1170-451d-b29a-fe7d78cabcc9)

<br>

3. [SCHHEMAS] 패널의 빈 부분에서 마우스 오른쪽 버튼 클릭 후 [Create Schmea] 선택

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/3a82b8ce-6c69-412e-923f-89274ee79ab3)

<br>

4. 새로운 쿼리 창에서 [Name]에 쇼핑몰을 의미하는 'shop_db'를 입력하면 자동으로 탭 이름도 동일하게 변경되고, [Apply] 버튼을 클릭하면 Apply SQL Script to Database 창에 SQL 문이 자등으로 생성되고, 다시 [Apply]와 [Finish] 버튼을 클릭하면 좌측 [SCHEMAS] 패널의 목록에 'shop_db'가 추가됨
<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/503fe6bf-56f4-48e7-b514-50a735284610)


<br>

5. 탭을 닫기 위해 [File] - [Close Tab] 메뉴를 클릭하고 만약 Close SQL Tab 창이 나타나면 [Don't Save]를 선택함으로써 비어 있는 쇼핑몰 데이터베이스 완성

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/ff0758f9-56c7-4a84-94bd-87c077907af1)

<br>

## 테이블 만들기

이제 테이블을 만들 차례로, 아래와 같이 데이터베이스 안에 2개의 테이블을 생성해야 한다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/6668ca8e-2bba-4831-8135-ae226b8e13af)


<br>

### 테이블 설계하기
테이블을 생성하기 위해 설계도가 필요하며, 테이블을 설계한다는 것은 열 이름과 데이터 형식을 지정하는 것이다.
<br>
회원 테이블은 다음과 같이 설계를 완성했다고 가정한다.
<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/6fcf5a92-30c9-4dea-87ac-dc61046e266e)

<br>

회원 테이블은 아이디, 회원 이름, 주소 3개 열로 구성하고 각각의 영문 이름도 지정했다.
<br>
또한 데이터 형식은 모두 문자로 지정했다.
<br>
<br>
문자는 CHAR(Character) 라는 MySQL 문법상 이미 약속된 예약어를 사용해야 한다.
<br>
문자열 최대 길이도 적절히 지정한다.
<br>
<br>
널(Null)은 빈 것을 의미하며, 널 허용 안함(Not Null, NN)은 반드시 입력해야 한다는 의미이다.
<br>
회원은 당연하게 아이디와 이름은 있어야 하기 때문에 아이디 및 회원 이름 열은 꼭 입력하도록, 주소는 넣지 않아도 무방하도록 설계한다.
<br>
<br>
제품 테이블도 다음과 같이 설계를 완성했다고 가정한다.
<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/e637212a-ba92-4102-82d3-c270d9c5064d)

<br>

여기서 새로 등장한 INT는 Integer의 약자로 소수점이 없는 정수를 의미하고, DATE는 연, 월, 일을 입력한다.

<br>


### 테이블 생성하기

1. MySQL Workbench 창의 [SCHEMAS] 패널에서 'shop_db'의 >를 클릭해 확장하고 [Tables]를 마우스 오른쪽 버튼으로 클릭한 후 [Create Table]을 선택

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/6b9f4f1f-dcc9-4679-b641-1b618939cae6)


<br>

2. 우선 앞에서 설계한 회원 테이블의 내용을 입력하는데, [Table Name]에 member를, [Column Name]의 첫 번째 항목을 더블 클릭

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/01cbde5f-cf57-4b0e-a015-b840b9e412ea)

<br>

3. 첫 번째 [Column Name]은 member_id로, [Datatype]은 문자 8글자 이므로 CHAR(8)로 입력하고, 설계 시 아이디 열을 기본키로 설계했으므로 PK(기본 키)와 NN(Not Null)을 체크

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/00bc5fc4-7b2c-4885-993a-894c94c1ec61)


<br>

4. 위와 같은 방법으로 (member_name, CHAR(5)), (member_addr, CHAR(20))을 추가하고, [NN] 은 member_name만 체크하고, 우측 하단의 [Apply] 버튼을 클릭한다.
<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/3d844c55-547e-497a-b2f4-4223b779d2e9)

<br>

5. 위와 같은 방법으로 제품 테이블도 만들면 된다.

<br>


<br>
