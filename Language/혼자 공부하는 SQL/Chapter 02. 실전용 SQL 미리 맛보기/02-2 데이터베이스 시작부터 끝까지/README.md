# 데이터베이스 시작부터 끝까지 정리 내용

## 시작하기 전에

데이터베이스는 데이터를 저장하는 공간이다.
<br>
MySQL을 설치한 후에는 가장 먼저 데이터베이스를 준비하고 데이터베이스 안에 테이블을 생성해야 한다.
<br>
<br>
테이블은 2차원의 표 형태로 이루어져 있으며, 각 열에 해당하는 데이터를 한 행씩 입력할 수 있다.
<br>
필요하다면 행에 입력된 데이터를 수정하거나 삭제할 수도 있다.
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

## 테이블 설계하기
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


## 테이블 생성하기

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

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/df699617-b788-477b-9297-aa22941a1b3a)

<br>

## 데이터 입력하기
테이블 구성까지 완료하였으므로 이제는 실제로 데이터를 입력할 차례이다.
<br>
데이터는 행(가로) 단위로 입력하며, 회원 테이블에는 4건, 제품 테이블에는 3건의 데이터를 입력한다.
<br>
<br>
입력된 결과는 다음 그림과 같다.
<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/429b7866-4bd9-4522-975f-701cdff3ae80)

<br>

1. MySQL Workbench 창의 [SCHMEAS] 패널에서 [shop_db] - [Tables] - [member]를 선택하고 마우스 오른쪽 버튼 클릭 후 [Select Rows - Limits 1000]을 선택함

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/e2628b7a-60c9-4d11-9d07-3a8e7fcbf4df)

<br>

2. MySQL Workbech 창의 중앙에 [Result Grid] 창이 나타나는데, 아직 모두 NULL로 표시되어 있음(데이터가 하나도 없음)


<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/3c573c5d-2ac9-4f9f-aa71-1699c44674de)

<br>

3. [member_id], [member_name], [member_addr] 항목의 'NULL' 부분을 클릭해서 다음과 같이 데이터를 입력하고 Apply 버튼을 클릭하면 입력한 내용이 SQL로 생성됨

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/36b0042b-90ed-45a3-9de5-ce4a5d51883f)


<br>

4. Apply SQL Script to Database 창에서 [Apply]와 [Finish] 버튼을 클릭하면 데이터가 입력됨

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/fcd1b67d-0753-47f1-a4cd-2f7289f1d83a)

<br>

5. 위와 같은 방법으로 제품 테이블(product)에 테이블 3건을 추가

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/4694b1bd-7371-42c6-bdf4-96fc7a439047)


<br>

6. 이번에 수정, 삭제를 진행하기 위해 먼저 데이터를 추가해야 하는데, 다시 [SCHEMAS] 패널의 회원 테이블(member)에서 마우스 오른쪽 버튼을 클릭하고 [Select Rows - Limits 1000]을 선택 후 연습용 데이터를 1건 입력한 후 [Apply] 버튼을 연속 2회 클릭하고, [Finish] 버튼을 클릭해서 입력 데이터를 적용함

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/9ce49558-0951-4b83-95f7-55b4d82fbf17)

<br>

7. 조금 전에 입력한 데이터 내뇽을 수정하려면, 수정할 데이터를 클릭하고 변경하면 되는데, 여기서는 주소를 변경했으며 변경 후 [Apply] 버튼 클릭

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/e07ec4cd-0b2a-45f3-9e64-151b9c5714b9)

<br>

8. 이번에는 데이터 삭제인데, 삭제하고자 하는 행의 제일 앞 부분을 클릭하면 행이 파란색으로 선택되는데, 그 상태에서 마우스 오른쪽 버튼을 클릭하고 [Delete Row] - [Apply] 를 진행

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/2f0795a1-5aed-4515-9791-5d9f5158f0a3)

<br>

## 데이터 활용하기
이번에는 데이터를 활용하기 위한 방법인데, SQL에서는 데이터베이스를 활용하기 위해 주로 SELECT문을 이용한다.

<br>

1. 새 SQL을 입력하기 위해 툴바의 새로운 탭 아이콘을 클릭하고, 실행된 결과를 보기 위해 Output 아이콘을 클릭하여 [Output] 패널을 활성화 함

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/0ccbf99f-7424-45de-8b09-d17ee636dc82)

<br>

2. 작업할 데이터베이스를 선택하기 위해 [SCHMEAS] 패널의 'shop_db'를 더블 클릭하고, 진하게 변경되면 앞으로 쿼리 창에 입력할 SQL이 선택된 shop_db에 적용된다는 의미

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/feff44b6-d0b2-45b6-903a-d0aa3e8ef165)

<br>

3. 먼저 회원 테이블의 모든 해을 조회하기 위해 다음 SQL을 입력함<br>SELECT의 기본 형식은 SELECT 열_이름 FROM 테이블_이름 [WHERE 조건]이고, *는 모든 열을 의미함<br>따라서 '회원 테이블의 모든 열을 보여줘 라는 의미임'<br>툴바에서 실행 아이콘을 클릭하면 [Result Grid] 창에는 결과가, [Output] 패널에는 현재 결과의 건수와 조회하는 데 소요된 시간초)이 표시됨

```
SELECT * FROM member;
```

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/6b0d2fa7-1a02-467e-b2ac-152db081d047)

<br>


4. 회원 테이블 중 주소만 출력하기 위해 아래와 같이 새로 입력한 후 실행<br>열 이름 부분에 회원 이름(member_name)과 주소(member_addr)만 나오는데, 이렇게 여러 개의 열 이름을 콤마로 분리하면 필요한 열만 추출됨

```
SELECT member_name, member_addr FROM member;
```

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/a046d218-f09d-4619-9487-97bbd050a665)

<br>

5. 아이유 회원에 대한 정보만 추출하려고 하는데,아래와 같이 기존 SQL을 지우지 말고 다음 줄에 아래와 같이 입력한 후 실행<br>WHERE 다음에 특정 조건을 입력하여 회원 이름(member_name)이 '아이유'인 회원만 출력되도록 한 것<br>그런데 [Result Grid] 창의 아래쪽 탭을 보면 2개의 SQL이 모두 실행되었는데, SQL을 실행할 때 쿼리 창에 모든 SQL을 수행하기 때문임

```
SELECT * FROM member WHERE member_name = '아이유';
```

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/5c74c3be-4896-48f8-bcfd-d4ee8b28346f)

<br>

6. 5번과 같은 경우를 방지하기 위해 1개의 SQL만 남기고 모두 지우는 방법도 있지만, 더 편리한 방법은 필요한 부분만 마우스로 드래그해서 선택한 후 실행하는 것임

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/1088ffd6-b32f-4828-abe6-ff2051ca1091)


<br>
