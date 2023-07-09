# 데이터베이스 개체 정리 내용

## 시작하기 전에
테이블은 데이터베이스의 핵심 개체이다.
<br>
하지만 데이터베이스에서는 테이블 외에 인덱스, 뷰, 스토어드 프로시저, 트리거, 함수, 커서 등의 개체도 필요하다.
<br>
<br>
인덱스는 데이터를 조회할 때 **결과가 나오는 속도를 획기적으로 빠르게** 해주고, 뷰는 **테이블의 일부를 제한적으로 표현할 때 주로 사용**한다.
<br>
스토어드 프리시저는 **SQL에서 프로그래밍이 가능하도록** 해주고, 트리거는 잘못된 **데이터가 들어가는 것을 미연에 방지하는 기능**을 한다.
<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/85fb40a9-f750-4ab5-812e-a925116b3513)

<br>

## 인덱스(Index)
데이터를 조회할 때 테이블에 데이터가 적다면 결과가 금방 나오지만 데이터가 많아질수록 결과가 나오는 시간이 많이 소요된다.
<br>
인덱스는 이런 경우 결과가 나오는 시간을 대폭 줄여준다.

<br>

### 인덱스 개념 이해하기
인덱스란 책의 데일 뒤에 수록되는 '찾아보기'와 비슷한 개념이다.
<br>
책의 내용 중에서 특정 단어를 찾고자 할 때, 책의 처음부터 마지막까지 한 페이지씩 전부 찾아보는 것은 상당히 오래 걸린다.
<br>
그래서 찾아보기를 통해 먼저 해당 단어를 찾고 바로 옆에 적혀 있는 페이지로 이동하는 효율적인 방법을 사용하는 것이다.
<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/803213d1-8e33-4aea-a699-64de94c63f42)

<br>

예제에서는 데이터들의 양이 많지 않기 때문에 인덱스의 필요성을 느끼지 못할 수도 있다.
<br>
하지만 실무에서는 많게는 수천만 ~ 수억 건 이상의 데이터를 처리할 때 인덱스 없이 전체 데이터를 찾아본다는 것은 상상조차 할 수 없다.

<br>

### 인덱스 실습

1. MySQL Workbench을 열고 툴바에서 Create a new SQL tab for executing queries 아이콘을 클릭하고 SCHMEAS 패널의 'shop_db'를 더블 클릭해서 선택한다.

<br>


![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/81f04496-b0ce-4206-bf91-620f1d7b37a7)

<br>

2. 회원 테이블에는 아직 인덱스를 만들지 않았으므로 책과 비교하면 책의 찾아보기가 없는 상태이다. 이 상태로 책에서 어떤 단어를 찾는다면 당연히 1페이지부터 전체를 찾아봐야 할 것이다. 그러므로 테이블에서 '아이유'를 찾을 때는 회원 테이블의 1행부터 끝까지 전체를 살펴봐야 한다.<br>아래 SQL문을 실행한다.

```
SELECT * FROM member WHERE member_name = '아이유';
```

<br>


![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/b09b763a-82b7-4cb5-a1ab-33f6fce0dcd1)

<br>

3. 결과는 당연히 아이유를 잘 찾는다. 어떻게 아이유를 찾았는지 확인하기 위해 Execution Plan 탭을 클릭하면 Full Table Scan 이라고 나오는데, 이것을 해석하면 전체 테이블 검색이다. 처음부터 끝까지 엄청나게 오랜 시간이 걸려서 '아이유'를 찾은 것이다. 현재는 인덱스가 없으므로 별다른 방법이 없다.
> Execution Plan 탭이 보이지 않으면 아래 방향 화살표를 클릭하면 제일 아래에 보임

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/7229528e-960e-4ad0-ace2-f0724d24b1a1)

<br>

4. 이제 회원 테이블에 인덱스를 만들 차례로, 아래 SQL을 실행하면 인덱스가 생성되고, 인덱스는 열에 지정한다.
> SQL 마지막에 ON member(member_name)의 의미는 member 테이블의 member_name 열에 인덱스를 지정하라는 의미로, 결과는 특별히 눈에 보이지 않음

```
CREATE INDEX idx_member_name ON member(member_name);
```

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/f3778ef8-94d8-4108-8baf-d37ce992afd6)

<br>

5. 이제 인덱스가 생긴 회원 테이블에서 아이유를 찾아보려 하며, 2번에서 사용한 SQL문을 다시 실행한다. 결과는 동일하나 이번에는 차는 방법이 달라졌다.<br>Execution Plan 탭을 보면 Non-Unique Key Lookup 이라고 나오는데, Key Lookup은 인덱스를 통해 결과를 찾았다고 하는 것이다.
> 이런 방법을 인덱스 검색(Index Scan) 이라고 함

```
SELECT * FROM member WHERE member_name = '아이유';
```

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/8722e92a-fb8c-43b4-84aa-bc4a2c28f088)

<br>

인덱스에서 한 가지 더 기억해야 할 점은 인덱스 생성 여부에 따라 결과가 달라지는 것은 아니다.
<br>
즉 책의 내용을 찾을 때 찾아보기가 있으면 시간을 단축하는 효과는 있지만, 책의 찾아보기가 없어도 책의 첫 페이지부터 찾아야 하기 때문에 시간이 오래 걸릴 뿐 어차피 동일하게 찾을 수 있다.

<br>

## 뷰(View)
뷰는 테이블과 상당히 동일한 성격의 데이터베이스 개체이다.
<br>
뷰를 활용하면 보안도 강화하고, SQL문도 간단하게 사용할 수 있다.

<br>

### 인덱스 개념 이해하기
뷰를 한마디로 정의하면 '가상의 테이블' 이라고 할 수 있다.
<br>
일반 사용자의 입장에서는 테이블과 뷰를 구분할 수 없다.
<br>
<br>
즉, 일반 사용자는 테이블과 동일하게 뷰를 취급하면 된다.
<br>
다만 뷰는 실제 데이터를 가지고 있지 않으며, 진짜 테이블에 링크(Link)된 개념이라고 생각하면 된다.
<br>
<br>
뷰는 윈도우즈의 '바로  가기 아이콘'과 비스한 개념이다.
<br>
바탕 화면의 바로 가기 아이콘을 더블 클릭해서 실행하지만, 실제로 실행되는 파일은 다른 폴더에 있다.
<br>
<br>
지금까지 아이콘을 더블 클릭해서 프로그램이 실행된다고 해서 아무런 문제가 없다.
<br>
<br>
뷰도 비슷한 개념으로 실체는 없으며 테이블과 연결되어 있는 것뿐이다.
<br>
사용자가 뷰를 테이블처럼 생각해서 접근하면 알아서 테이블에 연결해준다.
<br>
<br>
그렇다면 뷰의 실체는 무엇이냐면 바로 SELECT 문이다.

<br>


![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/dbd34319-d8f1-424b-91b0-c02e9410de9f)

<br>

### 뷰 실습하기

1. MySQL Workbench을 열고 툴바에서 Create a new SQL tab for executing queries 아이콘을 클릭하고 SCHMEAS 패널의 'shop_db'를 더블 클릭해서 선택한다.

<br>


![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/81f04496-b0ce-4206-bf91-620f1d7b37a7)

<br>

2. 기본적인 뷰를 맏늘기 위해 회원 테이블과 연결되는 회원 뷰(member_view)를 만들기 위해 다음과 같이 SQL을 실행한다.<br>Output 패널에 초록색 체크 표시가 나타나면 SQL이 제대로 실행되었다는 의미이다.

```
CREATE VIEW member_view
AS
	   SELECT * FROM member;
```

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/6970ba1d-20e9-4859-8d80-58ba0a43d3ff)

<br>

3. 이제 회원 테이블(member)이 아닌 회원 뷰(member_view)에 접근해야 한다. 뷰에 접근하는 것은 테이블에 접근하는 것과 동일하다.<br>다음 SQL을 실행하면 회원 테이블에 접근했을 때와 동일한 결과가 나온다. 즉, 바탕 화면의 바로 가기 아이콘을 더블 클릭하던, 원본 프로그램을 실행하던 플그램이 실행되는 것과 동일한 개념이다.

```
SELECT * FROM member_view;
```

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/0f4a2759-6ed2-41b6-8b6d-16576e6e0949)

<br>

여기서 중요한 점이, 테이블을 사용하지 않고 왜 뷰를 사용하느냐이다.
- 보안에 도움이 됨
- 긴 SQL문을 간략하게 만들 수 있음

<br>

## 스토어드 프로시저(Stored Procedure)
스토어드 프로시저를 통해 SQL 안에서도 일반 프로그래밍 언어처럼 코딩을 할 수 있다.
<br>
일반 프로그래밍보다는 좀 불편하지만, 프로그래밍 로직을 작성할 수 있어서 때론 유용하게 사용된다.

<br>


### 스토어드 프로시저 개념 이해하기
스토어드 프로시저란 MySQL에서 제공하는 프로그래밍 기능으로, 여러 개의 SQL문을 하나로 묶어서 편리하게 사욯알 수 있다.
<br>
SQL을 묶는 개념 외에 C, 자바, 파이썬과 같은 프로그래밍 언어에서 사용되는 연산식, 조건문, 반복문 등을 사용할 수 있다.
<br>
<br>
스토어드 프로시저를 통해서 MySQL에서도 기본적인 형태의 일반 프로그래밍 로직을 코딩할 수 있다.
<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/e89144bc-412a-4721-919c-11fb2f193b08)

<br>

### 스토어드 프로시저 실습하기

1. MySQL Workbench을 열고 툴바에서 Create a new SQL tab for executing queries 아이콘을 클릭하고 SCHMEAS 패널의 'shop_db'를 더블 클릭해서 선택한다.

<br>


![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/81f04496-b0ce-4206-bf91-620f1d7b37a7)

<br>

2. 다음 두 SQL을 입력하고 한꺼번에 실행한다. 이 두 SQL은 앞으로도 상당히 자주 사용딘다고 가정한다. 매번 두 줄의 SQL을 입력해야 한다면 상당히 불편할 것이고, SQL의 문법을 잊어버리거나 오타를 입력할 수도 있다.

```
SELECT * FROM member WHERE member_name = '나훈아';
SELECT * FROM product WHERE product_name = '삼각김밥';
```

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/1cf0a3ae-3cc9-4e13-b38e-ae693f717362)


<br>

3. 두 SQL을 하나의 스토어드 프로시저로 만든다. 다음 SQL을 입력하고 실행한다. 첫 행과 마지막 행에 구분 문자라는 의미의 DELIMITER // ~ DELIMITER ; 문이 나왔는데, 일단 이것은 스토어드 프로시저를 묶어주는 약속으로 생각하면 된다. 그리고 BEGIN과 END 사이에 SQL 문을 넣으면 된다.

```
DELIMITER //
CREATE PROCEDURE myProc() 
BEGIN
	SELECT * FROM member WHERE member_name = '나훈아';
	SELECT * FROM product WHERE product_name = '삼각김밥';
END //
DELIMITER ;
```

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/6a165dc7-9bca-4ed8-88b7-b538a20e62f3)

<br>

4. 이제부터는 두 줄의 SQL문을 실행할 필요 없이 앞에서 만든 스토어드 프로시저를 호출하기 위해 CLAL 문을 실행하면 된다. 아래 SQL을 실행하면 두 SQL을 실행한 것과 동일한 것을 확인할 수 있다.

```
CALL myProc();
```

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/2eb8a037-766b-4ff5-b532-e1d9bdb593d1)

<br>
