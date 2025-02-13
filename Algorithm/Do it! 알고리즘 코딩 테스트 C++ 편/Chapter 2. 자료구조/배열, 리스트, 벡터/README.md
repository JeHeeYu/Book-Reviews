# 배열, 리스트, 벡터 정리 내용

## 배열과 리스트의 핵심 이론

### 배열
배열은 메모리의 연속 공간에 값이 채워져 있는 형태의 자료구조이다.
<br>
배열의 값은 인덱스를 통해 참조할 수 있으며, 선언한 자료형의 값만 저장할 수 있다.
<br>
<br>
다음 그림은 배열을 나타낸 것으로, 배열에 값이 채워져 있고 각 값은 0부터 5까지 인덱스가 지정되어 있다.
<br>

![image](https://user-images.githubusercontent.com/87363461/229275803-5c23f470-1a17-4060-abd5-707213b8dfc4.png)

<br>

### 배열의 특징
- 인덱스를 사용하여 값에 바로 접근할 수 있음
- 새로운 값을 삽입하거나 특정 인덱스에 있는 값을 삭제하기 어려움. 값을 삽입 및 삭제하기 위해 해당 인덱스 주변에 있는 값을 이동시키는 과정이 필요함
- 배열의 크기는 선언할 때 지정할 수 있으며 한 번 선언하면 크기를 늘리거나 줄일 수 없음
- 구조가 간단하므로 코딩 테스트에서 많이 사용

### 리스트
리스트는 값과 포인터를 묶은 노드라는 것을 포인터로 연결한 자료구조이다.
<br>
다음 그림이 리스트를 표현한 것이다.
<br>

![image](https://user-images.githubusercontent.com/87363461/229275869-cb8e5cb4-6fc4-49d4-9df2-d30e2e62084b.png)


<br>

### 리스트의 특징
- 인덱스가 없으므로 값에 접근하려면 Head 포인터부터 순서대로 접근해야 함. 즉 값에 접근하는 속도가 느림
- 포인터로 연결되어 있으므로 데이터를 삽입하거나 삭제하는 연산 속도가 빠름
- 선언할 때 크기를 별도로 지정하지 않아도 됨. 즉, 리스트의 크기는 정해져 있지 않으며 크기가 변하기 쉬운 데이터를 다룰 때 적절함
- 포인터를 저장할 공간이 필요하므로 배열보다 구조가 복잡함

### 벡터
벡터는 C++ 표준 라이브러리(STL)에 있는 자료구조 컨테이너 중 하나로, 사용자가 손쉽게 사용하기 위해 정의된 클래스이다.
<br>
기존의 배열과 같은 특징을 가지면서 배열의 단점을 보완한 동적 배열의 헝태라고 할 수 있다.

### 벡터의 특징
- 동적으로 원소를 추가할 수 있음. 즉, 크기가 자동으로 늘어남
- 맨 마지막 위치에 데이터를 삽입하거나 삭제할 때 문제가 없지만 중간 데이터의 삽입 삭제는 배열과 같은 메커니즘으로 동작함
- 배열과 마찬가지로 인덱스를 이용하여 각 데이터에 직접 접근할 수 있음

### 벡터의 기본 사용법
```
// 벡터 사용법

// 선언
vector<int> A;                // vector<자료형> 변수 이름; 형태로 선언

// 삽입 연산
A.push_back(1);               // 마지막에 1 추가
A.insert(A.begin(), 7);       // 맨 앞에 7 삽입
A.insert(A.begin() + 2, 10);  // index 2 위치에 10 삽입

// 값 변경
A[4] = -5;                    // index 4의 값을 -5로 변경

// 삭제 연산
A.pop_back();                 // 마지막 값 삭제
A.erase(A.begin() + 3);       // index 3 위치에 해당하는 값 삭제
A.clear();                    // 모든 값 삭제

// 정보 가져오기
A.size();           // 데이터 개수
A.front();          // 처음 값
A.back();           // 마지막 값
A[3];               // index 3에 해당하는 값
A.at(5);            // index 5에 해당하는 값
A.begin();          // 첫 번째 데이터 위치
A.end();            // 마지막 데이터 다음 위치
``` 


## 문제 001. 숫자의 합 구하기 (브론즈 4, 11720)

### [문제 링크](https://www.acmicpc.net/problem/11720)

### 1. 문제 분석
N의 범위가 1부터 100까지이므로 값을 int, long과 같은 숫자형으로 담을 수 없다.
<br>
먼저 문자열의 형태로 입력받은 후 문자 배열로 변환하고 문자 배열값을 순서대로 읽으면서 숫자형으로 변환하여 더해야 한다.
<br>
<br>
예를 들어 "1234"와 같이 문자열로 입력, 이를 다시 '1', '2', '3', '4'와 같이 문자 배열로 변환, 다시 배열을 1, 2, 3, 4로 변환하고 이를 더해 10을 구한다.
> 문자열을 숫자형으로 변경하기 위해 아스키코드를 이해해야 한다. 아스키코드에서 같은 의미의 문자와 숫자의 코드값 차이는 48이다.<br>예를 들어 문자 '1'은 아스키코드의 값이 49이므로 문자 '1'을 숫자 1로 변환하려면 '1' - 48 또는 '1' - '0'과 같이 연산하면 된다.

### 2. 손으로 풀어보기

1. 숫자의 개수만큼 입력받은 값을 string 형으로 저장한다.

```
string numbers = 10987654321
```

2. string 형으로 입력받은 값을 한 칸씩 나누어 char[] 형으로 변환한다.

![image](https://user-images.githubusercontent.com/87363461/229276459-093831b3-7e3f-49b8-b50b-75be03938e1e.png)

3. 인덱스 0부터 끝까지 배열을 탐색하며 각 값을 정수형으로 변환하고 결괏값에 누적한다.

![image](https://user-images.githubusercontent.com/87363461/229276479-feaaefd4-c3fd-492d-99d9-95a61a3bb373.png)

### 3. 슈도코드 작성
```
N값 받기
숫자를 string 변수(numbers)로 입력받기
sum 변수 선언

for(numbers 길이만큼 반복) {
    sum에 배열의 각 자리의 값을 정수화하여 더하기
}

sum 출력
```


### 4. 코드 구현

### [예제 코드](https://github.com/JeHeeYu/Book-Reviews/blob/main/Algorithm/Do%20it!%20%EC%95%8C%EA%B3%A0%EB%A6%AC%EC%A6%98%20%EC%BD%94%EB%94%A9%20%ED%85%8C%EC%8A%A4%ED%8A%B8%20C%2B%2B%20%ED%8E%B8/Chapter%202.%20%EC%9E%90%EB%A3%8C%EA%B5%AC%EC%A1%B0/%EB%B0%B0%EC%97%B4%2C%20%EB%A6%AC%EC%8A%A4%ED%8A%B8%2C%20%EB%B2%A1%ED%84%B0/11720.cpp)

<br>

## 문제 002. 평균 구하기 (브론즈1, 1546)

### [문제 링크](https://www.acmicpc.net/problem/1546)

### 1. 문제 분석
최고 점수를 기준으로 전체 점수를 다시 계산해야 하므로 모든 점수를 입력받은 후에 최고점을 별도로 저장해야 한다.
<br>
또한 문제에서 제시한 한 과목의 점수를 계산하는 식의 총합과 관련된 식으로 변환할 수 있다.
<br>
<br>
따라서 일일이 변환 점수를 구할 필요 없이 한 번에 변환한 점수의 평균 점수를 구할 수 있다.

```
// 변환 점수의 평균을 구하는 식

(A / M * 100 + B / M * 100 + C / M * 100) / 3 = (A * B * C) * 100 / M / 3
```

### 2. 손으로 풀어보기

1. 점수를 1차원 배열에 저장한다.

![image](https://user-images.githubusercontent.com/87363461/229327842-184f234a-6c0c-4bc4-bd60-06f7913f3c47.png)


2. 배열을 탐색하며 최고 점수와 점수의 총합을 구한다.

```
최고 점수 = 16, 총합 = 31
```

3. '총합 * 100 / 최고 점수 / 과목의 수'를 계산해 다시 계산한 점수의 평균값을 출력한다.

```
총합 * 100 / 최고 점수 / 과목의 수 = 31 * 100 / 16 / 5 = 38.75
```

### 3. 슈도코드 작성하기

```
N(시험을 본 과목의 개수)
A(과목 데이터 저장)

for(배열 길이만큼 반복) {
    점수 데이터 저장
}

for(배열 길이만큼 반복) {
    최고점 점수 판별하여 저장
    총점 계산
}

sum * 100 / 최고점 / 과목수 출력
```

### 4. 코드 구현하기

### [예제 코드](https://github.com/JeHeeYu/Book-Reviews/blob/main/Algorithm/Do%20it!%20%EC%95%8C%EA%B3%A0%EB%A6%AC%EC%A6%98%20%EC%BD%94%EB%94%A9%20%ED%85%8C%EC%8A%A4%ED%8A%B8%20C%2B%2B%20%ED%8E%B8/Chapter%202.%20%EC%9E%90%EB%A3%8C%EA%B5%AC%EC%A1%B0/%EB%B0%B0%EC%97%B4%2C%20%EB%A6%AC%EC%8A%A4%ED%8A%B8%2C%20%EB%B2%A1%ED%84%B0/1546.cpp)
