# 선택 정렬 정리 내용

## 선택 정렬(Selection Sort)
선택 정렬은 대상 데이터에서 최대나 최소 데이터를 나열된 순으로 찾아가며 선택하는 방벙비다.
<br>
선택 정렬은 구현 방법이 복잡하고, 시간 복잡도도 O(n^2)으로 효율적이지 않아 많이 사용하지 않는다.

<br>

## 선택 정렬의 핵심 이론

최솟값 또는 최댓값을 찾고, 남은 정렬 부분의 가장 앞에 있는 데이터와 swap하는 것이 정렬의 핵심이다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/6b98e497-de22-4204-b609-70daba014ab9)


<br>


선택 정렬의 과정은 다음과 같다.

1. 남은 정렬 부분에서 최솟값 또는 최댓값을 찾음
2. 남은 정렬 부분에서 가장 앞에 있는 데이터와 선택된 데이터를 swap
3. 가장 앞에 있는 데이터의 위치를 변경해(index++) 남은 정렬 부분의 범위를 축소
4. 전체 데이터 크기만큼 index가 커질 때까지, 즉 남은 정렬 부분이 없을 때까지 반복

<br>

## 문제 017. 소트인사이드 (실버 5, 1427)

[문제 링크](https://www.acmicpc.net/problem/1427)


<br>

### 1. 문제 분석하기

자연수를 받아 자릿수별로 정렬하는 문제이므로 먼저 숫자를 자릿수별로 나누는 작업이 필요하다.
<br>
나머지 연산으로 분리할 수도 있지만 여기서는 입력값을 string으로 받은 후 substr() 함수를 이용해 자릿수 단위로 분리하고, 이를 다시 int형으로 변경해 배열에 저장한다.
<br>
<br>
그 후 단순하게 배열을 정렬한다.

<br>


### 2. 손으로 풀어보기

1. string 변수로 정렬할 데이터를 받아 int형 배열에 저장한다.<br>이때는 substr() 함수를 사용해 숫자를 자릿수별로 나눈 후 배열에 저장한다.

<br>


![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/574f3abe-a78d-4006-91fc-aa6c63b21cb7)


<br>


2. 배열의 데이터를 선택 정렬 알고리즘을 이용해 내림차순 정렬한다.<br>내림차순 정렬이므로 최댓값을 찾아 기준이 되는 자리와 swap한다.

<br>


![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/dc28fd73-752a-4fc3-a44c-e7114b0414d8)


<br>

### 3. 슈도코드 작성하기

```
str(정렬하고자 하는 수)
A(자릿수별로 구분하여 저장한 배열)

for(str의 길이만큼 반복) {
    배열 A 저장 -> str.substr() 사용
}

for(i: 0 ~ str 길이만큼 반복) {
    for(j: i + 1 ~ str 길이만큼 반복) {
        현재 범위에서 Max값 찾기
    }
    
    현재 i의 값과 Max값 중 Max 값이 더 크면 Swap 수행
}
```

<br>

### 4. 코드 구현하기

### [예제 코드](https://github.com/JeHeeYu/Book-Reviews/blob/main/Algorithm/Do%20it!%20%EC%95%8C%EA%B3%A0%EB%A6%AC%EC%A6%98%20%EC%BD%94%EB%94%A9%20%ED%85%8C%EC%8A%A4%ED%8A%B8%20C%2B%2B%20%ED%8E%B8/Chapter%203.%20%EC%A0%95%EB%A0%AC/%EC%84%A0%ED%83%9D%20%EC%A0%95%EB%A0%AC/1427.cpp)
