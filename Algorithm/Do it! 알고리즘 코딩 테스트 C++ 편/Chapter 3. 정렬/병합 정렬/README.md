# 병합 정렬 정리 내용

## 병합 정렬(Merge Sort)
병합 정렬은 분할 정복(divide and conquer) 방식을 사용해 데이터를 분할하고 분할한 집합을 정렬하며 합치는 알고리즘이다.
<br>
병합 정렬의 시간 복잡도는 O(nlogn)이다.

## 병합 정렬의 핵심 이론

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/5344f2c5-d944-487f-bd09-7de14f2c81c6)


<br>


## 병합 정렬 수행 방식

위 그림에서 최초에는 8개의 그룹으로 나뉜다.
<br>
이 사앹에서 2개씩 그룹을 합치며 오름차순 정렬한다.
<br>
<br>
그 결과 (32, 42), (24, 60), (5, 15), (45, 90)이 된다.
<br>
이어서 2개씩 그룹을 합치며 다시 오름차순 정렬한다.
<br>
<br>
그 결과 (24, 32, 42, 60), (5, 15, 45, 90)이 된다.
<br>
<br>
이러한 방식으로 병합 정렬 과정을 거치면 (5, 15, 24, 32, 42, 45, 60, 90)이 되어 전체를 정렬할 수 있다.

<br>

## 2개의 그룹을 병합하는 과정

투 포인터 개념을 사용하여 왼쪽, 오른쪽 그룹을 병합한다.
<br>
왼쪽  포인터와 오른쪽 포인터의 값을 비교하여 작은 갑승ㄹ 결과 배열에 추가하고 포인터를 오른쪽으로 1칸 이동시킨다.


<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/dfc32b0f-2c3f-45e0-bef4-1b0e9d2aaeef)

<br>


## 문제 020 수 정렬하기 2 (실버 5, 2751)

[문제 링크](https://www.acmicpc.net/problem/2751)

<br>

### 1. 문제 분석하기
N의 최대 범위가 1,000,000이므로 O(nlogn)의 시간 복잡도로 정렬을 수행한다.

<br>


### 2. 손으로 풀어 보기

1. 정렬할 그룹을 최소 길이로 나눈다.<br>아래 그림과 같이 원본 배열 길이가 5이므로 2, 2, 1 길이로 나눈다.<br>이제 나눈 그룹마다 병합 정렬을 수행한다.<br>그룹마다 index1, index2를 지정하여 비교하면서 정렬 용도로 선언한 tmp 배열에 병합 정렬을 수행한다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/7b0a8390-3ad3-4be2-92de-4c4c3b8f9838)



<br>


여기서는 그룹이 3개이므로 2번째, 3번째 그룹을 병합한다.

<br>




<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/f79b0944-1206-4ba9-a564-fa0265274d4c)


<br>


2. 이어서 병합된 그룹을 대상으로 정렬한다.<br>index2의 경우 오른쪽으로 이동할 공간이 없으므로 index1만 이동하는 형태로 정렬한다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/afc38603-1a91-4afe-91a5-843101c1d9ac)


<br>


3. 마지막 정렬로 1, 2, 3, 4, 5 순서로 정렬이 끝난다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/e5736a87-7863-4438-a7b4-89b124e8d71e)


<br>


### 3. 슈도코드 작성하기

```
N(정렬할 수 개수)
A(정렬할 배열 선언)
tmp(정렬 시 잠시 사용할 임시 배열 선언)

for(N의 개수만큼) {
    배열 A에 데이터 저장
}

병합 정렬 함수 수행
결괏값 출력

// 병합 정렬 함수 구현 부분)
병합 정렬(s, e) {
    s(시작점), e(종료점), m(중간점)
    
    // 재귀 함수 형태로 구현
    병합 정렬(s, e)
    병합 정렬(m + 1, e)
    
    for(s ~ e) {
        임시 배열 tmp 저장
    }
    
    // 두 그룹을 병합하는 로직
    index1 -> 앞쪽 그룹 시작점
    index2 -> 뒤쪽 그룹 시작점
    
    while(index1 <= 중간점 && index2 <= 종료점)
        양쪽 그룹의 index가 가리키는 값을 비교하여 더 작은 수를 선여 배열에 저장하고 선택된 데이터의 index값을 한 칸 오른쪽으로 이동
        반복문이 끝난 후 남은 데이터 정리
}
```


<br>

### 4. 코드 구현하기

### [예제 코드](https://github.com/JeHeeYu/Book-Reviews/blob/main/Algorithm/Do%20it!%20%EC%95%8C%EA%B3%A0%EB%A6%AC%EC%A6%98%20%EC%BD%94%EB%94%A9%20%ED%85%8C%EC%8A%A4%ED%8A%B8%20C%2B%2B%20%ED%8E%B8/Chapter%203.%20%EC%A0%95%EB%A0%AC/%EB%B3%91%ED%95%A9%20%EC%A0%95%EB%A0%AC/2751.cpp)

<br>


## 문제 021. 버블 소트

[문제 링크](https://www.acmicpc.net/problem/1517)

<br>


### 1. 문제 분석하기

N의 최대 범위가 500,000이므로 O(nlogn)의 시간 복잡도로 정렬을 수행하면 된다.
<br>
<br>
이 문제의 제목은 버블 정렬이지만, N의 최대 범위가 500,000이므로 버블 정렬을 사용하면 제한 시간을 초과한다.
<br>
즉, 이 문제는 버블 정렬이 아닌 O(nlogn)의 시간 복잡도를 가진 병합 정렬을 사용해야 한다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/eea4e816-9bbf-40f6-abf0-113ab4423c28)



<br>

위 그림을 보면 두 그룹을 병합 정렬하는 과정에서 뒤쪽 그룹의 5가 앞쪽 그룹의 24, 43, 42, 60 앞에 놓인다.
<br>
이는 버블 정렬에서 swap을 4번 해야 볼 수 있는 효과이다.
<br>
<br>
아랫 그림도 마찬가지인데 45는 60보다 앞에 놓이므로 버블 정렬에서 swpa을 1번 한 것과 동일하다.

<br>


### 2. 손으로 풀어 보기

병합 정렬을 이용하나 정렬 과정에서 index가 이동한 거리를 result에 저장한다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/7342321b-5a67-4125-a7ed-8d2e41e7f8ba)


<br>

그림을 보면 그룹을 정렬하는 과정에서 원소가 앞으로 이동한 거리만큼 result에 더한다.
<br>
예를 들어 (3, 2), (8, 1), (7, 4), (5, 6) 그룹은 1, 2, 3 번째 그룹에서 2, 1, 4 원소만 이동하므로 result에 총 3을 더한다.
<br>
<br>

(2, 3, 1, 8), (4, 7, 5, 6) 그룹은 1이 2칸, 5, 6이 각각 1칸씩 이동하므로 reslut에 4를 더한다.
<br>
<br>
마지막으로 (1, 2, 3, 8, 4, 5, 6, 7)은 4, 5, 6, 7이 1칸씩 이동하므로 result에 4를 더한다.

<br>


### 3. 슈도코드 작성하기

```
N(정렬할 수 개수)
A(정렬할 배열 선언)
tmp(정렬 시 잠시 사용할 임시 배열 선언)

for(N의 개수만큼) {
    배열 A에 데이터 저장
}

병합 정렬 함수 수행
결괏값 출력

// 병합 정렬 함수 구현 부분)
병합 정렬(s, e) {
    s(시작점), e(종료점), m(중간점)
    
    // 재귀 함수 형태로 구현
    병합 정렬(s, e)
    병합 정렬(m + 1, e)
    
    for(s ~ e) {
        임시 배열 tmp 저장
    }
    
    // 두 그룹을 병합하는 로직
    index1 -> 앞쪽 그룹 시작점
    index2 -> 뒤쪽 그룹 시작점
    
    while(index1 <= 중간점 && index2 <= 종료점) {
    양쪽 그룹의 index가 가리키는 값을 비교하여 더 작은 수를 선택하여 배열에 저장하고 선택된 데이터의 index값을 한 칸 오른쪽으로 이동
        이때 뒤쪽 데이터 값이 더 작아 선택되는 경우 swap이 일어났다고 가정하여 현재 남아있는 앞쪽 데이터 개수만큼 결괏값에 더함
    }
        
        반복문이 끝난 후 남은 데이터 정리
}
```

### 4. 코드 구현하기

### [예제 코드](https://github.com/JeHeeYu/Book-Reviews/blob/main/Algorithm/Do%20it!%20%EC%95%8C%EA%B3%A0%EB%A6%AC%EC%A6%98%20%EC%BD%94%EB%94%A9%20%ED%85%8C%EC%8A%A4%ED%8A%B8%20C%2B%2B%20%ED%8E%B8/Chapter%203.%20%EC%A0%95%EB%A0%AC/%EB%B3%91%ED%95%A9%20%EC%A0%95%EB%A0%AC/1517.cpp)

