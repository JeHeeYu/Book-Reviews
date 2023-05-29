# 퀵 정렬 정리 내용

## 퀵 정렬(Quick Sort)
퀵 정렬은 기준값(Pivot)을 선정해 해당 값보다 작은 데이터와 큰 데이터로 분류하는 것을 반복해 정렬하는 알고리즘이다.
<br>
기준값이 어떻게 선정되는지가 시간 복잡도에 많은 영향을 미친다.
<br>
<br>
평균 시간 복잡도는 O(nlogn)이며 최악의 경우 시간 복잡도는 O(n^2)이다.

## 퀵 정렬의 핵심 이론
Pivot을 중심으로 계속 데이터를 2개의 집합으로 나누면서 정렬하는 것이 퀵 정렬의 핵심이다.

<br>


![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/b74b164d-c9c4-4c8a-ab6a-ab0112d04d7c)


<br>

퀵 정렬의 과정은 다음과 같다.

1. 데이터를 분할하는 pivot을 설정
2. pivot을 기준으로 다음 3 ~ 9 과정을 거쳐 데이터를 2개의 집합으로 분리한다.
3. start가 가리키는 데이터가 pivot이 가리키는 데이터보다 작으면 start를 오른쪽으로 1칸 이동
4. end가 가리키는 데이터가 pivot이 가리키는 데이터보다 크면 end를 왼쪽으로 1칸 이동
5. start가 가리키는 데이터가 pivot이 가리키는 데이터보다 크고, end가 가리키는 데이터가 pivot이 가리키는 데이터보다 작으면 start, end가 가리키는 데이터를 swap, start는 오른쪽으로 end는 왼쪽으로 1칸씩 이동
6. start와 end가 만날 때까지 3 ~ 5 과정 반복
7. start와 end가 만나면 만난 지점에서 가리키는 데이터와 pivot이 가리키는 데이터를 비교하여 pivot이 가리키는 데이터가 크면 만난 지점의 오른쪽에, 작으면 만난 시점의 왼쪽에 pivot이 가리키는 데이터 삽입
8. 분리 집합에서 각각 다시 pivot 선정
9. 분리 집합이1 개 이하가 될 때까지 1 ~ 8 과정 반복

<br>

## 문제 019. K번째 수 (실버 5, 11004)

[문제 링크](https://www.acmicpc.net/problem/11004)


<br>


### 1. 문제 분석하기

먼저 pivot을 정해야 하는데, 아래와 같은 규칙으로 정한다.

```
pivot == K : K번째 수를 찾은 것이므로 알고리즘 종료
pivot > K : pivot의 왼쪽 부분에 K가 있으므로 왼쪽(S ~ pivot - 1)만 정렬 수행
pivot < K : pivot의 오른쪽 부분에 K가 있으므로 오른쪽(pivot + 1 ~ E)만 정렬 수행
```

그리고 배열의 중간 위치를 pivot으로 설장한다.

<br>

### 2. 손으로 풀어 보기

1. 중간 위치를 pivot으로 설정한 다음 맨 앞에 있는 값과 swap한다.<br>pivot을 맨 앞으로 옮기는 이유는 i, j 이동을 편하게 하기 위함이다.<br>이후 i와 j를 pivot을 제외한 그룹에서 왼쪽, 오른쪽 끝으로 정한다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/d6da4aee-6343-466e-bacf-74b5498155c3)


<br>

2. 우선 j를 이동하는데, j번째 값이 pivot보다 크면 j-- 연산을 반복한다.<br>그 결과 j는 1에 위치하게 된다.<br>j를 이동한 후에는 i번째 값이 pivot보다 작으면서 i보다 j가 크면 i++ 연산을 반복한다.<br>지금은 i와 j의 위치가 같으므로 i는 이동하지 않는다.

<br>


![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/d9f5a566-a6ef-4f4c-9682-7cc1321c367f)


<br>

3. pivot을 두 집합으로 나눠 주는 위치, 즉 i와 j가 만난 위치와 swap 한다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/75fc96be-ea6d-499a-9def-3155049127af)

<br> 


4. K는 2이므로 이제 더 이상 정렬하지 않고 A[1]을 출력한다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/7e530108-f130-4dcc-b8b7-4a89b056f887)


<br>

### 3. 슈도코드 작성하기

```
N(숫자의 개수), K(K번째 수)
A(숫자 데이터 저장 배열)

for(N만큼 반복) {
    배열 A 저장
}

퀵 정렬 실행
K번째 데이터 출력

// 함수 구현 부분
퀵 정렬 함수(시작, 종료, K) {
    피벗 구하기 함수(시작 인덱스 종료 인덱스)
    if(피벗 == K) 종료
    else if(K < 피벗) 퀵 정렬 수행(시작 인덱스, 피벗 - 1, K)
    else 퀵 정렬 수행(피벗 + 1, 종료 인덱스, K)
}

피벗 구하기 함수(시작 인덱스, 종료 인덱스) {
    데이터가 2개인 경우는 바로 비교하여 정렬
    M(중앙값)
    중앙값을 시작 위치와 swap
    pivot을 시작 위치 값 A[S}로 저장
    i(시작점), j(종료점)
    
    while i <= j:
        피벗보다 큰 수가 나올 때까지 i 증가
        피벗보다 작은 수가 나올 때까지 j 감소
        찾은 i와 j 데이터를 swap
    피벗 데이터를 나눈 두 그룹의 경계 indexp에 저장
    경계 index 반환
}
```

### 4. 코드 구현하기

### [예제 코드](https://github.com/JeHeeYu/Book-Reviews/blob/main/Algorithm/Do%20it!%20%EC%95%8C%EA%B3%A0%EB%A6%AC%EC%A6%98%20%EC%BD%94%EB%94%A9%20%ED%85%8C%EC%8A%A4%ED%8A%B8%20C%2B%2B%20%ED%8E%B8/Chapter%203.%20%EC%A0%95%EB%A0%AC/%ED%80%B5%20%EC%A0%95%EB%A0%AC/11004.cpp)

