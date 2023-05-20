# 투 포인터 정리 내용

## 문제 006 수들의 합 5 (실버5, 2018)

[문제 링크](https://www.acmicpc.net/problem/2018)

### 1. 문제 분석하기
이 문제는 시간 복잡도 분석으로 사용할 알고리즘의 범위부터 줄여야 한다.
<br>
우선 문제에 주어진 시간 제한은 2초이며 N의 최댓값은 10,000,000 으로 매우 크게 잡혀있다.
<br>
<br>
이런 상황에서는 O(nlogn)의 시간 복잡도 알고리즘을 사용하면 제한 시간을 초과하므로 O(n)의 시간 복잡도 알고리즘을 사용해야 한다.
<br>
<br>
이런 경우에서 자주 사용하는 알고리즘이 투 포인터 알고리즘이다.
<br>
연속된 자연수의 합을 구하는 것이 문제이므로 시작 인덱스와 종료 인덱스를 지정하여 연속된 수를 표현한다.

### 2. 손으로 풀어 보기

1. 입력받은 값을 N에 저장한 후 코드에서 사용할 변수를 모두 초기화한다.<br> 결과 변수 count를 1로 초기화하는 이유는 N이 15일 때 숫자 15만 뽑는 경우의 수를 미리 넣고 초기화했기 때문이다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/f0364791-5701-4381-8d29-86bd86376c55)


<br>


2. 다음에서 보이는 투 포인터 이동 원칙을 활용해 배열의 끝까지 탐색하면서 합이 N이 될 경우의 수를 구한다.<br>start_index를 오른쪽으로 한 칸 이동하는 것은 연속된 자연수에서 왼쪽 값을 삭제하는 것과 효과가 같다.<br>end_index를 오른쪽으로 한 칸 이동하는 것은 연속된 자연수의 범위를 한 칸 더 확장하는 의미이다.<br>sum과 N이 같을 때는 경우의 수를 1 증가시키고, end_index를 오른쪽으로 이동시킨다.

```
// 투 포인터 이동 원칙

sum > N : sum = sum - start_index; start_index++;
sum < N : end_index++; sum = sum + end_index;
sum == N : end_index++; sum = sum + end_index; count++;
```

3. 2단계를 end_index가 N이 될 때까지 반복하되, 포인터가 이동할 때마다 현재의 총합과 N을 비교해 값이 같으면 count를 1만큼 증가시키면 된다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/4cdab8f6-e199-4d7b-bcda-a9ddf7f5f1e3)


<br>

### 3. 슈도코드 작성하기

```
N(연속된 자연수의 합)
start_index = 1
end_index = 1
sum = 1 // 현재 연속된 합 값을 저장하는 변수

while(end_index != N) {
    if(sum == N) 경우의 수 증가, end_index 증가, sum 값 변경
    else if(sum > N) sum값 변경, start_index 증가
    else if(sum < N) end_index 증가, sum값 변경
}

경우의 수 출력
```



### 4. 코드 구현하기

[예제 코드](https://github.com/JeHeeYu/Book-Reviews/blob/main/Algorithm/Do%20it!%20%EC%95%8C%EA%B3%A0%EB%A6%AC%EC%A6%98%20%EC%BD%94%EB%94%A9%20%ED%85%8C%EC%8A%A4%ED%8A%B8%20C%2B%2B%20%ED%8E%B8/Chapter%202.%20%EC%9E%90%EB%A3%8C%EA%B5%AC%EC%A1%B0/%ED%88%AC%20%ED%8F%AC%EC%9D%B8%ED%84%B0/2018.cpp)



<br>


## 문제 007 주몽 (실버4, 1940)

[문제 링크](https://www.acmicpc.net/problem/1940)

### 1. 문제 분석하기
우선 시간 복잡도를 고려해 보면 두 재료의 번호의 합, 즉 크기를 비교하므로 값을 정렬하면 더 문제를 쉽게 풀 수 있다.
<br>
N의 최대 범위가 15,000 이므로 O(nlogn) 시간 복잡도 알고리즘을 사용해도 문제가 없다.
<br>
<br>
일반적으로 정렬 알고리즘의 시간 복잡도는 O(nlogn)으로 정렬을 사용해도 상관 없다.

<br>

### 2. 손으로 풀어 보기

1.  먼저 재료 데이터를 배열 A[N]에 저장한 후 오름차순 정렬한다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/89459059-8b3a-4448-a50b-d675b2c037c1)


<br>

2. 투 포인터 i, j를 양쪽 끝에 위치시킨 후 문제의 조건에 적합한 포인터 이동 원칙을 활용해 탐색을 수행한다.

```
// 투 포인터 이동 원칙

A[i] + A[j] > M : j--; // 번호의 합이 M보다 크므로 큰 번호 index를 내린다.
A[i] + A[j] < M : i++; // 번호의 합이 M보다 작으므로 작은 번호 index를 올린다.
A[i] + A[j] == M : i++; j--; count++; // 양쪽 포인터를 모두 이동시키고 count를 증가시킨다.
```

3. 2단계를 i와 j가 만날 때까지 반복하고 반복이 끝나면 count를 출력한다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/71541eee-7484-4edb-a608-a9747be4b4c5)



<br>


### 3. 슈도코드 작성하기

```
N(재료의 개수), M(갑옷이 되는 번호)
A(재료 데이터 저장 배열)

for(N만큼 반복) {
    재료 배열 저장
}

재료 배열 정렬

while(i < j) {
    if(재료 합 < M) 작은 번호 재료를 한 칸 뒤로 변경
    else if(재료 합 > M) 큰 번호 재료를 한 칸 앞으로 변경
    else 경우의 수 증가, 양쪽 index 각각 변경
}

경우의 수 출력
```


<br>


### 4. 코드 구현하기

[예제 코드](https://github.com/JeHeeYu/Algorithm/blob/main/%EB%B0%B1%EC%A4%80/Silver/1940.%E2%80%85%EC%A3%BC%EB%AA%BD/%EC%A3%BC%EB%AA%BD.cc)

<br>

## 문제 008 좋다 (골드5, 1253)

[문제 링크](https://www.acmicpc.net/problem/1253)

<br>

### 1. 문제 분석하기
시간 복잡도 부터 보면 N의 개수가 최대 2,000이라 가정해도 좋은 수 하나를 찾는 알고리즘의 시간 복잡도는 N^2보다 작아야 한다.
<br>
만약 좋은 수 하나를 찾는 데 시간 복잡도가 N^2인 알고리즘을 사용하면 최종 시간 복잡도는 N^3이 되어 제한 시간 안에 문제를 풀 수 없기 때문이다.
<br>
<br>
따라서 좋은 수 하나를 찾는 알고리즘의 시간 복잡도는 최소 O(nlogn)이어야 한다.
<br>
<br>
정렬과 투 포인터 알고리즘을 사용할 수 있는데, 단 절열된 데이터에서 자기 자신을 좋은 수 만들기에 포함하면 안된다.

<br>

### 2. 손으로 풀어 보기

1. 수를 입력받아 배열에 저장한 후 정렬한다

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/853b8a4d-db9f-4a08-ae4a-7f680b35b300)


<br>

2. 투 포인터 i, j를 배열 A 양쪽 끝에 위치시키고 조건에 적합한 투 포인터 이동 원칙을 활용해 탐색을 수행한다.<br>판별의 대상이 되는 수는 K라고 가정한다.

```
// 투 포인터 이동 원칙

A[i] + A[j] > K : j--; A[i] + A[j] < K : i++;
A[i] + A[j] == K : count++; 프로세스 종료
```

3. 2단계를 배열의 모든 수에 대하여 반복한다.<br>즉, K가 N이 될 때까지 반복하며 좋은 수가 몇 개인지 센다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/7e6a67d7-3b9b-45ac-a61a-e6309c74570e)

<br>

### 3. 슈도코드 작성하기

```
N(배열의 데이터 개수)
A(데이터 저장 배열)

for(N만큼 반복) {
    배열 A에 데이터 저장
}

배열 A 정렬하기

for(N만큼 반복) {
    변수 초기화(찾고자 하는 값 find = A[k], 포인터 i, 포인터 j)
    while(i < j) {
        if(A[i] + A[j] == find)
            두 포인터 i j가 k가 아닐 경우 결괏값에 반영 및 while 문 종료
            두 포인터 i j 가 k가 맞을 경우 포인터 변경 및 계속 수행
        else if(A[i] + A[j] < find) 포인터 i 증가
        else 포인터 j 감소
    }
}
```

<br>

### 4. 코드 구현하기

[예제 코드](https://github.com/JeHeeYu/Book-Reviews/blob/main/Algorithm/Do%20it!%20%EC%95%8C%EA%B3%A0%EB%A6%AC%EC%A6%98%20%EC%BD%94%EB%94%A9%20%ED%85%8C%EC%8A%A4%ED%8A%B8%20C%2B%2B%20%ED%8E%B8/Chapter%202.%20%EC%9E%90%EB%A3%8C%EA%B5%AC%EC%A1%B0/%ED%88%AC%20%ED%8F%AC%EC%9D%B8%ED%84%B0/1253.cpp)
