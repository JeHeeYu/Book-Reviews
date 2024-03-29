# 구간 합 정리 내용

## 구간 합 핵심 이론
구간 합 알고리즘을 활용하기 위해 먼저 합 배열을 구해야 한다.
<br>
합 배열은 특정 범위에서 배열의 합을 말한다.

```
// 합 배열 S 정의

S[i] = A[0] + A[1] + A[2] + ... + A[i -1] + Ai]     // A[0]부터 A[i]까지의 합
```

합 배열은 기존의 배열을 전처리한 배열이라고 생각하면 편하다.
<br>
이렇게 합 배열을 미리 구해 놓으면 기존 배열의 일정 범위의 합을 구하는 <b>시간 복잡도가 O(N)에서 O(1)로 감소한다.</b>
<br>
<br>
예를 들어 아래와 같은 배열이 있다.

<br>

![image](https://user-images.githubusercontent.com/87363461/229328343-daf04fcd-b131-451e-b545-bf0dfafde7ae.png)

<br>

A[i]부터 A[j]까지의 배열 합을 합 배열 없이 구할 경우, 최악의 경우는 i가 0, j가 N의 경우로 시간 복잡도는 O(N)이 된다.
<br>
이런 경우 합 배열을 이용하게 된다면 O(1)안에 답을 구할 수 있다.
<br>
<br>
합 배열은 다음과 같이 간단한 공식으로 구할 수 있다.

```
// 합 배열 S를 구하는 공식

S[i] = S[i - 1] + A[i]
```

이렇게 구현된 합 배열을 이용하여 구간 합도 쉽게 구할 수 있다.
<br>
i에서 j까지의 구간 합을 구하는 공식은 다음과 같다.

```
// 구간 합을 구하는 공식

S[j] - S[i - 1]     // i에서 j까지의 구간 합
```

### 구간 합 공식 구하는 방법
먼저 아래와 같은 배열이 있으며, 배열 A[2]부터 A[5] 까지의 구간 합을 합 배열을 통해 구하는 과정이다.

<br>

![image](https://user-images.githubusercontent.com/87363461/229328430-564597de-c3c4-41fc-85ee-5a79e3388f0e.png)

<br>

그림을 보면 합 배열과 구간 합이 연관되어 있는 것을 볼 수 있다.
<br>
A[0] + ... A[5]에서 A[0] + A[1]을 빼면 구간 합 A[2] + ... + A[5]가 나오므로, S[5]에서 S[1]을 빼면 구간 합을 쉽게 구할 수 있다.
<br>
<br>
이렇게 합 배열만 미리 구해 두면 구간 합은 한 번의 계산으로 구할 수 있다.

```
// A[2] ~ A[5] 구간 합을 합 배열로 구하는 과정

S[5] = A[0] + A[1] + A[2] + A[3] + A[4] + A[5]
S[1] = A[0] + A[1]
S[5] - S[1] = A[2] + A[3] + A[4] + A[5]
```

<br>

## 문제 003 구간 합 구하기 4 (실버 3, 11659)

### [문제 링크](https://www.acmicpc.net/problem/11659)

### 1. 문제 분석하기
문제의 수의 개수와 합을 구해야 하는 횟수는 최대 100,000 회이다.
<br>
여기서 모든 구간마다 합을 매번 계산하면 0.5초 안에 모든 구간 합 계산을 끝낼 수 없다.
> 구간 합을 매번 계산할 경우 최악의 경우 1억 회 이상의 연산을 수행하게 되어 1초 이상의 수행 시간이 필요함

<br>
이럴 때 바로 구간 합을 이용해야 한다.

### 2. 손으로 풀어보기

1. N개의 수를 입력 받으면서 동시에 합 배열을 생성한다.
> 합 배열 공식 : S[i] = S[i - 1] + A[i]

![image](https://user-images.githubusercontent.com/87363461/229328858-d6d31d74-9b97-4819-92ef-bdfb75c2b6c5.png)

2. 구간 i ~ j가 주어지면 구간 합을 구하는 공식으로 정답을 출력한다.
> 구간 합 공식 : S[j] - S[i - 1]

```
// 문제 질의

질의 1(1, 3) : S[3] - S[0] = 12 - 0 = 12
질의 2(2, 4) : S[4] - S[1] = 14 - 5 = 9
질의 3(5, 5) : S[5] - S[4] = 15 - 14 = 1
```

### 3. 슈도코드 작성하기

```
suNo(숫자 개수), quizNo(질의 개수), S(합 배열)

for(숫자 개수만큼 반복) { 
    합 배열 생성(S[i] = S[i - 1] + A)
}

for(질의 개수만큼 반복) {
    질의 범위 받기(i ~ j)
    부분 합 출력(S[j] - S[i - 1]_
}
```

### 4. 코드 구현하기

### [예제 코드](https://github.com/JeHeeYu/Book-Reviews/blob/main/Algorithm/Do%20it!%20%EC%95%8C%EA%B3%A0%EB%A6%AC%EC%A6%98%20%EC%BD%94%EB%94%A9%20%ED%85%8C%EC%8A%A4%ED%8A%B8%20C%2B%2B%20%ED%8E%B8/Chapter%202.%20%EC%9E%90%EB%A3%8C%EA%B5%AC%EC%A1%B0/%EA%B5%AC%EA%B0%84%20%ED%95%A9/11659.cpp)


<br>


## 문제 004 구간 합 구하기 5 (실버 1, 11660)

### [문제 링크](https://www.acmicpc.net/problem/11660)

### 1. 문제 분석하기
먼저 질의의 개수가 100,000이므로 질의마다 합을 구할 수 없어 구간 합 배열을 이용해야 한다.
<br>
이 문제는 구간 합 배열이 2차원으로 확장된 문제이다.
<br>
<br>
먼저 2차원 구간 합 배열을 구하기 위해 아래와 같이 정의할 수 있다.

```
/// 2차원 구간 합 배열 D[X][Y] 정의

D[X][Y] = 원본 배열의 (0, 0)부터 (X, Y)까지의 사각형 영역 안에 있는 수의 합
```

### 2. 손으로 풀어 보기

1. 2차원 구간 합 배열의 1행, 1열부터 구해야 한다.

![image](https://user-images.githubusercontent.com/87363461/230532839-ae70ed7d-bbd6-4ee7-8440-1540a0a9230b.png)

2. 이를 통해 나머지 2차원 구간 합 배열을 채운다.

```
// D[i][j]의 값을 채우는 구간 합 공식

D[i][j] = D[i][j - 1] + D[i - 1][j] - D[i - 1][j - 1] + A[i][j]
```

![image](https://user-images.githubusercontent.com/87363461/230532922-0e8d70ef-4cd5-4bd1-935f-f9290b95098f.png)

3. 원본 배열에 구간 합을 표시하여 질의에 대한 답을 도출한다.

![image](https://user-images.githubusercontent.com/87363461/230533007-7b0d8b07-e450-4d0d-96b8-c8c0cc2adb0b.png)

예를 들어, 질의가 2 2 3 4 라면 (3, 4)구간 합에서 (1, 4)구간 합, (3, 1) 구간 합을 뺀 다음 중복하여 뺀 (1, 1)구간 합을 더하면 된다.
<br>
원본 배열에 표시한 구간 합을 다시 구간 합 배열에 표시하면 다음과 같다.

![image](https://user-images.githubusercontent.com/87363461/230533092-a69639ae-feb8-4421-8ea3-555f4d4ca952.png)

즉, 질의에 대한 답을 구하는 공식은 다음과 같다.

```
// 질의 X(1), Y(1), X(2), Y(2)에 대한 답을 구간 합으로 구하는 방법

D[X2][Y2] - D[X1 - 1][Y2] - D[X2][Y - 1] + D[X1 - 1][Y1 - 1]
```

### 3. 슈도코드 작성하기

```
N(배열 크기), M(질의 수)
A(원본 배열), D(합 배열)

for(N만큼 반복) {
    for(N만큼 반복) {
        원본 배열 저장
    }
}

for(N만큼 반복) {
    for(N만큼 반복) {
        합 배열 저장
        D[i][j] = D[i][j - 1] + D[i - 1][j] - D[i - 1][j - 1] + A[i][j];
    }
}

for(M만큼 반복) {
    질의 계산 및 출력하기
    결과 = D[x2][y2] - D[x1 - 1][y2] - D[x2][y1 - 1] + D[x1 - 1][y1 - 1];
}
```

### 4. 코드 구현하기

### [예제 코드](https://github.com/JeHeeYu/Book-Reviews/blob/main/Algorithm/Do%20it!%20%EC%95%8C%EA%B3%A0%EB%A6%AC%EC%A6%98%20%EC%BD%94%EB%94%A9%20%ED%85%8C%EC%8A%A4%ED%8A%B8%20C%2B%2B%20%ED%8E%B8/Chapter%202.%20%EC%9E%90%EB%A3%8C%EA%B5%AC%EC%A1%B0/%EA%B5%AC%EA%B0%84%20%ED%95%A9/11660.cpp)


<br>


## 문제 005 구간 합 구하기 5 (실버 1, 11660)

### [문제 링크](https://www.acmicpc.net/problem/10986)

### 1. 문제 분석하기
N의 최댓값이 10 ^ 6이라 연산량이 적게 느껴질 수 있으나, 10 ^ 6개의 수에 대하여 모든 구간 합을 구해야 하므로 1초 안에 연산하기는 어렵다.
<br>
여기서도 구간 합 배열을 이용해야 한다.

#### 문제의 핵심 아이디어
- (A + B) % C는 ((A % C) + (B % C)) % C와 같음
- 다시 말해 특정 구간 수들의 나머지 연산을 더해 나머지 연산을 한 값과 이 구간 합의 나머지 연산을 한 값은 동일함
- 구간 합 배열을 이용한 식 S[j] - S[i]는 원본 배열의 i + 1부터 j까지의 구간 합임
- S[j] % M의 값과 S[i] % M의 값이 같다면 (S[j] - S[i]) % M은 0임
- 즉, 구간 합 배열의 원소를 M으로 나눈 나머지로 업데이트하고 S[j]와 S[i]가 같은 (i, j) 쌍을 찾으면 원본 배열에서 i + 1부터 j까지의 구간 합이 M으로 나누어떨어진다는 것을 알 수 있음

### 2. 손으로 풀어 보기

1. 먼저 배열 A의 합 배열 S를 생성한다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/ee861fc3-b2fa-471e-8837-3f2e9958e8d2)


<br>

2. 합 배열 S의 모든 값을 M(3)으로 나머지 연산을 수행해 값을 업데이트 한다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/222b50cf-5590-4b52-95af-e2830f97608a)

<br>

3. 우선 변경된 합 배열에서 원소 값이 0인 개수만 세어 정답에 더한다. 변경된 합 배열의 원소 값이 0이라는 뜻은 원본 배열의 0부터 i까지의 구간 합이 이미 M으로 나누어떨어진 다는 뜻이기 때문이다.

```
경우의 수 = + 3
```

<br>

4. 이제 변경된 합 배열에서 원소 값이 같은 인덱스의 개수, 즉, 나머지 값이 같은 합 배열의 개수를 센다. 변경된 합 배열에서 원소 값이 같은 2개의 원소를 뽑는 모든 경우의 수를 구하여 정답에 더하면 된다.
<br>
<br>
위의 예에서는 0이 3개, 1이 2개이므로 3C2, 2C2로 경우의 수를 구하여 더하면 된다.

```
3C2 = 3 -> 경우의 수 = + 3
2C2 = 1 -> 경우의 수 = + 1
```

<br>

### 3. 슈도코드 작성하기

```
N(수열의 개수), M(나누어떨어져야 하는 수)
S(합 배열), C(같은 나머지를 가지는 인덱스를 카운트하는 배열)

for(i -> 1 ~ N) {
    S[i] = S[i - 1] + A[i] // 합 배열 저장
}

for(i -> 0 ~ N) {
    remainder = S[i] % M // 합 배열을 M으로 나눈 나머지 값
    if(remainder == 0) 정답을 1 증가시킴
    C[remainder]의 값을 1 증가시킴
}

for(i -> 0 ~ N) {
    C[i](i를 나머지로 가지는 인덱스의 개수)에서 2가지를 뽑는 경우의 수를 정답에 더하기
    // C[i]개 중에 2개를 뽑는 경우의 수 계산 공식 -> C[i] * (C[i] - 1) / 2
}

결괏값(anser) 출력
```

<br>

### 4. 코드 

### [예제 코드](https://github.com/JeHeeYu/Book-Reviews/blob/main/Algorithm/Do%20it!%20%EC%95%8C%EA%B3%A0%EB%A6%AC%EC%A6%98%20%EC%BD%94%EB%94%A9%20%ED%85%8C%EC%8A%A4%ED%8A%B8%20C%2B%2B%20%ED%8E%B8/Chapter%202.%20%EC%9E%90%EB%A3%8C%EA%B5%AC%EC%A1%B0/%EA%B5%AC%EA%B0%84%20%ED%95%A9/10986.cpp)
