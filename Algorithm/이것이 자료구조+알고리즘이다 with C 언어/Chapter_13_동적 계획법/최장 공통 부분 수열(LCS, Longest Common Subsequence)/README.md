# 최장 공통 부분 수열

## 최장 공통 부분 수열(LCS, Longest Common Subsequence)
수학에서 수열(Sequence)은 어떤 물건이나 객체의 목록을 가리키는 용어이다.
<br>
그리고 이 수열(즉, 정렬되어 있는 객체의 목록)에서 일부 요소를 제거한 것을 부분 수열(Subsequence)라고 한다.
<br>
<br>
예를 들어 'GOOD MORNING'이 수열이라고 한다면, 'OMIG'는 이 수열의 부분 수열이라고 할 수 있다.
<br>
'OMIG'는 'GOOD MORNING' 에서 'G, O, D, O, R, N, N'을 제거한 결과이기 때문이다.
<br>
<br>
공통 부분 수열은 두 수열 사이에 공통적으로 존재하는 부분 수열을 말한다.
<br>
'GOOD MORNING'과 'GUTEN MORGEN'의 공통 부분 수열을 뽑으라면 'G', 'GO', 'GON', 'GMORN' 등이 나온다.
<br>
<br>
여기서 최장 공통 부분 수열은 여러 개의 공통 부분 수열 중 가장 긴 것을 말한다.
<br>
<br>
최장 공통 부분 수열(LSC) 알고리즘은 두 데이터를 비교할 때 아주 유용한 알고리즘이다.
<br>
두 텍스트 파일의 차이점을 찾아주는 UNIX의 diff 유틸리티도 이 알고리즘에 기반하여 만들어졌다.

<br>

## LCS 알고리즘
LCS 문제를 동적 계획벙브로 풀기 전에 먼저 이 문제가 최적 부분 구조로 이루어져 있는지 확인해야 한다.
<br>
이를 위해 X, Y 두 문자열이 있다고 가정하고 이 두 문자열을 매개 변수로 받아 이들 사이에서 나올 수 있는 LCS의 길이를 구하는 함수를 LCS_LENGTH() 라고 가정한다.
<br>
<br>
우선 i 또는 j 중 하나가 0이면 LCS_LENGTH(X(i), Y(j))의 결과는 0이 된다.
<br>
i 또는 j가 0이라면 두 문자열 중 하나의 길이가 0이라는 의미이고, 둘 중 하나의 길이가 0이면 공통 부분 수열이 존재하지 않기 때문이다.
<br>
<br>
두 번째 경우를 보면 x(i)와 y(j)가 같다면, 즉, X(i)와 Y(j)의 마지막 요소가 동일하다면 LCS_LENGTH(X(i), Y(j))는 LCS_LENGTH(X(i-1), Y(j-1)) + 1을 반환한다.
<br>
<br>
두 문자열의 동일한 마지막 요소는 당연히 LCS에 포함되므로 상수 1로 대체할 수 있기 때문이다.
<br>
<br>
그리고 마지막 요소를 뺀 두 문자열끼리의 LCS 길이를 구하여 상수 1과 더하면 원래 구하고자 했던 LCS 길이를 얻을 수 있다.
<br>
따라서 이 경우는 다음과 같은 식이 성립된다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/fe6a78d3-b73d-4620-a698-c53a09a914c0)


<br>

세 번째 경우를 보면, 두 문자열의 길이거 어느 한쪽도 0이 아니고 마지막 요소도 서로 다른 경우이다.
<br>
이 경우 LCS_LENGTH(X(i-1), Y(j))와 LCS_LENGTH(X(i), Y(j-1)) 중 큰 쪽이 LCS_LENGTH(X(i), Y(j))의 해이다.
<br>
<br>
이유는 두 문자열의 마지막 요소를 각각 x(i), y(j)라고 하면 두 요소끼리는 달라도 x(i)와 y(j-1)은 같을 수 있다.
<br>
또는 x(i-1)과 y(j)가 같을 수도 있다.
<br>
<br>
따라서 이 경우에는 LCS_LENGTH(X(i-1), Y(j))와 LCS_LENGTH(X(i), Y(j-1)) 중에서 큰 쪽이 LCS_LENGTH(X(i), Y(j))의 해가 되는 것이다.
<br>
<br>
이 세 가지 경우를 정리하면 다음과 같이 점화식을 만들 수 있다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/d26cc643-a694-4112-bf58-8251dddbbdcc)


<br>

이 점화식은 X와 Y의 LCS가 부분 LCS의 답으로부터 만들어지고 있음을 보여준다.
<br>
즉, 이 문제는 최적 부분 구조로 되어 있어서 동적 계획법으로 해를 구할 수 있다는 뜻이다.
<br>
<br>
한편 이 점화식은 다음과 같이 정리할 수도 있다.
<br>
이 점화식은 i x j의 테이블이 있다고 할 때 TABLE[i, j]는 LCS의 길이이며 TABLE의 각 요소는 TABLE[i, j]의 부분 문제들의 답을 담고 있다는 사실을 설명한다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/f395cd51-f2d9-4c29-913c-8d238e62a895)



<br>

여기서 한 가지 주의해야 할 점이 있는데, 이 점화식에 나오는 i = 0, j = 0은 C 언어에서의 배열 인덱스 처럼 첫 번째 요소를 가리키는 것이 아니라는 사실이다.
<br>
i = 0 또는 j = 0은 그야말로 '아무것도 없음'을 나타낸다.
<br>
문자열 X의 첫 번째 요소는 i = 1, 문자열 Y의 첫 번째 요소는 j = 1이다.
<br>
<br>
아래는 앞의 점화식을 그대로 C언어 코드로 옮겨 작성한 LCS() 함수이다.
<br>
매개 변수는 모두 5개이며, 첫 번째와 두 번째 매개 변수는 문자열 X와 Y, 세 번째와 네 번째 매개변수는 테이블의 마지막 인덱스, 다섯 번째는 LCS 테이블이다.
<br>
<br>
마지막 매개 변수인 LCS 테이블의 크기는 i x j가 아닌 (i+1) x (j+1)이다.
<br>
이 테이블이 점화식의 테이블보다 행과 열이 1씩 긴 더 이유는 '아무것도 없는 행(i=0)과 열(j=0)을 표현하기 위해서다.

```
typedef struct structLCSTable
{
    int** data;
} LCSTable;

int LCS(char* x, char* y, int i, int j, LCSTable* table)
{
    if(i == 0 || j == 0) {
        table->data[i][j] = 0;
        return table->data[i][j];
    }
    else if(x[i - 1] == y[j - 1]) {
        table->data[i][j] = LCS(x, y, i - 1, j - 1, table) + 1;
        return table->data[i][j];
    }
    else {
        int a = LCS(x, y, i - 1, j, table);
        int b = LCS(x, y, i, j - 1, table);
        
        if(a > b) {
            table->data[i][j] = a;
        }
        else {
            table->data[i][j] = b;
        }
        
        return table->data[i][j];
    }
}
```

앞의 함수가 두 문자열 'GOOD MORNING' 및 'GUTEN MORGEN'으로부터 얻어낸 LCS 테이블의 결과는 다음과 같다.
<br>
이 테이블의 오른쪽 아래 모서리에 있는 마지막 요소에는 두 문자열 사이의 LCS 길이가 저장되어 있다.
<br>
<br>
그리고 테이블 안쪽에는 X와 Y의 부분 문자열에 대한 LCS의 길이들로 채워져 있다.
<br>
예를 들어 'GOOD MO'와 'GUTON MO' 사이에 존재하는 LCS의 길이는 다음 테이블에서 4('G', ' ', 'M', 'O')라는 사실을 알 수 있다.

<br>

![image](https://github.com/JeHeeYu/Book-Reviews/assets/87363461/8229ee39-346a-4018-9b5a-e9c11d10abf4)

<br>

이 LCS() 함수는 이렇게 작성하면 수행 시간이 지속적으로 증가한다.
<br>
LCS() 함수는 자기 자신을 재귀 호출하는 분기문 블록을 두 군데나 갖고 있기 때문이다.
<br>
<br>
그 중 마지막 블록은 LCS() 함수를 두 번 재귀 호출한다.
<br>
이런 구조의 재귀 호출은 수행 시간이 길어지느 ㄴ단점이 있다.
<br>
하지만 동적 계획법을 이용한다면 이 알고리즘을 재구성하여 수행 시간을 O(nm) 수준으로 확 낮출 수 있다.

<br>

## 동적 계획법 기반 LCS 알고리즘
LCS 문제를 동적 계획법으로 풀 때 다음과 같이 동적 계획법 설계 절차를 따른다.























