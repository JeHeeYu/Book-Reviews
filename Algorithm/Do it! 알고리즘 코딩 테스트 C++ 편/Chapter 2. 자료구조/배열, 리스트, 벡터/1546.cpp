#include <iostream>

using namespace std;

int main()
{
    /*
        N : 시험 본 과목의 개수
        A : 시험 본 과목의 점수를 입력 받을 배열
        sum : 과목의 총 합
        max : 과목의 최고 점수
        result : 정답을 입력 받을 변수
    */
    int N = 0;
    int A[1000];
    long sum = 0;
    long max = 0;
    double result = 0;
    
    cin >> N;
    
    // 시험 본 과목의 개수 입력
    for(int i = 0; i < N; i++) {
        cin >> A[i];
    }
    
    for(int i = 0; i < N; i++) {
        // 최고 점수 계산을 위한 조건문
        if(A[i] > max) {
            max = A[i];
        }
        
        // 과목 총합 더하기
        sum += A[i];
    }
    
    // 총합 평균 계산
    result = sum * 100.0 / max / N;
    
    cout << result << '\n';

    return 0;
}
