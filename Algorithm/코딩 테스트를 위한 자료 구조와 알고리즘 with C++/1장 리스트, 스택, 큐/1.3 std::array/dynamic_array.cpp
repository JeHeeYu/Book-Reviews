#include <iostream>
#include <sstream>
#include <algorithm>

template <typename T>
class dynamic_array
{
    T* data;
    size_t n;
    
public:
    // 배열 크기를 인자로 받는 생성자
    dynamic_array(int n)
    {
        this->n = n;
        data = new T[n];
    }
    
    // 복사 생성자
    dynamic_array(const dynamic_array<T>& other)
    {
        n = other.n;
        data = new T[n];
        
        for(int i = 0; i < n; i++) {
            data[i] = other[i];
        }
    }
    
    // 멤버 데이터에 접근하기 위한 [] 연산자
    // [] 연산자를 제공함으로써 std::array와 비슷한 방식으로 배열 원소 접근 가능
    T& operator[](int index)
    {
        return data[index];
    }
    
    const T& operator[](int index) const
    {
        return data[index];
    }
    
    // 멤버 데이터에 접근하기 위한 at 함수
    T& at(int index)
    {
        if(index < n)
            return data[index];
        
        throw "Index out of range";
    }
    
    // 배열의 크기를 반환하는 함수
    size_t size() const
    {
        return n;
    }
    
    // 소멸자
    ~dynamic_array()
    {
        delete[] data;
    }
    
    // 배열 원소를 순회하는 반복자 함수 정의
    T* begin() { return data; }
    const T* begin() const { return data; }
    T* end() { return data + n; }
    const T* end() const { return data + n; }
    
    // 두 배열을 하나로 합치는 연산을 수행하는 + 연산자 함수 정의
    friend dynamic_array<T> operator+(const dynamic_array<T>& arr1, dynamic_array<T>& arr2)
    {
        dynamic_array<T> result(arr1.size() + arr2.size());
        std::copy(arr1.begin(), arr1.end(), result.begin());
        std::copy(arr2.begin(), arr2.end(), result.begin() + arr1.size());
        
        return result;
    }
    
    // 배열에 저장된 모든 데이터를 문자열로 변환하는 함수
    // 데이터 구분을 위한 문자열 sep을 인자로 받으며 sep의 기본 값은 쉼표로 지정
    std::string to_string(const std::string& sep = ", ")
    {
        if(n == 0)
            return "";
            
        std::ostringstream os;
        os << data[0];
        
        for(int i = 1; i < n; i++)
            os << sep << data[i];
            
        return os.str();
    }
};

struct student
{
    std::string name;
    int standard;
};

std:: ostream& operator<<(std::ostream& os, const student& s)
{
    return (os << "[" << s.name << ", " << s.standard << "]");
}

int main()
{
    int nStudents;
    
    std::cout << "1반 학생 수를 입력하세요.";
    std::cin >> nStudents;
    
    dynamic_array<student> class1(nStudents);
    
    for(int i = 0; i < nStudents; i++)
    {
        std::string name;
        int standard;
        std::cout << i + 1 << "번째 학생 이름과 나이를 입력하세요.";
        std::cin >> name >> standard;
        class1[i] = student{name, standard};
    }
    
    // 배열 크기보다 큰 인덱스의 학생에 접근
    try
    {
        // 예외 발생
        class1.at(nStudents) = student{"John", 8};
    }
    catch (...)
    {
        std::cout << "예외 발생!" << std::endl;
    }
    
    // 깊은 복사
    auto class2 = class1;
    std::cout << "1반을 복사하여 2반 생성:" << class2.to_string() << std::endl;
    
    // 두 학급을 합쳐서 새로운 큰 학급을 생성
    auto class3 = class1 + class2;
    std::cout << "1반과 2반을 합쳐 3반 생성: " << class3.to_string() << std::endl;

    return 0;
}
