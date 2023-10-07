#include <iostream>
#include <vector>

using uint = unsigned int;

class hash_map
{
    std::vector<int> data;
    
public:
    // 모든 원소를 -1로 초기화
    hash_map(size_t n)
    {
        data = std::vector<int>(n, -1);
    }
    
    // 삽입 함수
    void insert(uint value)
    {
        int n = data.size();
        data[value % n] = value;
        
        std::cout << value << "을(를) 삽입했습니다." << std::endl;
    }
    
    // 특정 원소가 있는지 확인하는 함수
    bool find(uint value)
    {
        int n = data.size();
        return (data[value % n] == value);
    }
    
    // 삭제 함수
    void erase(uint value)
    {
        int n = data.size();
        
        if(data[value % n] == value)
        {
            data[value % n] = -1;
            
            std::cout << value << "을(를) 삭제했습니다." << std::endl;
        }
    }
};

using namespace std;

int main()
{
    hash_map map(7);
    
    auto print = [&](int value) {
        if(map.find(value))
            std::cout << "해시 맵에서 " << value << "을(를) 찾았습니다.";
        else
            std::cout << "해시 맵에서 " << value << "을(를) 찾지 못했습니다.";
            
        std::cout << std::endl;
    };
    
    map.insert(2);
    map.insert(25);
    map.insert(10);
    print(25);
    
    map.insert(100);
    print(100);
    print(2);
    
    map.erase(25);

    return 0;
}
