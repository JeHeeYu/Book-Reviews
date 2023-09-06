#include <iostream>
#include <forward_list>
#include <vector>

int main()
{
    std::vector<std::string> vec = {
        "Lewis Hamilton", "Lewis Hamilton", "Nico Roseberg",
        "Sebastian Vettel", "Lewis Hamilton", "Sebastian Vettel",
        "Sevastian Vettel", "Sebastian Veteel", "Fernando Alonso"
    };
    
    // 상수 시간
    auto it = vec.begin();
    
    std::cout << "가장 최근 우승자: " << *it << std::endl;
    
    // 상수 시간
    it += 8;
    
    std::cout << "8년 전 우승자: " << *it << std::endl;
    
    // 상수 시간
    advance(it, -3);
    
    std::cout << "그후 3년 뒤 우승자: " << *it << std::endl;
    
    std::forward_list<std::string> fwd(vec.begin(), vec.end());
    
    auto it1 = fwd.begin();
    
    std::cout << "가장 최근 우승자: " << *it1 << std::endl;
    
    // 선형 시간
    advance(it1, 5);
    
    std::cout << "5년 전 우승자: " << *it1 << std::endl;

    return 0;
}
