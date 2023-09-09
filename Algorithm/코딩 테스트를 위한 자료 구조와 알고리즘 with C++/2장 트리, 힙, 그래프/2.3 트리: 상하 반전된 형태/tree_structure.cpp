#include <iostream>
#include <queue>

// 한 직원은 최대 부하 직원 두 명을 거느릴 수 있음
// 최대 두 개의 직원을 거느릴 수 있는 이진 트리 형태
struct node
{
    std::string position;   // 조직도상 직책
    node* first;            // 첫 번째 직원
    node* second;           // 두 번째 직원
};

// 트리 구조
struct org_tree
{
    node* root;
    
        // 새로운 트리를 만드는 정적 함수
    static org_tree create_org_structure(const std::string& pos)
    {
        org_tree tree;
        tree.root = new node {pos, NULL, NULL};
        
        return tree;
    }
    
    // 특정 직책을 찾는 함수
    static node* find(node* root, const std::string& value)
    {
        if(root == NULL) {
            return NULL;
        }
        
        // 가장 먼저 루트 검사
        if(root->position == value) {
            return root;
        }
        
        // 루트 노드가 아닐 경우 왼쪽 서브 트리 검사
        auto firstFound = org_tree::find(root->first, value);
        
        if(firstFound != NULL) {
            return firstFound;
        }
        
        // 왼쪽이 아닐 경우 오른쪽 서브 트리 검사
        return org_tree::find(root->second, value);
    }
    
    // 새로운 원소(부하 직원)을 추가하는 삽입 함수
    bool addSubordinate(const std::string& manager, const std::string& subordinate)
    {
        auto managerNode = org_tree::find(root, manager);
        
        if(!managerNode) {
            std::cout << manager << "을(를) 찾을 수 없습니다" << std::endl;
            
            return false;
        }
        
        // 최대 두 명까지 거느릴 수 있으나 두 명이 모두 찬 경우 실패
        if(managerNode->first && managerNode->second) {
            std::cout << manager << " 아래에 " << subordinate << "을(를) 추가할 수 없습니다." << std::endl;
        
            return false;
        }
        
        // 첫 번째가 비었으면 첫 번째에, 아닐 경우 두 번째에 부하 직원 추가
        if(!managerNode->first) {
            managerNode->first = new node {subordinate, NULL, NULL};
        }
        else {
            managerNode->second = new node {subordinate, NULL, NULL};
        }
        
        std::cout << manager << " 아래에 " << subordinate << "(을)를 추가했습니다." << std::endl;
    
        return true;
    }
};



int main()
{
    auto tree = org_tree::create_org_structure("CEO");
    
    tree.addSubordinate("CEO", "부사장");
    tree.addSubordinate("부사장", "IT부장");
    tree.addSubordinate("부사장", "마케팅부장");
    tree.addSubordinate("IT부장", "보안팀장");
    tree.addSubordinate("IT부장", "앱개발팀장");
    tree.addSubordinate("마케팅부장", "물류팀장");
    tree.addSubordinate("마케팅부장", "홍보팀장");
    tree.addSubordinate("부사장", "재무부장");

    return 0;
}
