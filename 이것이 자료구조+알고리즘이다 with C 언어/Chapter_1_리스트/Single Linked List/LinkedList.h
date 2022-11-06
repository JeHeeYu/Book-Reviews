#ifndef LINKEDLIST_H
#define LINKEDLIST_H

#include <stdio.h>
#include <stdlib.h>

typedef int ElementType;

// 노드 구조체 정의
typedef struct tagNode
{
    ElementType Data;
    struct tagNode* NextNode;
} Node;

// 함수 원형
Node* SLL_CreateNode(ElementType NewData);          // 노드 생성
void SLL_DestoryNode(Node* Node);                   // 노드 소멸
void SLL_AppendNode(Node** Head, Node* NewHead);    // 노드 추가
void SLL_InsertAfter(Node* Current, Node* NewNode); // 노드 삽입
void SLL_InsertNewHead(Node** Head, Node* NewHead); // 헤드 삽입
void SLL_RemoveNode(Node** Head, Node* Remove);     // 노드 삭제
Node* SLL_GetNodeAt(Node* Head, int Location);      // 노드 탐색
int SLL_GetNodeCount(Node* Head);                   // 노드 개수

#endif
