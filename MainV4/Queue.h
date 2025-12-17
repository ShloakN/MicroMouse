#include <stdlib.h>
#include <Array.h>
#ifndef QUEUELIB
#define QUEUELIB

class Queue{

public:
  struct Node {
    int row;
    int col;
    Array<char, 150> path;
    Node* next;
  }; 

  Node* front = nullptr;
  Node* rear  = nullptr;

public:

  void enQueue(int row = -1, int col = -1, Array<char, 150> path = {}, char dir = 'X'){

    Node* node = (Node*)malloc(sizeof(Node));
    if (node == nullptr) {
        Serial.println("Queue Memory allocation failed!");
        return;
    }
    node->row = row;
    node->col = col;

    node->path = path;//.fill(path);//copy path!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    node->path.push_back(dir);
    node->next = nullptr;

    if(rear == nullptr){
      front = rear = node;
    } else{
      rear->next = node;
      rear = node;
    }
  }

  Node* deQueue(){
    if(front == nullptr){
      //Serial.println("Queue Empty");
      return nullptr;
    }
    
    Node* temp = front;
    front = front->next;
    if (front == nullptr) {
        rear = nullptr;
    }
    return temp;
    
  }

  bool isEmpty(){
    return (front == nullptr);
  }

  void freeNode(Node* node){
    free(node);
  }

  void clearQueue() {
    while (front != nullptr) {
        Node* temp = front;
        front = front->next;
        free(temp);  // Free each node's memory
    }
    rear = nullptr;
  }


};

class PriorityQueue{

public:
  struct Node {
    int row;
    int col;
    int cost;
    Array<char, 150> path;
    Node* next;
  }; 

  Node* front = nullptr;
  Node* rear  = nullptr;

public:

  void enQueue(int row = -1, int col = -1, Array<char, 150> path = {}, char dir = 'X', int cost = 0){

    Node* node = (Node*)malloc(sizeof(Node));
    if (node == nullptr) {
        Serial.println("Queue Memory allocation failed!");
        return;
    }
    node->row = row;
    node->col = col;
    node->path = path;//.fill(path);//copy path!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    node->path.push_back(dir);
    node->cost = cost + node->path.size();
    node->next = nullptr;

    if(rear == nullptr){
      front = rear = node;
      //Serial.print("EnQ: ");
      //display();
      return;
    } 
    if(node->cost < front->cost){
      node->next = front;
      front = node;
      //Serial.print("EnQ: ");
      //display();
      return;
    }

    Node* prev = nullptr;
    Node* curr = front;

    while(curr != nullptr && curr->cost <= node->cost){

      prev = curr;
      curr = curr->next; 
    }
    prev->next = node;
    node->next = curr;
    
    if (node->next == nullptr) {
      rear = node;
    }
    //Serial.print("EnQ: ");
    //display();

  }

  Node* deQueue(){
    if(front == nullptr){
      ////Serial.println("Queue Empty");
      //Serial.print("DeQ: ");
      //display();
      return nullptr;
    }
    
    Node* temp = front;
    front = front->next;
    if (front == nullptr) {
        rear = nullptr;
    }
    //Serial.print("DeQ: ");
    //display();
    return temp;
  }

  bool isEmpty(){
    return (front == nullptr);
  }

  void freeNode(Node* node){
    free(node);
  }

  void clearQueue() {
    while (front != nullptr) {
        Node* temp = front;
        front = front->next;
        free(temp);  // Free each node's memory
    }
    rear = nullptr;
  }

  void display(){
    Node* ptr = front;
    while(ptr != nullptr){
      Serial.print(ptr->cost);
      Serial.print(", ");
      ptr = ptr->next;
    }
    Serial.println();
  }


};


#endif