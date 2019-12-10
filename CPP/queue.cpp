#include <iostream> 
#include <queue> 
using namespace std; 

typedef int Value;

class Node {
	Value _value;
	Node *_next;
public:
	Node(const Value v) { _value = v; _next = NULL; }
	Value value(){ return _value; }
	Node *next(){ return _next; }
	void link(Node *n){ _next = n; }
};

class LinkedList {
	Node *_front;
	Node *_back;
	unsigned int _size;
public:
	LinkedList() { _front = NULL; _back = NULL; _size = 0; }
	Node *front() { return _front; }
	Node *back() { return _back; }
	bool empty() { return _size==0; }
	unsigned int size() { return _size; }
	void push (const Value value) {
		Node *temp = new Node(value);
		if(_back != NULL){
			_back->link(temp);
			_back = _back->next();
		}
		else
			_front = _back = temp;
		_size++;
	}
	void pop() {
		Node *temp = _front;
		_front = temp->next();
		delete temp;
		_size--;
	}
};
  
int main() { 
    // Empty Queue 
    queue<int> myqueue; 
    myqueue.push(0); 
    myqueue.push(1); 
    myqueue.push(2); 
    // queue becomes 0, 1, 2 
  
    // Printing content of queue 
	cout << "Queue" << endl;
    while (!myqueue.empty()) { 
        cout << ' ' << myqueue.front(); 
        myqueue.pop(); 
    } 
	cout << endl << endl;

	LinkedList list;
	list.push(0);
	list.push(1);
	list.push(2);

	cout << "LinkedList without pop" << endl;
	for(Node *node=list.front(); node!=NULL; node=node->next()){
        cout << ' ' << node->value();
	}
	cout << endl << endl;

	cout << "LinkedList with pop" << endl;
    while (!list.empty()) { 
        cout << ' ' << list.front()->value(); 
        list.pop(); 
    } 
	cout << endl << endl;
} 
