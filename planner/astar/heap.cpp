#include "heap.h"
#include <stdexcept>

using namespace std;

template <class T>
heap<T>::heap(void) {
}

template <class T>
heap<T>::~heap(void) {
}

template <class T>
void heap<T>::swap(int const &a, int const &b) {
  T temp = this->queue[a];
  this->queue[a] = this->queue[b];
  this->queue[b] = temp;
  int p = this->priorities[a];
  this->priorities[a] = this->priorities[b];
  this->priorities[b] = p;
}

template <class T>
void heap<T>::siftup(void) {
  if (this->queue.empty()) {
    throw std::out_of_range("heap<>::siftup(): empty heap");
  }
  int curr = this->queue.size() - 1;
  while (curr != 0) {
    if (this->priorities[curr] < this->priorities[parent(curr)]) {
      swap(curr, parent(curr));
      curr = parent(curr);
    } else {
      break;
    }
  }
}

template <class T>
void heap<T>::siftdown(void) {
  int curr = 0;
  int target = lchild(curr);
  while (target < this->queue.size()) {
    // compare left and right children
    if (rchild(curr) < this->queue.size() &&
      this->priorities[rchild(curr)] < this->priorities[lchild(curr)]) {
      target = rchild(curr);
    }
    // compare target and current node
    if (this->priorities[target] < this->priorities[curr]) {
      swap(target, curr);
      curr = target;
      target = lchild(curr);
    } else {
      break;
    }
  }
}

template <class T>
void heap<T>::push(T const &item, int priority) {
  this->queue.push_back(item);
  this->priorities.push_back(priority);
  this->siftup();
}

template <class T>
T heap<T>::pop(void) {
  if (this->queue.empty()) {
    throw std::out_of_range("heap<>::pop(): empty heap");
  } else {
    // take the first item off
    T s = this->queue[0];
    swap(0, this->queue.size()-1);
    this->queue.pop_back();
    this->priorities.pop_back();
    // sift the nodes down
    this->siftdown();
    return s;
  }
}

template <class T>
bool heap<T>::empty(void) const {
  return this->queue.empty();
}

template <class T>
size_t heap<T>::size(void) const {
  return this->queue.size();
}

template <class T>
int heap<T>::parent(int index) const {
  return (index - 1) / 2;
}

template <class T>
int heap<T>::lchild(int index) const {
  return (index * 2) + 1;
}

template <class T>
int heap<T>::rchild(int index) const {
  return (index * 2) + 2;
}
