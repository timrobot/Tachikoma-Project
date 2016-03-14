#include "heap.h"

#define parent(index) (((index)-1)/2)
#define lchild(index) (((index)*2)+1)
#define rchild(index) (((index)*2)+2)

heap::heap(void) {
}

heap::~heap(void) {
}

void heap::swap(int a, int b) {
  state *temp = queue[a];
  queue[a] = queue[b];
  queue[b] = temp;
}

void heap::siftup(void) {
  assert(!queue.empty());
  int curr = queue.size() - 1;
  while (curr != 0) {
    if (*queue[curr] < *queue[parent(curr)]) {
      swap(curr, parent(curr));
      curr = parent(curr);
    } else {
      break;
    }
  }
}

void heap::siftdown(void) {
  int curr = 0;
  int target = lchild(curr);
  while (target < queue.size()) {
    // compare left and right children
    if (rchild(curr) < queue.size() &&
      *queue[rchild(curr)] < *queue[lchild(curr)]) {
      target = rchild(curr);
    }
    // compare target and current node
    if (*queue[target] < *queue[curr]) {
      swap(target, curr);
      curr = target;
      target = lchild(curr);
    } else {
      break;
    }
  }
}

void heap::push(state *item) {
  queue.push_back(&item);
  siftup();
}

state *heap::pop(void) {
  if (queue.empty()) {
    return NULL;
  } else {
    // take the first item off
    state *s = queue[0];
    swap(0, queue.size()-1);
    queue.pop_back();
    // sift the nodes down
    siftdown();
    return s;
  }
}

bool heap::empty(void) {
  return queue.empty();
}
