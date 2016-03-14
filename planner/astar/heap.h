#ifndef heap_h
#define heap_h

#include "svec.h"

class heap {
  public:
    heap(void);
    ~heap(void);
    void siftup(void);
    void siftdown(void);
    void push(state *item);
    state *pop(void);
    bool empty(void);

  private:
    void swap(int a, int b);
    svec queue;
};

#endif
