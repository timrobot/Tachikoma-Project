#ifndef svec_h
#define svec_h

#include <cstdlib>
#include "state.h"

class svec {
  public:
    svec(int initsize = 0);
    ~svec(void);

    state *&operator[](int index);
    state **data; // pointer to integers

    state **begin();
    const state **cbegin();
    state **end();
    const state **cend();

    size_t size(void);
    bool empty(void);
    void clear(void);
    void erase(int index); // SLOW

    void push_back(state *item);
    void pop_back(void);

    size_t _size;
    size_t max_size;
};

#endif
