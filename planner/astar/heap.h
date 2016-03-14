#ifndef heap_h
#define heap_h

#include <vector>
#include <stdexcept>

template <class T>
class heap {
  public:
    heap(void);
    ~heap(void);
    void siftup(void);
    void siftdown(void);
    void push(T const &item, int priority);
    T pop(void);
    bool empty(void) const;
    size_t size(void) const;

  private:
    void swap(int const &a, int const &b);
    std::vector<T> queue;
    std::vector<int> priorities;
    int parent(int index) const;
    int lchild(int index) const;
    int rchild(int index) const;
};

#endif
