#include "svec.h"
#include <cstring>

svec::svec(int initsize) :
  _size(0),
  max_size(0),
  data(NULL) {
  if (initsize > 0) {
    _size = initsize;
    max_size = initsize * 2;
    data = new state *[max_size];
  }
}

svec::~svec() {
  clear();
}

state *&svec::operator[](int index) {
  return data[index];
}

state **svec::begin() {
  return &data[0];
}

state **svec::end() {
  return &data[_size];
}

size_t svec::size(void) {
  return _size;
}

bool svec::empty(void) {
  return _size == 0;
}

void svec::clear(void) {
  if (data) {
    delete[] data;
    data = NULL;
  }
  _size = 0;
  max_size = 0;
}

void svec::erase(int index) {
  memmove(&data[index], &data[index+1], sizeof(state *) * _size-index-1);
  _size--;
  if (_size == 0) {
    clear();
  }
}

void svec::push_back(state *item) {
  if (max_size <= _size + 1) {
    size_t old_size = max_size;
    max_size = (max_size == 0) ? 1 : max_size * 2;
    state **newdat = new state *[max_size];
    if (old_size > 0) {
      memcpy(newdat, data, sizeof(state *) * old_size);
      delete[] data;
    }
    data = newdat;
  }
  data[_size++] = item;
}

void svec::pop_back(void) {
  _size--;
}
