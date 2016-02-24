#ifndef TK_ASTAR_MAP_H
#define TK_ASTAR_MAP_H

#include <cstdint>

struct bitfield {
  uint8_t parent : 2;
  uint8_t opened : 1;
  uint8_t closed : 1;
};

struct rf_state {
  struct collision_byte prop;
  uint32_t g_value;
  uint32_t heap_position;
};

// Compressed map in order to navigate
class map() {
  public:
    map(int w, int h);
    ~map(void);
    bool collision(int x, int y);
};

#endif
