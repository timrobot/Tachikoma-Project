#ifndef __TK_LOGGER_H__
#define __TK_LOGGER_H__

#include <sys/ipc.h>
#include <sys/shm.h>

namespace logger {

  bool init(std::string fname = "");
  void log(std::string tag, std::string msg);
  void destroy(void);

};

#endif
