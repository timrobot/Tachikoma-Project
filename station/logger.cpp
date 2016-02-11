#include <cstdio>
#include <mutex>
#include "logger.h"

#define DEFAULT_LOGFILE_NAME  "debuglog.txt"

using namespace std;
using namespace logger;

static FILE *logfile;
static mutex write_lock;

bool init(string fname) {
  if (logfile) {
    return false;
  }
  if (fname == "") {
    fname = DEFAULT_LOGFILE_NAME;
  }
  if (access(fname.c_str(), F_OK) != 0) {
    creat(fname.c_str(), O_WRONLY, O_CREAT | 0644);
  }
  logfile = fopen(fname.c_str(), "a+");
  return true;
}

void log(string tag, string msg) {
  if (!logfile) {
    init();
  }
  time_t t = time(0);
  struct tm *now = localtime(&t);
  write_lock.lock();
  fprintf(logfile, "{%04d-%02d-%02d.%02d:%02d:%02d} [%s] %s\n",
      now->tm_year,
      now->tm_mon,
      now->tm_mday,
      now->tm_hour,
      now->tm_min,
      now->tm_sec,
      tag.c_str(),
      log.c_str());
  write_lock.unlock();
}

void destroy(void) {
  fclose(logfile);
  logfile = NULL;
}
