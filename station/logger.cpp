#include <cstdio>
#include "logger.h"

#define DEFAULT_LOGFILE_NAME  "debuglog.txt"

static int initialized;
static FILE *logfile;

void logger_open(const char *fname) {
  if (!initialized) {
    // append to the file
    logfile = fopen(fname, "a");
    initialized = 1;
  }
}

void logger_log(const char *log) {
  if (!initialized) {
    logger_init(DEFAULT_LOGFILE_NAME);
  }
  fprintf(logfile, "%s", log);
}

void logger_close(void) {
  if (initialized) {
    fclose(logfile);
    logfile = NULL;
    initialized = 0;
  }
}
