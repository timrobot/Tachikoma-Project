#ifndef __TK_LOGGER_H__
#define __TK_LOGGER_H__

#include <armadillo>

#define LOGGER_GRAY 1
#define LOGGER_RGB  2
#define LOGGER_ARGB 3

void logger_open(const char *fname);
void logger_log(const char *log);
void logger_disp(const arma::cube &image, int type);
void logger_close(void);

#endif
