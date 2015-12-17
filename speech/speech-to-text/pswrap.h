#ifndef __TK_PSWRAP_H__
#define __TK_PSWRAP_H__

#include <pocketsphinx.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct pswrap {
  ps_decoder_t *ps;
  cmd_ln_t *config;
} pswrap_t;

int pswrap_init(pswrap_t *info);
int pswrap_decipher(pswrap_t *info, char *filename, char **buf);
void pswrap_free(pswrap_t *info);

#ifdef __cplusplus
}
#endif

#endif
