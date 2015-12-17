#include <stdio.h>
#include <string.h>
#include <signal.h>
#include "pswrap.h"

/** Initialize the speech engine
 *  @param info
 *    the information struct for the engine
 *  @return 0 on success, -1 otherwise
 */
int pswrap_init(pswrap_t *info) {
  printf("Model dir: %s\n", MODELDIR);
  info->config = cmd_ln_init(NULL, ps_args(), TRUE,
      "-hmm", MODELDIR "/en-us/en-us",
      "-lm", MODELDIR "/en-us/en-us.lm.dmp",
      "-dict", "custom.dict",      // custom dictionary
      NULL);
  if (!info->config) {
    fprintf(stderr, "[pswrap] Cannot init config.\n");
    goto error;
  }

  info->ps = ps_init(info->config);
  if (!info->ps) {
    fprintf(stderr, "[pswrap] Cannot init ps.\n");
    goto error;
  }

  return 0;

error:
  memset(info, 0, sizeof(pswrap_t));
  return -1;
}

/** Try to decipher a file
 *  @param info
 *    the information struct for the engine
 *  @return n characters deciphered on success,
 *    -1 otherwise
 */
int pswrap_decipher(pswrap_t *info, char *filename, char **buf) {
  FILE *fh;
  int rv;
  char *hyp;
  int score;

  // open file
  fh = fopen(filename, "rb");
  if (!fh) {
    fprintf(stderr, "[pswrap] Cannot find file: %s.\n", filename);
    return -1;
  }

  // get data
  rv = ps_decode_raw(info->ps, fh, -1);
  if (rv < 0) {
    fprintf(stderr, "[pswrap] Couldn't decode file: %s.\n", filename);
    return -1;
  }

  // decode
  hyp = (char *)ps_get_hyp(info->ps, &score);
  if (!hyp) {
    fprintf(stderr, "[pswrap] Cannot get hypothesis\n");
    return -1;
  }

  *buf = hyp;
  fclose(fh);
  return strlen(hyp);
}

/** Remove the speech engine
 *  @param info
 *    the information struct for the engine
 */
void pswrap_free(pswrap_t *info) {
  if (info->ps) {
    ps_free(info->ps);
    memset(info, 0, sizeof(pswrap_t));
  }
}
