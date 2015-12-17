#include <stdlib.h>
#include "stt.h"
#include "tts.h"
#include "speech.h"

static char readbuf[128];

/** Start the speech engine
 *  @return 0 on success, -1 otherwise
 */
int speech::start(void) {
  return stt_start_listening();
}

/** Stop the speech engine
 */
void speech::stop(void) {
  stt_stop_listening();
}

/** Listen for a speech phrase
 *  @return a phrase if found, otherwise NULL
 */
char *speech::listen(void) {
  if (stt_listen(readbuf) == 0) {
    readbuf[0] = '\0';
    return NULL;
  } else {
    return readbuf;
  }
}

/** Say something to the speakers
 *  @param fmt
 *    the message to say
 */
void speech::say(const char *fmt) {
  // TODO: account for var arg in later on
  tts_say(fmt);
}
