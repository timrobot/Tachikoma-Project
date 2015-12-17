#include <festival.h>
#include "tts.h"

static bool festival_initialized;
static bool voice_selected;
static void tts_init(void);

/** Helper method to initialize the text to speech lib
 */
static void tts_init(void) {
  if (!festival_initialized) {
    festival_initialize(1, 4200000);
    festival_initialized = true;
  }
}

/** Select a voice to use for the output
 *  @param voicename
 *    the name of the voice to use
 *  @return 0 on success, -1 otherwise
 */
int tts_select_voice(const char *voicename) {
  char voice_selection[64];
  int res;
  if (!festival_initialized) {
    tts_init();
  }
  sprintf(voice_selection, "(%s)", voicename);
  res = festival_eval_command((EST_String)voice_selection);
  voice_selected = true;
  return res;
}

/** Say a message
 *  @param msg
 *    the message to say
 *  @return 0
 */
int tts_say(const char *msg) {
  if (!voice_selected) {
//    tts_select_voice("voice_cmu_us_slt_arctic_hts");
    tts_select_voice("voice_kal_diphone");
  }
  festival_say_text(msg);
  festival_wait_for_spooler();
  return 0;
}
