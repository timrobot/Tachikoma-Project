#include <flite.h>
#include "tts.h"

// TODO; DOXY COMMENTS

static int flite_initialized;
static int voice_selected;
static cst_voice *voice;
static void tts_init(void);

register_cmu_us_kal();

static void tts_init(void) {
  if (!flite_initialized) {
    flite_init();
    flite_initialized = 1;
  }
}

int tts_select_voice(const char *voicename) {
  // ignore all queries
  if (!flite_initialized) {
    tts_init();
  }
  voice = register_cmu_us_kal(NULL);
  voice_selected = 1;
  return voice ? 0 : -1;
}

int tts_say(const char *msg) {
  if (!voice_selected) {
    tts_select_voice(NULL);
  }
  flite_text_to_speech(msg, voice, "play");
}
