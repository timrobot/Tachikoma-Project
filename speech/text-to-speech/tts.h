#ifndef __TK_TTS_H__
#define __TK_TTS_H__

#ifdef __cplusplus
extern "C" {
#endif

int tts_select_voice(const char *voicename);
int tts_say(const char *msg);

#ifdef __cplusplus
}
#endif

#endif
