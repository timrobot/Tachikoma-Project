Notes:
================

Created by Timothy Yong (Rutgers University) 24 April 2015

Uses pocketsphinx to decipher hypotheses from gstreamer generated raw files.

The listen process spawns two recording processes whilst running decipher
and sending back hypotheses through a pipe to the main process stt.
