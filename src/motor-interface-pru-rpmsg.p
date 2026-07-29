;;; -*- asm -*-
;;; Remoteproc firmware variant: same motion core, but host
;;; notifications go through rpmsg instead of a bare INTC event.
;;; (Separate wrapper file because pasm has no -D command line option.)
#define NOTIFY_RPMSG 1
#include "motor-interface-pru.p"
