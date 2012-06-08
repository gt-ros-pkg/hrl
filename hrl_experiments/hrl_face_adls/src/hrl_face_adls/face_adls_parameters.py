
CONTACT_FORCE_THRESH = 3.0
ACTIVITY_FORCE_THRESH = 3.0
DANGEROUS_FORCE_THRESH = 10.0

APPROACH_VELOCITY = 0.0025
GLOBAL_VELOCITY = 0.0025
HEIGHT_STEP = 0.17
LATITUDE_STEP = 0.12
LOCAL_VELOCITY = 0.0025
LONGITUDE_STEP = 0.06
RETREAT_HEIGHT = 1.65
SAFETY_RETREAT_HEIGHT = 1.9
SAFETY_RETREAT_VELOCITY = 0.0150
SLOW_RETREAT_VELOCITY = 0.0200
SHAVE_HEIGHT = 0.8
TIMEOUT_TIME = 30.0

outcomes_spa = ['succeeded','preempted','aborted']

#class TransitionIDs:
#    GLOBAL_START      =  1
#    GLOBAL_PREEMPT    =  2
#    GLOBAL_STOP       =  2
#    LOCAL_START       =  3 #TODO FIX
#    LOCAL_PREEMPT     =  2
#    LOCAL_STOP        =  4
#    SHAVE_START       =  8
#    MOVE_COLLISION        =  9
#    ELL_RETREAT_GLOBAL    = 10
#    ELL_MOVE_GLOBAL       = 11
#    ELL_APPROACH_GLOBAL   = 12
#    ELL_RETREAT_SLOW      = 13
#    ELL_RETREAT_FAST      = 14
#    HOLDING               = 15

class Messages:
    GLOBAL_START         = "Starting global movement %s."
    GLOBAL_PREEMPT_OTHER = "Preempted arm movement for global move %s."
    GLOBAL_PREEMPT       = "Global move %s preempted."
    GLOBAL_SUCCESS       = "Global move to pose %s successful."
    LOCAL_PREEMPT_OTHER  = "Preempted arm movement for local move."

