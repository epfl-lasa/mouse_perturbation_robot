# MouseMsg.msg

uint8 FM_NONE = 0
uint8 FM_BTN_A = 1
uint8 FM_BTN_B = 2
uint8 FM_LEFT_CLICK = 3
uint8 FM_RIGHT_CLICK = 4
uint8 FM_WHEEL = 5
uint8 FM_CURSOR = 6

uint8 event
int32 buttonState
int32 relX
int32 relY
int32 relZ
int32 relWheel
float32 filteredRelX
float32 filteredRelY
float32 filteredRelZ