#ifndef COMMAND_PROCESSING_H
#define COMMAND_PROCESSING_H

typedef enum {
    CMD_DIR_STOP = 0,
    CMD_DIR_FRONT,
    CMD_DIR_BACK,
    CMD_DIR_LEFT,
    CMD_DIR_RIGHT,
    CMD_DIR_FRONT_LEFT,
    CMD_DIR_FRONT_RIGHT,
    CMD_DIR_BACK_LEFT,
    CMD_DIR_BACK_RIGHT
} command_direction_e;

void CommandProcessing_init(void);
void CommandProcessing_switchSpeedSetting(bool bSwitchSpeed);
float CommandProcessing_modifySetpointInRpm(void);
int CommandProcessing_manageDirection(void);

void CommandProcessing_getDirection(int iDirectionX, int iDirectionY);

#endif // COMMAND_PROCESSING_H