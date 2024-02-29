#pragma once

#include "core/pif_task.h"

#include "drivers/display.h"

#include "common/time.h"

extern bool cmsInMenu;

// Device management
bool cmsDisplayPortRegister(displayPort_t *pDisplay);

// For main.c and scheduler
void cmsInit(void);
uint16_t cmsHandler(PifTask *p_task);

long cmsMenuChange(displayPort_t *pPort, const void *ptr);
long cmsMenuExit(displayPort_t *pPort, const void *ptr);
void cmsUpdate(uint32_t currentTimeUs);

#define CMS_STARTUP_HELP_TEXT1 "MENU: THR MID"
#define CMS_STARTUP_HELP_TEXT2     "+ YAW LEFT"
#define CMS_STARTUP_HELP_TEXT3     "+ PITCH UP"

// cmsMenuExit special ptr values
#define CMS_EXIT             (0)
#define CMS_EXIT_SAVE        (1)
#define CMS_EXIT_SAVEREBOOT  (2)
