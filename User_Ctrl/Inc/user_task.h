#ifndef USER_TASK_H
#define USER_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "common.h"
#include "user_communication.h"
#include "user_FDCAN.h"

extern osThreadId_t user_task;
extern TaskHandle_t user_task_handle;

void User_Task_Init(void);
void User_Task(void *argument);
float wheelpos(void);

#ifdef __cplusplus
}
#endif

#endif /* USER_TASK_H */
