#ifndef USER_CONTROL_H
#define USER_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "common.h"
#include "motor_driver.h"
#include "chassis_driver.h"
#include "user_FDCAN.h"
#include "user_communication.h"

//各电机ID宏定义
#define SCREW1         (1)
#define SCREW2         (2)

#define LEG1			 (3)
#define LEG2			 (4)

#define ELBOW			 (6)
#define WRIST			 (7)


//上下层对应的can号
#define FDCAN_WL        (hfdcan3)
#define FDCAN_TK     	(hfdcan1)

// 电机期望
typedef struct{
    uint8_t status;
    uint8_t mode;
	uint8_t last_mode;
	bool clean;
	float last_vel_kp;
	float last_vel_ki;
	float vel;
	float pos;
	float tor;
	float mit_kp;
	float mit_kd;
    float pos_kp;
    float pos_kd;
	float vel_kp;
	float vel_ki;
}motorTarget_t;

// 心跳包
typedef struct{
	uint8_t screw1;
	uint8_t screw2;
	uint8_t wheel1;
	uint8_t wheel2;
	uint8_t wheel3;
	uint8_t wheel4;
	uint8_t leg1;
	uint8_t leg2;
	uint8_t PC;
	uint8_t op;
}Heart_t;

typedef struct
{
    bool runSucceedFlag;
	uint8_t periodCntPC;
	bool reset_flag;

}	gRobot_t;

extern Heart_t HB;

extern gRobot_t gRobot;

extern driverMsg_t screw1Msg, screw2Msg, leg1Msg,leg2Msg;

extern motorTarget_t screw1Tar, screw2Tar, leg1Tar, leg2Tar;

extern SteeringWheelTar_t wheel1Tar, wheel2Tar, wheel3Tar, wheel4Tar;
extern SteeringWheelMsg_t wheel1Msg, wheel2Msg, wheel3Msg, wheel4Msg;

extern int esc;

/* ------------------------------------------------------------
 * Driver message snapshot
 * - 在任务上下文调用
 * - 内部用临界区一次性拷贝，避免读到“半更新”的结构体
 * ------------------------------------------------------------ */
typedef struct
{
	SteeringWheelMsg_t wheel1Msg;
	SteeringWheelMsg_t wheel2Msg;
	SteeringWheelMsg_t wheel3Msg;
	SteeringWheelMsg_t wheel4Msg;
	driverMsg_t screw1Msg;
	driverMsg_t screw2Msg;
	driverMsg_t leg1Msg;
	driverMsg_t leg2Msg;
	uint32_t tick;
} DriverMsgsSnapshot_t;
extern DriverMsgsSnapshot_t driverMsgs;

void User_Control_GetDriverMsgsSnapshot(DriverMsgsSnapshot_t *out);

// 常用控制函数
void RobotInit(void);
void GPIOControl(void);
void DMControl(FDCAN_HandleTypeDef *hfdcan, uint32_t driverNum, motorTarget_t *driverTarget);
void DriverControl(FDCAN_HandleTypeDef *hfdcan, uint32_t driverNum, motorTarget_t *driverTarget, driverMsg_t *driverMsg);
void MotorControl(void);
void ChassisControl(void);
void ErrorStop(void);
void DisableAll(void);
void EscControl(void);
bool reset_control(void);
void DriverStateControl(void);
void RobotControl(void);

#ifdef __cplusplus
}
#endif

#endif //  USER_CONTROL_H