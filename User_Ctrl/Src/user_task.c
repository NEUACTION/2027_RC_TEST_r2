#include "user_task.h"
#include "user_control.h"
#include "motor_driver.h"
#include "chassis_driver.h"
#include "user_usart.h"
#include "vofa.h"

/* 驱动器回报帧 StdId 规则：StdId = 0x280 + DriverNum */
static int Fdcan_ReportStdIdToDriverNum(uint32_t stdId, uint8_t *outDriverNum)
{
	if (outDriverNum == NULL)
	{
		return -1;
	}
	
	if (stdId == ELBOW || stdId == WRIST)
	{
		const uint32_t dn = stdId;
		if ((dn < 1U) || (dn > 64U))
		{
			return -1;
		}
		*outDriverNum = (uint8_t)dn;
		return 0;
	}
	else
	{
		const uint32_t dn = stdId - DRIVER_CLIENT_BASE_ID;
		if ((dn < 1U) || (dn > 64U))
		{
			return -1;
		}
		*outDriverNum = (uint8_t)dn;
		return 0;
	}
}

/* RXtask 上下文解析：FDCAN1 / FDCAN3 解析函数不同也没问题，在这里分发即可 */
static void User_FDCAN_RxTaskParse(const UserFDCAN_RxMsg_t *msg)
{
	if ((msg == NULL) || (msg->bus == NULL))
	{
		return;
	}

	uint8_t dn = 0U;
	if (Fdcan_ReportStdIdToDriverNum(msg->header.Identifier, &dn) != 0)
	{
		return;
	}

	int frame_num = msg->data_len / 8;
	for (int i = 0; i < frame_num; i++)
	{
		const uint8_t *msg_data = msg->data + i * 8;
		/* FDCAN1: 上层 CAN（UP_FDCAN）解析：TURN/ROLLER */
		if (msg->bus == &FDCAN_WL)
		{
			switch (dn)
			{
				case SCREW2:
				{
					GetDriverMsg(&screw2Msg, msg_data);
					HB.screw2 = 0;
					break;
				}
				default:
				{
					break;
				}
			}
		}

		/* FDCAN3: 轮子 CAN（WHEEL_FDCAN）解析：ONE/TWO/THR */
		if (msg->bus == &FDCAN_TK)
		{
			switch (dn)
			{
				case SCREW1:
				{
					GetDriverMsg(&screw1Msg, msg_data);
					HB.screw1 = 0;
					break;
				}
				default:
				{
					break;
				}
			}
		}
	}

}

osThreadId_t user_task = NULL;
TaskHandle_t user_task_handle = NULL;

/**
 * @brief 将毫秒转换为系统节拍数
 * @param timeout_ms 超时时间，单位毫秒
 * @retval TickType_t 对应的系统节拍数
 */
TickType_t ms_to_ticks(uint32_t timeout_ms)
{
	if (timeout_ms == 0U)
	{
		return 0;
	}
	if (timeout_ms == 0xFFFFFFFFU)
	{
		return portMAX_DELAY;
	}
	return pdMS_TO_TICKS(timeout_ms);
}

uint8_t Beep(uint32_t time)
{
	static uint8_t flag = 0;
	static uint32_t lastTime = 0;
	static uint32_t beep_Cnt = 0;
	
	if (lastTime != time) flag = 0;//如果跟上次不一样，重新计时

	lastTime = time;

	if (!flag)//判断是否为第一次进入
	{
		flag = 1;
		beep_Cnt = 0;//清零计时器
	}
	
	if (beep_Cnt < time) BEEP_ON;
	else if (beep_Cnt < time * 2) BEEP_OFF;
	else 
	{
		BEEP_OFF;
		flag = 0;//清零等待下一次进入
		return 1;//响完一轮返回1
	}

	beep_Cnt++;
	return 0;
}

void Start_Info(void)
{
	static uint8_t i = 0;
	if (i < 2 && gRobot.runSucceedFlag)
	{
		if (Beep(100) == 1) i++;		//定时器响两声表示上位机运行正常
	}
}

/**
 * @brief 初始化
 */
void User_Task_Init(void)
{
	/* UART8：TX 用 VOFA，RX 用 DMA+IDLE */
	(void)User_UART8_RxDmaStart();
	
	// FDCAN 初始化
	FDCAN_ENABLE();
	// 光电串口初始化
	USART_ENABLE();
	// USB通信初始化
	User_Communication_Init();

	// 注册FDCAN回调
	User_FDCAN_RegisterRxCallback(User_FDCAN_RxTaskParse);
	// 注册USB回调：在 UsbRxTask 上下文解析PC帧
	UsbRx_RegisterCallback(GetPCMsg);

	// 设置驱动器初始状态
	RobotInit();

	SendPs();

	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 1010);


	BEEP_ON;
	osDelay(500);
	BEEP_OFF;
	osDelay(2000);
	BEEP_ON;
	osDelay(500);
	BEEP_OFF;

	/* 开启主线程 */
	if ((user_task == NULL) && (user_task_handle == NULL))
	{
		const osThreadAttr_t userTask_attributes = {
			.name = "user_task",
			.stack_size = 1024U * 4U,
			.priority = osPriorityAboveNormal
		};
		user_task = osThreadNew(User_Task, NULL, &userTask_attributes);
	}
}
uint32_t Beep_Time = 0;
void SetBeep(uint32_t time){
    Beep_Time = time;
}
void Beep_Control(void)
{
    if(Beep_Time > 0)
    {
        BEEP_ON;
        Beep_Time --;
    }
    else
    {
        BEEP_OFF;
    }
}
/**
 * @brief 主任务
 * @param argument 任务参数
 */
void User_Task(void *argument)
{
	(void)argument;
	user_task_handle = xTaskGetCurrentTaskHandle();

	TickType_t last_wake = xTaskGetTickCount();
	const TickType_t period = pdMS_TO_TICKS(5);


	// static int tim_cnt = 0;
	static bool reset_flag = false;

	for (;;)
	{
		vTaskDelayUntil(&last_wake, period);

		/* 这里写每 5ms 需要执行的逻辑 */

		// 心跳包自增
		HB_Add();
		Beep_Control();
		// if(gRobot.runSucceedFlag)
		// 复位
			if(!reset_flag)
			{
				SetBeep(100);
				reset_flag = 1;
				continue;
			} // 复位完成后才执行正常控制逻辑
			static int cnt = 0;
			cnt++;
			if(cnt > 2)
			{
				lightmode();
				SendLight();
				cnt = 0;
			}
			MotorControl();
			//DEBUG("%f, %f, %f, %f, %f, %f, %f, %f, %f\r\n" , fabsf(screw2Tar.pos), fabsf(screw1Msg.pos), fabsf(screw2Msg.pos), (fabsf(screw1Msg.pos) - fabsf(screw2Msg.pos)), -screw1Msg.currQ, screw2Msg.currQ, -screw1Msg.currQ - screw2Msg.currQ, screw1Msg.vel, screw2Msg.vel);
		/* 获取驱动器最新消息 */
			User_Control_GetDriverMsgsSnapshot(&driverMsgs);
			SendPCMsg();

	}
}

 void lightmode(void)
{
    if(light_mode[0] == 0)
	{
		light_num[0] = 225;
		light_num[1] = 225;
		light_num[2] = 225;
	}
	else if(light_mode[0] == 1)
	{
		light_num[0] = 225;
		light_num[1] = 0;
		light_num[2] = 0;
	}
	else if(light_mode[0] == 2)
	{
		light_num[0] = 0;
		light_num[1] = 0;
		light_num[2] = 225;
	}
	else if(light_mode[0] == 3)
	{
		light_num[0] = 225;
		light_num[1] = 0;
		light_num[2] = 225;
	}
	else if(light_mode[0] == 4)
	{
		light_num[0] = 0;
		light_num[1] = 225;
		light_num[2] = 0;
	}
	

	 if(light_mode[1] == 0)
	{
		light_num[3] = 225;
		light_num[4] = 225;
		light_num[5] = 225;
	}
	else if(light_mode[1] == 1)
	{
		light_num[3] = 225;
		light_num[4] = 0;
		light_num[5] = 0;
	}
	else if(light_mode[1] == 2)
	{
		light_num[3] = 0;
		light_num[4] = 0;
		light_num[5] = 225;
	}
	else if(light_mode[1] == 3)
	{
		light_num[3] = 225;
		light_num[4] = 0;
		light_num[5] = 225;
	}
	else if(light_mode[1] == 4)
	{
		light_num[3] = 0;
		light_num[4] = 225;
		light_num[5] = 0;
	}
}
