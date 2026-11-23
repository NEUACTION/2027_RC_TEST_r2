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
				case WHEEL3:
				{
					GetDriverMsg(&wheel3Msg, msg_data);
					HB.wheel3 = 0;
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
				case WHEEL2:
				{
					GetDriverMsg(&wheel2Msg, msg_data);
					HB.wheel2 = 0;
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

	valve_num[12] = 1;//灯带常亮

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

	uint32_t cnt = 0;
	uint32_t colorcnt = 0;

	static int tim_cnt = 0;
	static bool reset_flag = false;

	for (;;)
	{
		vTaskDelayUntil(&last_wake, period);

		/* 这里写每 5ms 需要执行的逻辑 */

		// 心跳包自增
		HB_Add();

		// if(gRobot.runSucceedFlag)
		// {
		// 	// 复位
			if(!reset_flag)
			{
				SetDriverState(&FDCAN_TK, PTP_MODE, WHEEL2, ENABLE);
				SetDriverState(&FDCAN_WL, PTP_MODE, WHEEL3, ENABLE);
				reset_flag = 1;
				// reset_flag = reset_control();
				if (reset_flag)
				{
					BEEP_ON;
					HAL_Delay(500);
					BEEP_OFF;
				}
				continue;
			} // 复位完成后才执行正常控制逻辑

			// if (wheel2Msg.enableState && wheel3Msg.enableState)
			// {
				// static float dpos = 0.05f;

				// static float wheel2Pos = 0;
				// static float wheel3Pos = 0;

				static int tim_cnt1 = 0;
				
				tim_cnt1++;

				// if(tim_cnt1 == 1){
				// 	SetLimits(&FDCAN_WL, PTP_MODE, WHEEL2, 20 * 4096, -1, -1);
				// 	SetLimits(&FDCAN_WL, PTP_MODE, WHEEL3, 20 * 4096, -1, -1);
				// }
				if (tim_cnt1 >= 150)
				{

					wheel2Tar.pos = -wheelpos();
					wheel3Tar.pos = wheelpos();
					// VelCtrl(&FDCAN_WL, PTP_MODE, 2, (int32_t)(5 * 4096.f), 0, false);
					PosCtrl(&FDCAN_TK, PTP_MODE, WHEEL2, (int32_t)(wheel2Tar.pos * 4096.f), 6.505, 0.005);
					PosCtrl(&FDCAN_WL, PTP_MODE, WHEEL3, (int32_t)(wheel3Tar.pos * 4096.f), 6.505, 0.005);

				}
				// else
				// {
				// 	SetDriverState(&hfdcan3, PTP_MODE, WHEEL2, ENABLE);
				// 	SetDriverState(&hfdcan3, PTP_MODE, WHEEL3, ENABLE);
				// }
				

				DEBUG("%f, %f, %f, %f, %f, %f, %f, %f\r\n" , fabsf(wheel3Tar.pos), fabsf(wheel2Msg.pos), fabsf(wheel3Msg.pos), (fabsf(wheel2Msg.pos) - fabsf(wheel3Msg.pos)), wheel2Msg.currQ, wheel3Msg.currQ, wheel2Msg.vel, wheel3Msg.vel);
			// }
			/* 机器人控制 */
			// gRobot.reset_flag = 1;
			// Start_Info();
			// RobotControl();
		// }

		// RobotControl();

		// DEBUG("%d, %d, %d, %d\r\n", wheel1Tar.clean, wheel2Tar.clean, wheel3Tar.clean, wheel4Tar.clean);

		// static int cnt = 0;
		// if (cnt == 0)
		// {
		// 	SetDriverMode(&FDCAN_WL, PTP_MODE, LEG4, POS_MODE, (int32_t)(cnt * 4096.f));
		// }
		// cnt++;
		// SetStateAndPosCtrl(&FDCAN_WL, PTP_MODE, LEG4, ENABLE, (int32_t)(cnt * 4096.f));

		// valve_num[0] = 1;
		// valve_num[5] = 1;
		// SendValve();
		// valve_num[3] = 1;
		// static int valve_cnt = 0;
		// valve_cnt++;
		// if (valve_cnt < 25)
		// {
		// 	valve_num[9] = 1;
		// }
		// else if (valve_cnt < 50)
		// {
		// 	valve_num[9] = 0;
		// }
		// else
		// {
		// 	valve_cnt = 0;
		// }
		// SendValve();

		// MotorControl();

		// static int flag = 0;
		// if(flag == 0)
		// {
		// 	leg4Tar.mode = 0;
		// 	flag = 1;
		// }
		// else if (flag == 1)
		// {
		// 	leg4Tar.mode = 2;
		// 	flag = 0;
		// }

		// DEBUG("%d\r\n", leg4Msg.enableState);
		// DEBUG("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\r\n", ps_num[0], ps_num[1], ps_num[2], ps_num[3], ps_num[4], ps_num[5], ps_num[6], ps_num[7], ps_num[8], ps_num[9], ps_num[10], ps_num[11], ps_num[12], ps_num[13], ps_num[14], ps_num[15]);
		// DEBUG("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\r\n", ps_num_2[0], ps_num_2[1], ps_num_2[2], ps_num_2[3], ps_num_2[4], ps_num_2[5], ps_num_2[6], ps_num_2[7], ps_num_2[8], ps_num_2[9], ps_num_2[10], ps_num_2[11], ps_num_2[12], ps_num_2[13], ps_num_2[14], ps_num_2[15]);
		// DMEnable(&FDCAN_TK, ELBOW, DM_POS_MODE);
		// DMSetPosZero(&FDCAN_TK, ELBOW, DM_POS_MODE);

		// DMDisable(&FDCAN_TK, WRIST, DM_POS_MODE);
		// DMDisable(&FDCAN_TK, ELBOW, DM_POS_MODE);

		// static int flag = 0;
		// if(flag == 0)
		// {
		// 	SetDriverZero(&FDCAN_TK,PTP_MODE,SHOULDER);
		// 	flag = 1;
		// }
		// else
		// {
		// 	ReadPVT(&FDCAN_TK, PTP_MODE, SHOULDER);
		// }

		// DEBUG("%f, %f, %f\r\n", shoulderMsg.pos, elbowMsg.pos, wristMsg.pos);

		// static int tim_cnt = 0;
		// tim_cnt++;
		// if (tim_cnt <= 11)
		// {
		// 	valve_num[10] = 1;	
		// }
		// else if (tim_cnt <= 22)
		// {
		// 	valve_num[10] = 0;
		// }
		// else
		// {
		// 	tim_cnt = 0;
		// }
		// SendValve();

		// DMDisable(&FDCAN_TK, WRIST, DM_POS_MODE);

		// DEBUG("%f\r\n", wristMsg.pos);

		// __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 1500);

		// static int tim_cnt = 0;
		// if (tim_cnt == 0)	
		// {
		// 	// DMEnable(&FDCAN_TK, ELBOW, DM_POS_MODE);
		// 	DMEnable(&FDCAN_TK, WRIST, DM_POS_MODE);
		// 	tim_cnt = 1;
		// }
		// // else if (tim_cnt == 1)
		// // {
		// // 	// DMPosCtrl(&FDCAN_TK, ELBOW, 0, 0.5);
		// // 	DMPosCtrl(&FDCAN_TK, WRIST, 0, 0.5);
		// // 	if (fabs(elbowMsg.pos) < 0.05 && fabs(wristMsg.pos) < 0.05)
		// // 	{
		// // 		tim_cnt = 2;
		// // 	}
		// // }
		// // else
		// // {
		// // 	DMDisable(&FDCAN_TK, ELBOW, DM_POS_MODE);
		// // 	DMDisable(&FDCAN_TK, WRIST, DM_POS_MODE);
		// // }
		// else if (tim_cnt <= 10)
		// {
		// 	tim_cnt++;
		// 	// DMSetPosZero(&FDCAN_TK, ELBOW, DM_POS_MODE);
		// 	DMSetPosZero(&FDCAN_TK, WRIST, DM_POS_MODE);
		// }
		// else
		// {
		// 	DMDisable(&FDCAN_TK, ELBOW, DM_POS_MODE);
		// 	DMDisable(&FDCAN_TK, WRIST, DM_POS_MODE);
		// }

		/* 获取驱动器最新消息 */
		User_Control_GetDriverMsgsSnapshot(&driverMsgs);
		// SendPCMsg();

		cnt++;
		colorcnt++;
		cnt %= 200;
		colorcnt %= 1800;
	}
}
float wheelpos(void)
{
	//速度环 0.8 0.2
	const float tarvel = 20.0f;   // 速度 r/s
	const float s1     = 10.0f;   // 加速段行程 r
	const float s2     = 4.0f;   // 匀速段行程 r
	const float s3     = 4.0f;   // 减速段行程 r
	const float dt     = 0.005f; // 调用周期 5 ms                   
	const float t1 = 2.0f * s1 / tarvel;   // 加速时间
	const float t2 = 2.0f * s3 / tarvel;         // 匀速时间
	const float a1 = tarvel / t1;          // 加速度
	const float a2 = -tarvel / t2;         // 减速度
		
	static float s_ref = 0.0f;   // 累计参考位置 r
	static float vel   = 0.0f;   // 参考速度 r/s
	static int   stage = 0;      // 0=加速 1=匀速 2=减速 3=结束

	switch (stage)
	{
		case 0:                                  /* 匀加速 */
			vel += a1 * dt;
			if (vel >= tarvel)
			{
				vel   = tarvel;
					stage = 1;
			}
			break;

		case 1:       
			if (s_ref >= s1 + s2)
			{
				stage = 2;
			}
			break;

		case 2:                                  /* 匀减速 失能*/
			vel += a2 * dt;
			if (vel <= 0.0f)
			{
				vel   = 0.0f;
				stage = 3;
			}

		default:                                 /* 结束后保持不动 */
				break;
	}

    s_ref += vel * dt; // 累计参考位置
	return s_ref;
}
