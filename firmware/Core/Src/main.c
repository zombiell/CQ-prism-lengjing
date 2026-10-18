/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ps2.h"
#include "motor.h"
#include "chassis.h"
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* ---------- 手柄状态 ---------- */
static ps2_state_t g_ps2;

/* ---------- 控制参数 ----------
 * 本手柄摇杆无模拟量，只能用方向按键做数字量控制
 */
#define CTRL_PERIOD_MS      10u    /* 控制周期 10ms，即 100Hz */
#define PS2_PERIOD_MS       10u    /* 手柄扫描周期 10ms */
#define MOVE_SPEED          0.50f  /* 平移速度（0~1） */
#define TURN_SPEED          0.50f  /* 自转速度（0~1） */

/* ---------- 运行状态 ---------- */
static uint8_t g_estop = 0;        /* 急停锁存：1 = 已急停 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ============================================================
 * printf 重定向到 USART1（GCC newlib）
 *
 * 为什么不直接用 HAL_UART_Transmit？
 *   因为用寄存器操作更快，也不依赖 HAL 句柄的锁。
 *   syscalls.c 里有一个 __weak 的 _write，这里定义强符号覆盖它。
 * ============================================================ */
int _write(int file, char *ptr, int len)
{
    (void)file;

    for (int i = 0; i < len; i++) {
        /* 等待发送数据寄存器空 */
        while (!(USART1->SR & USART_SR_TXE)) {
            /* 等待 */
        }
        USART1->DR = (uint8_t)ptr[i];
    }

    /* 等最后一个字节真正发完 */
    while (!(USART1->SR & USART_SR_TC)) {
        /* 等待 */
    }

    return len;
}

/* ============================================================
 * LED 控制（PC13，User Label = LED）
 *
 * Blue Pill 板载的【用户 LED】接在 PC13，低电平点亮：
 *     PC13 = 0 → 亮
 *     PC13 = 1 → 灭
 *
 * 注意：板上另一个标 PWR 的灯是电源指示灯，通电就常亮，
 *       程序控制不了它，不要拿它当心跳指示。
 * ============================================================ */
static void led_write(uint8_t on)
{
    /* on = 1 点亮 → 引脚输出低电平 */
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void led_toggle(void)
{
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}

/* ============================================================
 * 方向按键 → 速度分量
 *
 * ⚠️ 本手柄的摇杆【没有模拟量】。手册原文：
 *    "左右摇杆模拟值为无效，推到极限时，
 *     对应发送 UP、RIGHT、DOWN、LEFT、∧、○、X、□"
 *    所以摇杆字节（ps2_rx[5]~[8]）是无效数据，不能用；
 *    空闲时它会读出乱七八糟的值，把电机驱得乱转。
 *    只能按"方向按键"来当数字量用。
 * ============================================================ */
static float btn_to_float(uint8_t pressed, float value)
{
    return pressed ? value : 0.0f;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
     HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  /* ---------- 1. 启动 PS2 驱动 ---------- */
  PS2_Init();

  /* ---------- 2. 启动电机 PWM ---------- */
  motor_init();

  /* ---------- 3. 开机自检：打印时钟信息，确认 72MHz ---------- */
  printf("\r\n========================================\r\n");
  printf("  PS2 Mecanum Car  -  STM32F103C8T6\r\n");
  printf("========================================\r\n");
  printf("SYSCLK = %lu Hz\r\n", (unsigned long)SystemCoreClock);
  printf("PCLK1  = %lu Hz\r\n", (unsigned long)HAL_RCC_GetPCLK1Freq());
  printf("----------------------------------------\r\n");
  printf("Left  stick : forward/back + strafe\r\n");
  printf("Right stick : rotate\r\n");
  printf("START       : toggle mode + release E-STOP\r\n");
  printf("SELECT      : emergency stop\r\n");
  printf("========================================\r\n\r\n");

  /* ---------- 4. LED 闪两下，表示初始化完成 ---------- */
  for (int i = 0; i < 2; i++) {
    led_write(1);
    HAL_Delay(120);
    led_write(0);
    HAL_Delay(120);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* 各任务的软件定时时间戳，进入循环前归零 */
  uint32_t t_ctrl = 0;    /* 上次控制输出时刻 */
  uint32_t t_ps2  = 0;    /* 上次手柄扫描时刻 */
  uint32_t t_dbg  = 0;    /* 上次调试打印时刻 */
  uint32_t t_led  = 0;    /* 上次 LED 翻转时刻 */

  /* 注意：不要在 WHILE 区块里再写 while(1)，
     CubeMX 的循环语句就在下面，重复写会造成嵌套死循环 */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    /* ==========================================================
     * 【1】读输入：扫描 PS2 手柄
     * ========================================================== */
    if ((now - t_ps2) >= PS2_PERIOD_MS) {
        t_ps2 = now;

        if (!PS2_Scan(&g_ps2)) {
            /* 手柄掉线 —— 立即停车并锁存急停（最重要的安全逻辑） */
            chassis_stop();
            g_estop = 1;
        }
    }

    /* ==========================================================
     * 【2】处理按键
     * ========================================================== */
    if (g_ps2.btn1 & PS2_BTN_SELECT) {
        g_estop = 1;                 /* SELECT = 急停 */
    }

    if (g_ps2.btn1 & PS2_BTN_START) {
        g_estop = 0;                 /* 解除急停 */
        HAL_Delay(200);              /* 消抖 */
    }

    /* ==========================================================
     * 【3】控制周期：解析意图 → 麦轮运动学 → 输出 PWM
     *
     * ⚠️ 本手柄摇杆【没有模拟量】（见手册字节定义）：
     *    左摇杆推到极限 → 发送 UP / RIGHT / DOWN / LEFT
     *    右摇杆推到极限 → 发送 ∧(三角) / ○(圈) / X(叉) / □(方)
     *
     * 所以映射关系：
     *    左摇杆 = 前后左右平移（八方向）
     *    右摇杆 = 原地自转（○ 顺时针 / □ 逆时针）
     *
     *    两个可以同时用 → 边走边转
     * ========================================================== */
    if ((now - t_ctrl) >= CTRL_PERIOD_MS) {
        t_ctrl = now;

        if (g_estop) {
            chassis_stop();
        } else {
            /* 3.1 左摇杆 / 方向键 → 平移 */
            float vx = btn_to_float(g_ps2.btn1 & PS2_BTN_UP,    +MOVE_SPEED)
                     + btn_to_float(g_ps2.btn1 & PS2_BTN_DOWN,  -MOVE_SPEED);

            float vy = btn_to_float(g_ps2.btn1 & PS2_BTN_RIGHT, +MOVE_SPEED)
                     + btn_to_float(g_ps2.btn1 & PS2_BTN_LEFT,  -MOVE_SPEED);

            /* 3.2 右摇杆 → 原地自转 */
            float w  = btn_to_float(g_ps2.btn2 & PS2_BTN_CIRCLE, +TURN_SPEED)   /* 右推 */
                     + btn_to_float(g_ps2.btn2 & PS2_BTN_SQUARE, -TURN_SPEED);  /* 左推 */

            /* 3.3 输出到麦轮（归一化在 chassis_mecanum 里做） */
            chassis_mecanum(vx, vy, w);
        }
    }

    /* ==========================================================
     * 【4】LED 心跳：正常慢闪，急停/掉线快闪
     * ========================================================== */
    {
        uint32_t led_period = g_estop ? 100u : 500u;
        if ((now - t_led) >= led_period) {
            t_led = now;
            led_toggle();
        }
    }

    /* ==========================================================
     * 【5】调试打印，每秒一次（调通后可整段注释掉）
     * ========================================================== */
    if ((now - t_dbg) >= 1000u) {
        t_dbg = now;
        printf("[%s] mode=%02X btn1=%02X btn2=%02X L=(%3u,%3u) R=(%3u,%3u)\r\n",
               g_estop ? "E-STOP" : "RUN   ",
               g_ps2.mode, g_ps2.btn1, g_ps2.btn2,
               g_ps2.ljoy_lr, g_ps2.ljoy_ud,
               g_ps2.rjoy_lr, g_ps2.rjoy_ud);
    }

    /* USER CODE END 3 */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* ============================================================
   * 诊断用：用 LED 报错
   *
   * 三种现象对应三种情况，一眼就能区分：
   *   开机闪两下 → 然后【快速闪烁不停】 = 进了 Error_Handler，
   *                                        HAL 初始化失败（最可能是 HSE 起振）
   *   开机闪两下 → 然后【慢闪 / 快闪】   = 程序正常运行
   *   什么都不动 / 一直常亮               = 卡在别的死循环，不是这里
   *
   * 两个注意点：
   *  1. 这里已经 __disable_irq()，不能用 HAL_Delay，只能忙等
   *  2. 这个函数可能在 MX_GPIO_Init() 之前被调用（比如
   *     SystemClock_Config() 里 HSE 起振失败），所以必须
   *     自己先把 PC13 配成输出，否则灯根本点不亮
   * ============================================================ */
  __disable_irq();

  /* 自己配置 PC13，不依赖 MX_GPIO_Init() */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  {
    GPIO_InitTypeDef gi = {0};
    gi.Pin   = LED_Pin;
    gi.Mode  = GPIO_MODE_OUTPUT_PP;
    gi.Pull  = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_GPIO_Port, &gi);
  }

  while (1)
  {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    for (volatile uint32_t i = 0; i < 1000000u; i++) { }
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
