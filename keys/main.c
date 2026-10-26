/*==============================================================================
 *  STM32F407VET6  板载 5 键驱动 + 演示
 *  目标：STM32F407VET6  /  HAL 库  /  Keil MDK
 *
 *  ---------------------------------------------------------------------------
 *  实测结果（2026-10-26，用 按键探测/main.c 扫出来的，别再改）
 *  ---------------------------------------------------------------------------
 *      KEY1 = PD10
 *      KEY2 = PD11
 *      KEY3 = PD12
 *      KEY4 = PD13
 *      KEY5 = PD14
 *
 *  电气特性（探测程序实测）：
 *      静止读数 = 0   板上有外部下拉电阻
 *      按下读数 = 1   按键把这个脚接到 3.3V
 *      所以是【高电平有效】，程序里再开一个内部下拉，抗干扰
 *
 *  注意：这 5 个脚跟你自己接的 PE2~PE5 完全无关。
 *        实测按这 5 个键，PE2~PE5 一个都没动。
 *
 *  ---------------------------------------------------------------------------
 *  用法
 *  ---------------------------------------------------------------------------
 *      把本文件整个覆盖到工程的 Core/Src/main.c，编译烧录即可。
 *      串口 115200（PA9=TX PA10=RX），按一个键就打一行。
 *      LED1（PB6）每按一次翻转一次，不接串口也能看出来。
 *
 *      输出长这样：
 *          [KEY] KEY1 PRESS   state=[1][ ][ ][ ][ ]
 *          [KEY] KEY1 RELEASE
 *          [KEY] KEY3 LONG    (held 800ms)
 *
 *  ---------------------------------------------------------------------------
 *  移植到别的引脚 / 别的极性
 *  ---------------------------------------------------------------------------
 *      · 换引脚：改 KEY_PORT 和 s_key_pin[] / s_key_name[]
 *      · 换极性：KEY_ACTIVE_HIGH 改 0（低电平有效，静止 1、按下 0）
 *      · 改手感：KEY_DEBOUNCE_CNT（消抖）/ KEY_LONG_CNT（长按阈值）
 *
 *  ---------------------------------------------------------------------------
 *  API（想自己用，看这四个就够）
 *  ---------------------------------------------------------------------------
 *      void    Key_Init(void);        上电调一次
 *      void    Key_Poll(void);        每 10ms 调一次（放主循环或定时器里）
 *      uint8_t Key_Pressed(void);     取「本次新按下」的位掩码，读后清零
 *      uint8_t Key_Released(void);    取「本次新松开」的位掩码，读后清零
 *      uint8_t Key_Held(void);        当前按住哪些键（不清零）
 *      uint8_t Key_Long(void);        取「本次触发长按」的位掩码，读后清零
 *
 *      位掩码：bit0 = KEY1(PD10) ... bit4 = KEY5(PD14)
 *============================================================================*/

#include "stm32f4xx_hal.h"

/*==============================================================================
 *                              开关区
 *============================================================================*/

#define KEY_NUM            5     /* 按键数量 */
#define KEY_ACTIVE_HIGH    1     /* 1 = 高电平有效（本板就是这个）0 = 低电平有效 */
#define KEY_DEBOUNCE_CNT   2     /* 连续 N 次采样一致才认：2 x 10ms = 20ms */
#define KEY_LONG_CNT       80    /* 按住 N 次采样算长按：80 x 10ms = 800ms */
#define USE_SERIAL         1     /* 串口打印，不要就改 0 */
#define USE_LED            1     /* PB6 上那颗 LED，每按一次翻转 */

/*==============================================================================
 *                              引脚表
 *============================================================================*/

#define KEY_PORT           GPIOD

static const uint16_t s_key_pin[KEY_NUM] =
{
    GPIO_PIN_10,    /* KEY1 */
    GPIO_PIN_11,    /* KEY2 */
    GPIO_PIN_12,    /* KEY3 */
    GPIO_PIN_13,    /* KEY4 */
    GPIO_PIN_14     /* KEY5 */
};

static const char *s_key_name[KEY_NUM] =
{
    "KEY1", "KEY2", "KEY3", "KEY4", "KEY5"
};

/*==============================================================================
 *                              内部状态
 *============================================================================*/

static uint8_t  s_stable;               /* 消抖后的稳定状态位掩码 */
static uint8_t  s_cnt[KEY_NUM];         /* 每个键的消抖计数 */
static uint16_t s_hold[KEY_NUM];        /* 每个键的按住时长（采样次数） */
static uint8_t  s_ev_pressed;           /* 待取走的「按下」事件 */
static uint8_t  s_ev_released;          /* 待取走的「松开」事件 */
static uint8_t  s_ev_long;              /* 待取走的「长按」事件 */

/*==============================================================================
 * 一、串口（USART1 纯寄存器版，PA9=TX / PA10=RX）
 *============================================================================*/
static void USART1_InitRaw(uint32_t pclk2, uint32_t baud)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    GPIOA->MODER   &= ~((3u << 18) | (3u << 20));
    GPIOA->MODER   |=  ((2u << 18) | (2u << 20));
    GPIOA->OSPEEDR |=  ((3u << 18) | (3u << 20));
    GPIOA->OTYPER  &= ~((1u << 9)  | (1u << 10));
    GPIOA->PUPDR   &= ~((3u << 18) | (3u << 20));
    GPIOA->PUPDR   |=  (1u << 20);
    GPIOA->AFR[1]  &= ~((0xFu << 4) | (0xFu << 8));
    GPIOA->AFR[1]  |=  ((7u << 4) | (7u << 8));

    USART1->CR1 = 0;
    USART1->CR2 = 0;
    USART1->CR3 = 0;
    USART1->BRR = (pclk2 + (baud / 2u)) / baud;
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

static void U_PutC(char c)
{
#if (USE_SERIAL != 0)
    while ((USART1->SR & USART_SR_TXE) == 0)
    {
    }
    USART1->DR = (uint8_t)c;
#else
    (void)c;
#endif
}

static void U_Send(const char *s)
{
    while (*s)
    {
        U_PutC(*s);
        s++;
    }
}

/*==============================================================================
 * 二、时钟：HSI 16MHz -> PLL -> 168MHz
 *   必须用 HAL_RCC_ClockConfig()，它会顺手重算 SysTick，
 *   否则 HAL_Delay() 会快 10 倍左右。
 *============================================================================*/
static int Clock_168MHz(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    osc.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState            = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState        = RCC_PLL_ON;
    osc.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    osc.PLL.PLLM            = 16;
    osc.PLL.PLLN            = 336;
    osc.PLL.PLLP            = RCC_PLLP_DIV2;
    osc.PLL.PLLQ            = 7;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK)
    {
        return -1;
    }

    clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                         RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV4;
    clk.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5) != HAL_OK)
    {
        return -2;
    }
    return 0;
}

/*==============================================================================
 * 三、LED（PB6 = LED1）
 *============================================================================*/
static void LED_Init(void)
{
    GPIO_InitTypeDef gi = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    gi.Mode  = GPIO_MODE_OUTPUT_PP;
    gi.Pull  = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_LOW;
    gi.Pin   = GPIO_PIN_6;
    HAL_GPIO_Init(GPIOB, &gi);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
}

static void LED_Toggle(void)
{
#if (USE_LED != 0)
    if ((GPIOB->ODR & GPIO_PIN_6) != 0u)
    {
        GPIOB->BSRR = ((uint32_t)GPIO_PIN_6 << 16);
    }
    else
    {
        GPIOB->BSRR = (uint32_t)GPIO_PIN_6;
    }
#endif
}

/*==============================================================================
 * 四、按键驱动
 *============================================================================*/

/* 读一个键的「逻辑电平」：1 = 按下 */
static uint8_t Key_RawBit(uint8_t i)
{
    uint8_t v;

    v = ((KEY_PORT->IDR & s_key_pin[i]) != 0u) ? 1u : 0u;

#if (KEY_ACTIVE_HIGH != 0)
    return v;                   /* 高有效：读到 1 就是按下 */
#else
    return (uint8_t)(v ^ 1u);   /* 低有效：读到 0 才是按下 */
#endif
}

void Key_Init(void)
{
    GPIO_InitTypeDef gi = {0};
    uint8_t i;

    __HAL_RCC_GPIOD_CLK_ENABLE();

    gi.Mode  = GPIO_MODE_INPUT;
    gi.Speed = GPIO_SPEED_FREQ_LOW;

    /* 高有效 -> 内部下拉；低有效 -> 内部上拉。
     * 本板已有外部下拉，内部再下一个，抗干扰更强。 */
#if (KEY_ACTIVE_HIGH != 0)
    gi.Pull = GPIO_PULLDOWN;
#else
    gi.Pull = GPIO_PULLUP;
#endif

    gi.Pin = 0;
    for (i = 0; i < KEY_NUM; i++)
    {
        gi.Pin |= s_key_pin[i];
    }
    HAL_GPIO_Init(KEY_PORT, &gi);

    /* 清状态 */
    s_stable      = 0u;
    s_ev_pressed  = 0u;
    s_ev_released = 0u;
    s_ev_long     = 0u;
    for (i = 0; i < KEY_NUM; i++)
    {
        s_cnt[i]  = 0u;
        s_hold[i] = 0u;
        if (Key_RawBit(i) != 0u)
        {
            s_stable |= (uint8_t)(1u << i);     /* 上电就按着也算 */
        }
    }
}

/* 每 10ms 调一次。所有消抖/长按逻辑都在这里。 */
void Key_Poll(void)
{
    uint8_t i;
    uint8_t bit;
    uint8_t raw;

    for (i = 0; i < KEY_NUM; i++)
    {
        bit = (uint8_t)(1u << i);
        raw = Key_RawBit(i);

        if (raw == ((s_stable & bit) != 0u ? 1u : 0u))
        {
            /* 和稳定状态一致：清消抖计数 */
            s_cnt[i] = 0u;

            /* 按住的期间累计时长，到阈值报一次长按 */
            if (raw != 0u)
            {
                if (s_hold[i] < 0xFFFFu)
                {
                    s_hold[i]++;
                }
                if (s_hold[i] == KEY_LONG_CNT)
                {
                    s_ev_long |= bit;
                }
            }
        }
        else
        {
            /* 和稳定状态不一致：开始消抖 */
            s_cnt[i]++;
            if (s_cnt[i] >= KEY_DEBOUNCE_CNT)
            {
                s_cnt[i]  = 0u;
                s_hold[i] = 0u;
                s_stable ^= bit;                /* 状态翻过来 */

                if (raw != 0u)
                {
                    s_ev_pressed |= bit;
                }
                else
                {
                    s_ev_released |= bit;
                }
            }
        }
    }
}

uint8_t Key_Pressed(void)
{
    uint8_t v = s_ev_pressed;
    s_ev_pressed = 0u;
    return v;
}

uint8_t Key_Released(void)
{
    uint8_t v = s_ev_released;
    s_ev_released = 0u;
    return v;
}

uint8_t Key_Long(void)
{
    uint8_t v = s_ev_long;
    s_ev_long = 0u;
    return v;
}

uint8_t Key_Held(void)
{
    return s_stable;
}

/*==============================================================================
 * 五、CubeMX 生成代码需要的钩子
 *   usart.c 等生成文件里会调用 Error_Handler()，它原本定义在 CubeMX 的
 *   main.c 里。本文件把 main.c 整个替换了，所以要自己提供一份，否则链接报：
 *       Error: L6218E: Undefined symbol Error_Handler (referred from usart.o)
 *============================================================================*/
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
    while (1)
    {
    }
}
#endif

/*==============================================================================
 * 六、演示：把按键状态打到串口
 *============================================================================*/

static void Print_State(uint8_t held)
{
    uint8_t i;

    U_Send("  state=");
    for (i = 0; i < KEY_NUM; i++)
    {
        if ((held & (uint8_t)(1u << i)) != 0u)
        {
            U_PutC('[');
            U_PutC((char)('1' + i));        /* 1..5 */
            U_PutC(']');
        }
        else
        {
            U_Send("[ ]");
        }
    }
    U_Send("\r\n");
}

/* 处理一批事件并打印 */
static void Handle_Events(void)
{
    uint8_t p;
    uint8_t r;
    uint8_t l;
    uint8_t i;

    p = Key_Pressed();
    r = Key_Released();
    l = Key_Long();

    /* 有任何键按下就翻一下 LED，不接串口也能看出程序在跑 */
    if (p != 0u)
    {
        LED_Toggle();
    }

    for (i = 0; i < KEY_NUM; i++)
    {
        if ((p & (uint8_t)(1u << i)) != 0u)
        {
            U_Send("[KEY] ");
            U_Send(s_key_name[i]);
            U_Send(" PRESS");
            Print_State(Key_Held());
        }
    }
    for (i = 0; i < KEY_NUM; i++)
    {
        if ((l & (uint8_t)(1u << i)) != 0u)
        {
            U_Send("[KEY] ");
            U_Send(s_key_name[i]);
            U_Send(" LONG   (hold)\r\n");
        }
    }
    for (i = 0; i < KEY_NUM; i++)
    {
        if ((r & (uint8_t)(1u << i)) != 0u)
        {
            U_Send("[KEY] ");
            U_Send(s_key_name[i]);
            U_Send(" RELEASE");
            Print_State(Key_Held());
        }
    }
}

/*==============================================================================
 * 七、main
 *============================================================================*/
int main(void)
{
    int r;

    HAL_Init();

    USART1_InitRaw(16000000u, 115200u);

    U_Send("\r\n\r\n\r\n");
    U_Send("################################################\r\n");
    U_Send("#  STM32F407  on-board 5 keys demo  v1.0\r\n");
    U_Send("#  KEY1=PD10 KEY2=PD11 KEY3=PD12 KEY4=PD13 KEY5=PD14\r\n");
    U_Send("#  active HIGH, external pull-down, UART1 115200\r\n");
    U_Send("################################################\r\n");

    r = Clock_168MHz();
    if (r != 0)
    {
        U_Send("[ERR] clock 168MHz failed, stay at 16MHz\r\n");
    }
    else
    {
        USART1_InitRaw(84000000u, 115200u);
        U_Send("[OK]  clock 168MHz\r\n");
    }

    LED_Init();
    Key_Init();

    U_Send("[OK]  keys ready. press any key.\r\n");
    U_Send("      KEY1=PD10  KEY2=PD11  KEY3=PD12  KEY4=PD13  KEY5=PD14\r\n\r\n");

    while (1)
    {
        Key_Poll();             /* 10ms 一次 */
        Handle_Events();
        HAL_Delay(10);
    }
}
