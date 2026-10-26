/*==============================================================================
 *  STM32F407VET6  板载按键引脚探测
 *  目标：STM32F407VET6  /  HAL 库  /  Keil MDK
 *
 *  ---------------------------------------------------------------------------
 *  这个程序干什么
 *  ---------------------------------------------------------------------------
 *  板上 5 个按键分别接在哪个 GPIO 上？丝印看不清，用这个程序问出来。
 *
 *  原理（为什么能扫出来）：
 *    - 按键一端接 GND（最常见）：脚配内部上拉，静止读 1，按下读 0
 *    - 按键一端接 3.3V：脚上通常有外部下拉，静止读 0，按下读 1
 *    - 没接东西的悬空脚：强上拉下稳定读 1，不会乱跳，不会误报
 *
 *  所以统一把候选脚配成「输入 + 内部上拉」，然后盯着谁偏离静止电平：
 *    静止 1 -> 按 0   低有效按键
 *    静止 0 -> 按 1   高有效按键
 *  两种都能抓到，不用你先猜极性。
 *
 *  ---------------------------------------------------------------------------
 *  三步流程（串口会依次打印 STEP1 / STEP2 / STEP3）
 *  ---------------------------------------------------------------------------
 *    STEP1  双极性探测
 *           每个脚先「上拉」读一次，再「下拉」读一次
 *             两次读数相同 -> 有外部上/下拉电阻 -> 最可能是按键脚，会列出来
 *             两次读数不同 -> 悬空脚 -> 排除噪声源
 *    STEP2  用上拉模式采样静止基线
 *    STEP3  开始监听
 *           谁偏离基线就立刻打印 [KEY] Pxx  DOWN / UP
 *           每 5 秒打印一次 [SUM] 汇总
 *           找齐 5 个脚会打印 ALL 5 KEYS FOUND，这就是结束了
 *
 *  ---------------------------------------------------------------------------
 *  怎么用
 *  ---------------------------------------------------------------------------
 *    1. 把本文件整个覆盖到工程的 Core/Src/main.c，编译烧录
 *    2. 串口 115200（PA9=TX PA10=RX）打开
 *    3. 等 STEP3 出现后，把 5 个键一个一个按下去，每个按 1 秒左右松开
 *       （一个一个按，别同时按，这样编号才对得上）
 *    4. 每按到一个新脚会打印 [NEW] board key #N = Pxx
 *       N 就是「你按的第几个键」，Pxx 就是它的引脚 —— 这就是你要的答案
 *    5. 找齐 5 个会打印 ALL 5 BOARD KEYS FOUND，程序不会停，
 *       但 [SUM] 每 5 秒还会继续刷新，方便你核对
 *
 *  ---------------------------------------------------------------------------
 *  四个必须知道的注意点
 *  ---------------------------------------------------------------------------
 *    1. PE2~PE5 是「之前就已经确认是按键」的 4 个脚（你手工接过的那组）。
 *       但板载 5 个键有没有可能就是它们？现在说不准。所以本程序的原则是：
 *         - 照常扫，照常统计，**照样算进 5 个里**
 *         - 只在输出里加一个 [known] 标记，方便你认出「这几个是老的」
 *       也就是说：程序绝不会因为「以为是自己的键」就把板载键漏掉。
 *       如果最后 5 个里有 4 个是 PE2~PE5，那就说明板载键就是它们。
 *    2. 如果板载 5 个键里有一个是「复位键」，它接在 NRST 引脚上，不是 GPIO，
 *       这个程序永远扫不出来。看到只找到 4 个就是这种情况。
 *    3. 程序完全不碰 PA13/PA14（SWD 下载口），也把 PA9/PA10 让给串口，
 *       所以烧完以后还能正常下载，不会变砖。
 *    4. 屏如果还接着（PC3 / PB10 / PC14 / PC15），跑这个程序屏会花掉，
 *       因为这几个脚被反复配成输入。不影响按键结果，测完重新烧屏的程序即可。
 *
 *  ---------------------------------------------------------------------------
 *  第一次跑起来之后，按现象调这几个开关
 *  ---------------------------------------------------------------------------
 *    日志太长                        -> SHOW_FLOAT_PINS 改 0（默认就是 0）
 *    按键有反应但统计次数很少        -> DEBOUNCE_TICKS 改 1
 *    一个键被报好几次                -> DEBOUNCE_TICKS 改 3
 *    「confirmed」那一项上不去       -> CONFIRM_HITS 改小（它只是置信度，不影响找齐判定）
 *    板子没有 PD 口的按键            -> SCAN_PORT_D 改 0（扫得快一点）
 *    嫌 [known] 标记碍眼             -> KNOWN_KEYS 改 0（所有脚一视同仁）
 *============================================================================*/

#include "stm32f4xx_hal.h"

/*==============================================================================
 *                              开关区
 *============================================================================*/

#define SAMPLE_MS        20      /* 采样周期（ms） */
#define DEBOUNCE_TICKS   2       /* 连续 N 次采样一致才认：20ms x 2 = 40ms */
#define REPORT_MS        5000    /* 每 N ms 打印一次 [SUM] 汇总 */
#define SCAN_PORT_D      1       /* 1 = 也扫 PD 口（LQFP100 有 PD0~PD15） */
#define CONFIRM_HITS     3       /* 偏离基线 >= N 次才列为「确认按键脚」 */
#define SHOW_FLOAT_PINS  0       /* 1 = 把悬空脚也打印出来（日志会变长） */
#define TARGET_KEYS      5       /* 找齐几个板载键就宣布完成 */

/* 已知按键标记：PE2~PE5 是之前就已经确认是按键的 4 个脚。
 * 板载 5 个键有可能就是它们，所以默认【照常统计、照样算进 5 个里】，
 * 只额外加一个 [known] 标记，方便你认出「这几个是老的」。
 * 1 = 加标记（推荐，绝不会漏）
 * 0 = 连标记也不加，所有脚一视同仁 */
#define KNOWN_KEYS       1

/* 端口掩码：对应位 = 1 表示要扫这个脚
 * PA 口跳过的四个：
 *   PA9 / PA10  串口 TX/RX，本程序输出靠它
 *   PA13 / PA14 SWD 下载口，动了就下载不了 */
#define MASK_A  ((uint16_t)(0xFFFFu & ~((1u << 9) | (1u << 10) | (1u << 13) | (1u << 14))))
#define MASK_B  ((uint16_t)0xFFFFu)
#define MASK_C  ((uint16_t)0xFFFFu)
#define MASK_D  ((uint16_t)0xFFFFu)
#define MASK_E  ((uint16_t)0xFFFFu)

/* PUPDR 取值 */
#define PULL_NONE       0u
#define PULL_UP         1u
#define PULL_DOWN       2u

/*==============================================================================
 *                              数据结构
 *============================================================================*/

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t      pin;      /* 单个 bit */
    uint8_t       idle;     /* 静止电平（基线） */
    uint8_t       cnt;      /* 连续偏离计数（消抖用） */
    uint16_t      hits;     /* 偏离基线的次数 */
    uint8_t       ext;      /* 1 = 双极性一致，说明有外部上/下拉电阻 */
    uint8_t       known;    /* 1 = PE2~PE5，之前就知道是按键（只是标记，照样算） */
} BtnPin;

#define MAX_PIN   80
static BtnPin  s_pin[MAX_PIN];
static int     s_n   = 0;
static uint8_t s_done = 0;      /* 1 = 已经打印过「全部找到」横幅，不再重复 */

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
    while ((USART1->SR & USART_SR_TXE) == 0)
    {
    }
    USART1->DR = (uint8_t)c;
}

static void U_Send(const char *s)
{
    while (*s)
    {
        U_PutC(*s);
        s++;
    }
}

static void U_Dec(uint32_t v)
{
    char b[11];
    int  i = 0;

    if (v == 0u)
    {
        U_PutC('0');
        return;
    }
    while ((v > 0u) && (i < 10))
    {
        b[i] = (char)('0' + (char)(v % 10u));
        v /= 10u;
        i++;
    }
    while (i > 0)
    {
        i--;
        U_PutC(b[i]);
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
 * 三、GPIO 小工具
 *============================================================================*/
static void GPIO_Clock_All(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
}

/* pin 是单个 bit，先算出位序号 */
static uint32_t Pin_Pos(uint16_t pin)
{
    uint32_t pos = 0;

    while ((pos < 16u) && ((pin & (1u << pos)) == 0u))
    {
        pos++;
    }
    return pos;
}

/* 配成输入，并指定内部上/下拉 */
static void Pin_SetPull(GPIO_TypeDef *port, uint16_t pin, uint32_t pull)
{
    uint32_t pos = Pin_Pos(pin);
    uint32_t sh  = pos * 2u;

    port->MODER &= ~(3u << sh);                 /* 00 = 输入模式 */
    port->PUPDR &= ~(3u << sh);
    port->PUPDR |=  (pull << sh);
}

static uint8_t Pin_Read(GPIO_TypeDef *port, uint16_t pin)
{
    return ((port->IDR & pin) != 0u) ? 1u : 0u;
}

/* 把 port+pin 变成 "PE3" 这种名字 */
static void Pin_Name(GPIO_TypeDef *port, uint16_t pin, char *buf)
{
    uint32_t pos = Pin_Pos(pin);
    char     pc  = '?';

    if (port == GPIOA)      { pc = 'A'; }
    else if (port == GPIOB) { pc = 'B'; }
    else if (port == GPIOC) { pc = 'C'; }
    else if (port == GPIOD) { pc = 'D'; }
    else if (port == GPIOE) { pc = 'E'; }

    buf[0] = 'P';
    buf[1] = pc;
    if (pos >= 10u)
    {
        buf[2] = '1';
        buf[3] = (char)('0' + (char)(pos - 10u));
        buf[4] = 0;
    }
    else
    {
        buf[2] = (char)('0' + (char)pos);
        buf[3] = 0;
    }
}

/*==============================================================================
 * 四、候选脚表
 *============================================================================*/
static void Pins_Add(GPIO_TypeDef *port, uint16_t mask)
{
    uint32_t b;

    for (b = 0; b < 16u; b++)
    {
        if ((mask & (1u << b)) != 0u)
        {
            if (s_n < MAX_PIN)
            {
                s_pin[s_n].port = port;
                s_pin[s_n].pin  = (uint16_t)(1u << b);
                s_pin[s_n].idle = 1u;
                s_pin[s_n].cnt  = 0u;
                s_pin[s_n].hits = 0u;
                s_pin[s_n].ext  = 0u;
                s_pin[s_n].known = 0u;
                s_n++;
            }
        }
    }
}

/* 把 PE2~PE5 标成「之前已知的按键」（只加标记，不从表里删） */
static void Pins_MarkKnown(void)
{
    int      i;
    uint16_t known_mask;

    known_mask = (uint16_t)(GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);

    for (i = 0; i < s_n; i++)
    {
        if ((s_pin[i].port == GPIOE) && ((s_pin[i].pin & known_mask) != 0u))
        {
            s_pin[i].known = 1u;
        }
    }
}

static void Pins_Build(void)
{
    s_n = 0;
    Pins_Add(GPIOA, MASK_A);
    Pins_Add(GPIOB, MASK_B);
    Pins_Add(GPIOC, MASK_C);
#if (SCAN_PORT_D != 0)
    Pins_Add(GPIOD, MASK_D);
#endif
    Pins_Add(GPIOE, MASK_E);

#if (KNOWN_KEYS != 0)
    Pins_MarkKnown();
#endif
}

/*==============================================================================
 * 五、STEP1：双极性探测
 *   上拉读一次、下拉读一次：
 *     一样  -> 有外部电阻（按键脚、LED、BOOT1 之类）
 *     不同  -> 悬空
 *============================================================================*/
static void Step1_Probe(void)
{
    int     i;
    int     col;
    uint8_t b;
    char    nm[6];

    U_Send("\r\n[STEP1] bipolar probe: read each pin with pull-up, then pull-down\r\n");

    for (i = 0; i < s_n; i++)
    {
        Pin_SetPull(s_pin[i].port, s_pin[i].pin, PULL_UP);
    }
    HAL_Delay(5);
    for (i = 0; i < s_n; i++)
    {
        s_pin[i].idle = Pin_Read(s_pin[i].port, s_pin[i].pin);
    }

    for (i = 0; i < s_n; i++)
    {
        Pin_SetPull(s_pin[i].port, s_pin[i].pin, PULL_DOWN);
    }
    HAL_Delay(5);
    for (i = 0; i < s_n; i++)
    {
        b = Pin_Read(s_pin[i].port, s_pin[i].pin);
        if (b == s_pin[i].idle)
        {
            s_pin[i].ext = 1u;
        }
        Pin_SetPull(s_pin[i].port, s_pin[i].pin, PULL_UP);   /* 恢复上拉 */
    }
    HAL_Delay(5);

    /* 有外部电阻的脚：最可能是按键 */
    U_Send("[STEP1] pins WITH external resistor (best candidates):\r\n        ");
    col = 0;
    for (i = 0; i < s_n; i++)
    {
        if (s_pin[i].ext != 0u)
        {
            Pin_Name(s_pin[i].port, s_pin[i].pin, nm);
            U_Send(nm);
            if (s_pin[i].known != 0u)
            {
                U_PutC('*');            /* * = 之前已知的按键 PE2~PE5 */
            }
            U_PutC('=');
            U_PutC((s_pin[i].idle != 0u) ? '1' : '0');
            U_PutC(' ');
            col++;
            if ((col % 8) == 0)
            {
                U_Send("\r\n        ");
            }
        }
    }
    if (col == 0)
    {
        U_Send("(none)");
    }
    U_Send("\r\n");
#if (KNOWN_KEYS != 0)
    U_Send("[STEP1] legend: Pxx=1 means idle HIGH, * means already known (PE2~PE5)\r\n");
#endif

#if (SHOW_FLOAT_PINS != 0)
    U_Send("[STEP1] floating pins (ignored as noise):\r\n        ");
    col = 0;
    for (i = 0; i < s_n; i++)
    {
        if (s_pin[i].ext == 0u)
        {
            Pin_Name(s_pin[i].port, s_pin[i].pin, nm);
            U_Send(nm);
            U_PutC(' ');
            col++;
            if ((col % 10) == 0)
            {
                U_Send("\r\n        ");
            }
        }
    }
    U_Send("\r\n");
#endif
}

/*==============================================================================
 * 六、STEP2：采样静止基线
 *   这里用「上拉 + 多次采样取一致值」，比单次读稳。
 *============================================================================*/
static void Step2_Idle(void)
{
    int     i;
    int     k;
    uint8_t v;
    uint8_t same;
    char    nm[6];

    U_Send("[STEP2] sampling idle level (keep hands off the board)...\r\n");
    HAL_Delay(300);

    for (i = 0; i < s_n; i++)
    {
        same = 1u;
        v    = Pin_Read(s_pin[i].port, s_pin[i].pin);
        for (k = 0; k < 5; k++)
        {
            HAL_Delay(4);
            if (Pin_Read(s_pin[i].port, s_pin[i].pin) != v)
            {
                same = 0u;
            }
        }
        s_pin[i].idle = v;
        s_pin[i].cnt  = 0u;
        if (same == 0u)
        {
            Pin_Name(s_pin[i].port, s_pin[i].pin, nm);
            U_Send("[STEP2] warning: pin ");
            U_Send(nm);
            U_Send(" unstable at idle, treat its result with care\r\n");
        }
    }

    U_Send("[STEP2] idle baseline locked\r\n");
    U_Send("[STEP3] listening now. press the 5 keys one by one, 1 second each.\r\n");
    U_Send("        format: [KEY] Pxx PRESS (idle=0 now=1) / [KEY] Pxx RELEASE\r\n\r\n");
}

/*==============================================================================
 * 七、STEP3：监听 + 汇总
 *============================================================================*/
static void Print_Summary(void)
{
    int  i;
    int  any;
    int  found;
    int  conf;
    int  nk;
    char nm[6];

    any   = 0;
    found = 0;
    conf  = 0;
    nk    = 0;

    /* 所有被按过的脚都列出来，一个都不漏（PE2~PE5 也在里面） */
    U_Send("[SUM] keys hit  : ");
    for (i = 0; i < s_n; i++)
    {
        if (s_pin[i].hits > 0u)
        {
            Pin_Name(s_pin[i].port, s_pin[i].pin, nm);
            U_Send(nm);
            if (s_pin[i].known != 0u)
            {
                U_PutC('*');            /* * = 之前已知的 PE2~PE5 */
            }
            U_PutC('(');
            U_Dec((uint32_t)s_pin[i].hits);
            U_Send(") ");
            any++;
            found++;                    /* 按过 1 次就算「找到了」 */
            if (s_pin[i].known != 0u)
            {
                nk++;
            }
            if (s_pin[i].hits >= CONFIRM_HITS)
            {
                conf++;                 /* 按够次数才算「高置信」 */
            }
        }
    }
    if (any == 0)
    {
        U_Send("none yet, press a key");
    }
    U_Send("\r\n");

    /* 关键：用 found 判断「找齐没有」，不是用 conf。
     * 手动测试每个键只按 1 次很正常，用 conf 会永远凑不满。 */
    U_Send("[SUM] progress : found ");
    U_Dec((uint32_t)found);
    U_Send(" / ");
    U_Dec((uint32_t)TARGET_KEYS);
    U_Send("   (confirmed ");
    U_Dec((uint32_t)conf);
    U_Send(" / ");
    U_Dec((uint32_t)found);
    U_Send(", hits>=");
    U_Dec((uint32_t)CONFIRM_HITS);
    U_Send(")");
#if (KNOWN_KEYS != 0)
    U_Send("   PE2~PE5 among them = ");
    U_Dec((uint32_t)nk);
#endif
    U_Send("\r\n");

    if ((found >= TARGET_KEYS) && (s_done == 0u))
    {
        s_done = 1u;
        U_Send("\r\n");
        U_Send("################################################\r\n");
        U_Send("#  ALL ");
        U_Dec((uint32_t)found);
        U_Send(" KEYS FOUND  -  see the [SUM] list above\r\n");
        U_Send("################################################\r\n");
#if (KNOWN_KEYS != 0)
        if (nk >= 4)
        {
            U_Send("#  NOTE: ");
            U_Dec((uint32_t)nk);
            U_Send(" of them are PE2~PE5 -> the board keys ARE PE2~PE5\r\n");
            U_Send("#        (you were right, they are the same pins)\r\n");
        }
        else if (nk == 0)
        {
            U_Send("#  NOTE: none of them is PE2~PE5 -> board keys are elsewhere\r\n");
        }
        else
        {
            U_Send("#  NOTE: PE2~PE5 are partially involved, check the list by hand\r\n");
        }
#endif
        U_Send("#  copy those pins into your key driver, then rebuild\r\n");
        U_Send("#  (assumes you only pressed the 5 built-in keys, not your own)\r\n");
        U_Send("#  (this banner prints once; [SUM] keeps updating)\r\n");
        U_Send("\r\n");
    }
}

static void Step3_Loop(void)
{
    int      i;
    int      j;
    int      idx;
    uint8_t  v;
    uint32_t t;
    char     nm[6];

    t = 0u;

    while (1)
    {
        for (i = 0; i < s_n; i++)
        {
            v = Pin_Read(s_pin[i].port, s_pin[i].pin);

            if (v == s_pin[i].idle)
            {
                /* 回到静止电平：如果之前是「按着」状态，打印 UP */
                if (s_pin[i].cnt >= DEBOUNCE_TICKS)
                {
                    Pin_Name(s_pin[i].port, s_pin[i].pin, nm);
                    U_Send("[KEY] ");
                    U_Send(nm);
                    if (s_pin[i].known != 0u)
                    {
                        U_Send(" [known]");
                    }
                    U_Send(" RELEASE\r\n");
                }
                s_pin[i].cnt = 0u;
            }
            else
            {
                if (s_pin[i].cnt < 255u)
                {
                    s_pin[i].cnt++;
                }
                if (s_pin[i].cnt == DEBOUNCE_TICKS)
                {
                    s_pin[i].hits++;
                    Pin_Name(s_pin[i].port, s_pin[i].pin, nm);
                    U_Send("[KEY] ");
                    U_Send(nm);
                    if (s_pin[i].known != 0u)
                    {
                        U_Send(" [known]");
                    }
                    U_Send(" PRESS  (idle=");
                    U_PutC((s_pin[i].idle != 0u) ? '1' : '0');
                    U_Send(" now=");
                    U_PutC((v != 0u) ? '1' : '0');
                    U_Send(")\r\n");

                    /* 第一次发现这个脚：给它编号，方便对应「第几个键」
                     * 注意：要数【全表】里已经命中过的脚，不能只数到 i
                     * （之前只数到 i，导致先按 PD13 再按 PD11 都编成 #1） */
                    if (s_pin[i].hits == 1u)
                    {
                        idx = 0;
                        for (j = 0; j < s_n; j++)
                        {
                            if (s_pin[j].hits > 0u)
                            {
                                idx++;
                            }
                        }
                        U_Send("[NEW] key #");
                        U_Dec((uint32_t)idx);
                        U_Send(" = ");
                        U_Send(nm);
                        if (s_pin[i].known != 0u)
                        {
                            U_Send("   <- already known (PE2~PE5)");
                        }
                        U_Send("\r\n");
                    }
                }
            }
        }

        HAL_Delay(SAMPLE_MS);

        t += SAMPLE_MS;
        if (t >= REPORT_MS)
        {
            t = 0u;
            Print_Summary();
        }
    }
}

/*==============================================================================
 * 八、CubeMX 生成代码需要的钩子
 *
 *   usart.c 等生成文件里会调用 Error_Handler()，它原本定义在 CubeMX 的
 *   main.c 里。本文件把 main.c 整个替换了，所以必须自己提供一份，
 *   否则链接报：
 *       Error: L6218E: Undefined symbol Error_Handler (referred from usart.o)
 *============================================================================*/
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

/* 工程里如果开了 USE_FULL_ASSERT，assert_failed 也得有一份 */
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
 * 九、main
 *============================================================================*/
int main(void)
{
    int r;

    HAL_Init();

    /* 先用 HSI 16MHz 把串口打开，保证后面无论时钟成不成都有输出 */
    USART1_InitRaw(16000000u, 115200u);

    U_Send("\r\n\r\n\r\n");
    U_Send("################################################\r\n");
    U_Send("#  STM32F407  on-board KEY pin finder  v1.0\r\n");
    U_Send("#  boot OK, HSI 16MHz, UART1 115200 (PA9/PA10)\r\n");
    U_Send("################################################\r\n");

    r = Clock_168MHz();
    if (r != 0)
    {
        U_Send("[ERR] clock to 168MHz failed, code=");
        U_Dec((uint32_t)(-r));
        U_Send("  stay at 16MHz, timing will be slow but still usable\r\n");
    }
    else
    {
        USART1_InitRaw(84000000u, 115200u);
        U_Send("[OK]  clock 168MHz, UART1 re-based on PCLK2 84MHz\r\n");
    }

    GPIO_Clock_All();
    Pins_Build();

    U_Send("[OK]  candidate pins = ");
    U_Dec((uint32_t)s_n);
    U_Send("  (PA skip 9/10/13/14, PB PC PE all, PD ");
#if (SCAN_PORT_D != 0)
    U_Send("all");
#else
    U_Send("skipped");
#endif
    U_Send(")\r\n");

#if (KNOWN_KEYS != 0)
    U_Send("[OK]  PE2~PE5 = already known keys: tagged [known], STILL counted\r\n");
#endif
    U_Send("[OK]  goal = find ");
    U_Dec((uint32_t)TARGET_KEYS);
    U_Send(" keys, PE2~PE5 included, nothing excluded\r\n");

    Step1_Probe();
    Step2_Idle();
    Step3_Loop();

    while (1)
    {
    }
}
