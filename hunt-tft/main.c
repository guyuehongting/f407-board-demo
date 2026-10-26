/*==============================================================================
 * ST7735 定位程序  —— 第 4 版（SCK 已确认 = PB10）
 *
 * #############################################################################
 * #  前提：SCL -> PB10 是你亲眼看到丝印确认的，不再怀疑                    #
 * #                                                                        #
 * #  剩下三个未知数：SDA / CS / DC                                          #
 * #    SDA : 4 线 SPI 的 SDA 必须是 MOSI，SPI2 全芯片只有两个              #
 * #          -> PC3 或 PB15（先试 PC3，你的引脚列表里有 C3 没有 PB15）      #
 * #    CS  : 13 个候选（你列的 12 个 + PB15）                               #
 * #    DC  : 12 个候选（排除掉当 SDA 用的那个）                             #
 * #                                                                        #
 * #  RES 不用找！ST7735 支持软件复位（命令 0x01 SWRESET）。                  #
 * #  测试时把所有候选脚都拉高 —— 不管 RES 是哪个，它一定是「不复位」的高电平。#
 * #############################################################################
 *
 * #############################################################################
 * #  关键设计：绿色会留下来                                                #
 * #                                                                        #
 * #  每个组合初始化成功后**填满绿色**，然后不清屏、不停顿。                  #
 * #    · 组合不对 -> 屏根本没收到东西，还是白的，绿色不变                   #
 * #    · 组合对了 -> 屏变绿，而且**后面的错误组合改不动它，绿色会一直留着** #
 * #                                                                        #
 * #  所以你不用死盯屏幕，扫完抬头看一眼：                                   #
 * #      屏幕变绿了 = 找到了        屏幕还是白 = 没找到                     #
 * #  串口会一行行打印当前组合，屏幕变绿那一刻对应的那行就是答案。           #
 * #############################################################################
 *
 * #############################################################################
 * #  耗时：2 个 SDA 配置 x 169 组 = 338 组，一组约 0.33 秒 = 约 1 分 50 秒   #
 * #############################################################################
 *============================================================================*/

#include "stm32f4xx_hal.h"

/*==============================================================================
 *                            ▼▼▼  开关  ▼▼▼
 *============================================================================*/

/* MODE = 1 : 全量扫 SDA x CS x DC（169 组，56 秒）
   MODE = 2 : 手动确认（把找到的 CS/DC 填进下面，只测这一组）
   MODE = 3 : 定点测试 CS 只试 PC14/PC15，DC 试其余 12 个（37 组，45 秒）
   MODE = 4 : 8 组关键组合 x 8 种颜色
   MODE = 5 : 每组先 Clear_Panel 再测（解决「花屏是粘的」）
   MODE = 6 : 九步诊断，CS=PC15 / DC=PC14
   MODE = 7 : INVON 反显扫描（CS=PC14/PC15 x DC=12 脚）<- 已跑完
   MODE = 8 : 两套 CS/DC 顺序各填 4 种色（旧接线用）
   MODE = 9 : 全新接线 SPI1 最小点亮（拆线重接后用这个）      <- 默认  */
#define MODE            9

/* SPI 时钟分频（花屏可能是 PC14/PC15 在备份域跑不了高速导致的）
 *   0 = /8   -> 5.25 MHz
 *   1 = /16  -> 2.6  MHz
 *   2 = /32  -> 1.3  MHz
 *   3 = /64  -> 656  kHz   <- 默认，最稳；PC13~15 在备份域只能到这个量级  */
#define SPI_BR          3

/* --- MODE 1 用：先试哪个 SDA -------------------------------------------
 *   bit0 = SDA 用 PC3      ← 最可能，默认只跑这个（约 55 秒）
 *   bit1 = SDA 用 PB15
 *   想一次跑完就写 0x03（约 1 分 50 秒）                                   */
#define SDA_MASK        0x01

/* --- MODE 2 用：把扫出来的结果填这里 ------------------------------------ */
#define F_CS_NAME       "PE2"
#define F_CS_PORT       GPIOE
#define F_CS_PIN        GPIO_PIN_2

#define F_DC_NAME       "PE3"
#define F_DC_PORT       GPIOE
#define F_DC_PIN        GPIO_PIN_3

#define F_SDA_PB15      0       /* 0 = SDA 是 PC3    1 = SDA 是 PB15 */

/* --- 面板参数 ------------------------------------------------------------ */
#define TFT_USE_BGR     0
#define GRAM_W          132
#define GRAM_H          162

/* SPI_CR1 的 BR[2:0] 分频位 */
#if   (SPI_BR == 0)
#define BR_BITS         SPI_CR1_BR_1                        /* /8   */
#elif (SPI_BR == 1)
#define BR_BITS         (SPI_CR1_BR_1 | SPI_CR1_BR_0)       /* /16  */
#elif (SPI_BR == 2)
#define BR_BITS         SPI_CR1_BR_2                        /* /32  */
#else
#define BR_BITS         (SPI_CR1_BR_2 | SPI_CR1_BR_0)       /* /64  */
#endif

#define C_BLACK         0x0000
#define C_RED           0xF800
#define C_GREEN         0x07E0
#define C_BLUE          0x001F
#define C_WHITE         0xFFFF

/*==============================================================================
 * 一、底层延时（不依赖 SysTick）
 *============================================================================*/
static void Delay_Loop(uint32_t n)
{
    volatile uint32_t i;
    for (i = 0; i < n; i++)
    {
        __NOP();
    }
}

/*==============================================================================
 * 二、LED（PB6 = LED1）
 *============================================================================*/
static void LED_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    GPIOB->MODER   &= ~(3u << 12);
    GPIOB->MODER   |=  (1u << 12);
    GPIOB->OTYPER  &= ~(1u << 6);
    GPIOB->OSPEEDR |=  (3u << 12);
    GPIOB->PUPDR   &= ~(3u << 12);
}

static void Blink(uint8_t n, uint32_t cnt)
{
    uint8_t i;
    for (i = 0; i < n; i++)
    {
        GPIOB->BSRR = (uint32_t)GPIO_PIN_6;
        Delay_Loop(cnt);
        GPIOB->BSRR = ((uint32_t)GPIO_PIN_6 << 16);
        Delay_Loop(cnt);
    }
}

/*==============================================================================
 * 三、USART1 纯寄存器版
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

static void U_Num3(uint16_t v)
{
    char t[4];
    t[0] = (char)('0' + (v / 100u));
    t[1] = (char)('0' + ((v / 10u) % 10u));
    t[2] = (char)('0' + (v % 10u));
    t[3] = '\0';
    U_Send(t);
}

static void U_Hex32(uint32_t v)
{
    char t[9];
    uint8_t i, nib;
    for (i = 0; i < 8; i++)
    {
        nib = (uint8_t)((v >> ((7u - i) * 4u)) & 0xFu);
        t[i] = (char)((nib < 10u) ? ('0' + nib) : ('A' + nib - 10u));
    }
    t[8] = '\0';
    U_Send(t);
}

/*==============================================================================
 * 四、时钟（失败返回非 0，不死循环）
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
 * 五、候选引脚池
 *   你列的 12 个脚 + PB15（万一是 SDA）+ PB13
 *   运行时把「当前当 SDA 用的脚」排除，剩下的才当 CS / DC 候选
 *============================================================================*/
typedef struct
{
    GPIO_TypeDef *port;
    uint16_t      pin;
    const char   *name;
} PinDef;

#define POOL_N          14

static const PinDef s_pool[POOL_N] =
{
    { GPIOE, GPIO_PIN_2,  "PE2"  },
    { GPIOE, GPIO_PIN_3,  "PE3"  },
    { GPIOE, GPIO_PIN_4,  "PE4"  },
    { GPIOE, GPIO_PIN_5,  "PE5"  },
    { GPIOE, GPIO_PIN_6,  "PE6"  },
    { GPIOC, GPIO_PIN_0,  "PC0"  },
    { GPIOC, GPIO_PIN_1,  "PC1"  },
    { GPIOC, GPIO_PIN_13, "PC13" },
    { GPIOC, GPIO_PIN_14, "PC14" },
    { GPIOC, GPIO_PIN_15, "PC15" },
    { GPIOC, GPIO_PIN_2,  "PC2"  },
    { GPIOC, GPIO_PIN_3,  "PC3"  },
    { GPIOB, GPIO_PIN_15, "PB15" },
    { GPIOB, GPIO_PIN_13, "PB13" },
};

static uint8_t s_line[GRAM_W * 2];
static PinDef  s_dc;

/* 把池子里除 exclude 之外的所有脚都设成推挽输出并拉高
   —— 不管 RES 是哪个脚，它一定是「不复位」的高电平 */
static void Pins_HighExcept(const PinDef *exclude)
{
    GPIO_InitTypeDef gi = {0};
    uint8_t i;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    gi.Mode  = GPIO_MODE_OUTPUT_PP;
    gi.Pull  = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_LOW;

    for (i = 0; i < POOL_N; i++)
    {
        if (exclude != 0 &&
            s_pool[i].port == exclude->port &&
            s_pool[i].pin  == exclude->pin)
        {
            continue;
        }
        gi.Pin = s_pool[i].pin;
        HAL_GPIO_Init(s_pool[i].port, &gi);
        HAL_GPIO_WritePin(s_pool[i].port, s_pool[i].pin, GPIO_PIN_SET);
    }
}

/* 当前 SDA 用的脚 */
static PinDef Sda_Pin(uint8_t sda_pb15)
{
    PinDef p;
    p.port = sda_pb15 ? GPIOB : GPIOC;
    p.pin  = sda_pb15 ? GPIO_PIN_15 : GPIO_PIN_3;
    p.name = sda_pb15 ? "PB15" : "PC3";
    return p;
}

/* 从池子里排除 SDA，得到 CS/DC 候选 */
static uint8_t BuildCand(PinDef *out, const PinDef *sda)
{
    uint8_t i, n = 0;

    for (i = 0; i < POOL_N; i++)
    {
        if (s_pool[i].port == sda->port && s_pool[i].pin == sda->pin)
        {
            continue;
        }
        out[n] = s_pool[i];
        n++;
    }
    return n;
}

/*==============================================================================
 * 六、SPI2 纯寄存器版：SCK 固定 PB10，SDA 可切 PC3 / PB15
 *============================================================================*/
static void SPI2_Init(uint8_t sda_pb15)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

    /* PB10 = SPI2_SCK (AF5)，AFR[1] bits[11:8] */
    GPIOB->MODER   &= ~(3u << 20);
    GPIOB->MODER   |=  (2u << 20);
    GPIOB->OSPEEDR |=  (3u << 20);
    GPIOB->PUPDR   &= ~(3u << 20);
    GPIOB->AFR[1]  &= ~(0xFu << 8);
    GPIOB->AFR[1]  |=  (5u << 8);

    /* 两个候选 SDA 先都设成输入 */
    GPIOC->MODER &= ~(3u << 6);
    GPIOB->MODER &= ~(3u << 30);

    if (sda_pb15)
    {
        GPIOB->MODER   &= ~(3u << 30);
        GPIOB->MODER   |=  (2u << 30);
        GPIOB->OSPEEDR |=  (3u << 30);
        GPIOB->PUPDR   &= ~(3u << 30);
        GPIOB->AFR[1]  &= ~(0xFu << 28);
        GPIOB->AFR[1]  |=  (5u << 28);       /* PB15 = AF5 */
    }
    else
    {
        GPIOC->MODER   &= ~(3u << 6);
        GPIOC->MODER   |=  (2u << 6);
        GPIOC->OSPEEDR |=  (3u << 6);
        GPIOC->PUPDR   &= ~(3u << 6);
        GPIOC->AFR[0]  &= ~(0xFu << 12);
        GPIOC->AFR[0]  |=  (5u << 12);       /* PC3 = AF5 */
    }

    /* MSTR | SSM | SSI | BR=010(/8) | SPE，Mode 0 */
    SPI2->CR1 = 0;
    SPI2->CR2 = 0;
    SPI2->I2SCFGR &= (uint16_t)~SPI_I2SCFGR_I2SMOD;
    SPI2->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI
              | BR_BITS | SPI_CR1_SPE;
}

static void SPI2_Send(const uint8_t *buf, uint16_t len)
{
    uint16_t i;

    for (i = 0; i < len; i++)
    {
        while ((SPI2->SR & SPI_SR_TXE) == 0)
        {
        }
        *(__IO uint8_t *)&SPI2->DR = buf[i];

        while ((SPI2->SR & SPI_SR_RXNE) == 0)
        {
        }
        (void)*(__IO uint8_t *)&SPI2->DR;
    }

    while ((SPI2->SR & SPI_SR_BSY) != 0)
    {
    }
}

/*==============================================================================
 * 七、ST7735 底层
 *   TFT_Cmd / TFT_Data 只动 DC，**不碰 CS**，CS 由调用者控制。
 *   这样既能测「CS 接某个脚」，也能测「CS 模块内部接地」。
 *============================================================================*/
static void TFT_Cmd(uint8_t cmd)
{
    HAL_GPIO_WritePin(s_dc.port, s_dc.pin, GPIO_PIN_RESET);
    SPI2_Send(&cmd, 1);
}

static void TFT_Data(const uint8_t *buf, uint16_t len)
{
    HAL_GPIO_WritePin(s_dc.port, s_dc.pin, GPIO_PIN_SET);
    SPI2_Send(buf, len);
}

static void TFT_Data1(uint8_t v)
{
    TFT_Data(&v, 1);
}

static void TFT_WindowFull(void)
{
    uint8_t d[4];

    TFT_Cmd(0x2A);
    d[0] = 0; d[1] = 0;
    d[2] = 0; d[3] = (uint8_t)(GRAM_W - 1);
    TFT_Data(d, 4);

    TFT_Cmd(0x2B);
    d[0] = 0; d[1] = 0;
    d[2] = 0; d[3] = (uint8_t)(GRAM_H - 1);
    TFT_Data(d, 4);

    TFT_Cmd(0x2C);
}

static void TFT_FillAll(uint16_t color)
{
    uint16_t i, h;

    for (i = 0; i < GRAM_W; i++)
    {
        s_line[i * 2]     = (uint8_t)(color >> 8);
        s_line[i * 2 + 1] = (uint8_t)(color & 0xFF);
    }

    TFT_WindowFull();
    /* ★ 2026-10-25 修的根本 bug：
     *   TFT_WindowFull 最后一句是 TFT_Cmd(0x2C)，它把 DC 拉成了**低**。
     *   这里如果直接 SPI2_Send，像素数据就会带着 DC=低 发出去 ——
     *   面板会把每个颜色字节当成「命令」解释，于是永远花屏，永远不出纯色。
     *   必须先把 DC 拉高，再发像素数据。 */
    HAL_GPIO_WritePin(s_dc.port, s_dc.pin, GPIO_PIN_SET);

    for (h = 0; h < GRAM_H; h++)
    {
        SPI2_Send(s_line, (uint16_t)(GRAM_W * 2));
    }
}

/* 一套完整初始化（软件复位，不需要 RES 脚），最后填绿色 */
static void TFT_InitOnly(void)
{
    uint8_t d3[3];
    uint8_t d2[2];

    TFT_Cmd(0x01);  HAL_Delay(120);         /* SWRESET */
    TFT_Cmd(0x11);  HAL_Delay(120);         /* SLPOUT */

    TFT_Cmd(0xB1);  d3[0]=0x01; d3[1]=0x2C; d3[2]=0x2D; TFT_Data(d3, 3);
    TFT_Cmd(0xC0);  d3[0]=0xA2; d3[1]=0x02; d3[2]=0x84; TFT_Data(d3, 3);
    TFT_Cmd(0xC1);  TFT_Data1(0xC5);
    TFT_Cmd(0xC2);  d2[0]=0x0A; d2[1]=0x00; TFT_Data(d2, 2);
    TFT_Cmd(0xC3);  d2[0]=0x8A; d2[1]=0x2A; TFT_Data(d2, 2);
    TFT_Cmd(0xC4);  d2[0]=0x8A; d2[1]=0xEE; TFT_Data(d2, 2);
    TFT_Cmd(0xC5);  TFT_Data1(0x0E);

    TFT_Cmd(0x20);                          /* INVOFF */
#if TFT_USE_BGR
    TFT_Cmd(0x36);  TFT_Data1(0xC8);        /* MX|MY|BGR */
#else
    TFT_Cmd(0x36);  TFT_Data1(0xC0);        /* MX|MY */
#endif
    TFT_Cmd(0x3A);  TFT_Data1(0x05);        /* COLMOD 16bit */
    TFT_Cmd(0x13);  HAL_Delay(10);          /* NORON */
    TFT_Cmd(0x29);  HAL_Delay(20);          /* DISPON */

}

/* 初始化 + 填色（原来那个"填绿"的行为） */
static void TFT_InitFill(uint16_t color)
{
    TFT_InitOnly();
    TFT_FillAll(color);
}

/*==============================================================================
 * 清屏：把所有候选脚一起拉低 150ms 再一起拉高
 *
 *   原理：RES 一定在这 12 根线里（模块 6 根信号线之一）。
 *   全部拉低 = RES 被拉低 = 面板进入硬复位 = 显示关闭 = 屏幕回到白屏。
 *   然后再全部拉高，面板出复位，处于「已上电但没初始化」的干净状态。
 *
 *   ★ 这一步是为了解决「花屏是粘的」这个问题：
 *     一旦面板被点亮并显示乱码，后面错误的组合改不动它，
 *     所有测试看起来都是花屏，根本分不清哪一组真的生效。
 *   清屏之后，每一组的判据变成干净的：
 *       变绿  = 这一组对了
 *       还是白 = 这一组不对
 *============================================================================*/
static void Clear_Panel(const PinDef *sda)
{
    GPIO_InitTypeDef gi = {0};
    uint8_t i;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    gi.Mode  = GPIO_MODE_OUTPUT_PP;
    gi.Pull  = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_LOW;

    /* 全部拉低：RES 被拉低 -> 面板复位 -> 白屏 */
    for (i = 0; i < POOL_N; i++)
    {
        if (sda != 0 && s_pool[i].port == sda->port && s_pool[i].pin == sda->pin)
        {
            continue;                   /* SDA 保持 SPI 复用，不能当 GPIO 动 */
        }
        gi.Pin = s_pool[i].pin;
        HAL_GPIO_Init(s_pool[i].port, &gi);
        HAL_GPIO_WritePin(s_pool[i].port, s_pool[i].pin, GPIO_PIN_RESET);
    }
    HAL_Delay(150);

    /* 全部拉高：出复位 */
    for (i = 0; i < POOL_N; i++)
    {
        if (sda != 0 && s_pool[i].port == sda->port && s_pool[i].pin == sda->pin)
        {
            continue;
        }
        HAL_GPIO_WritePin(s_pool[i].port, s_pool[i].pin, GPIO_PIN_SET);
    }
    HAL_Delay(150);
}

/* 测一组并填指定颜色：CS 传 0 表示模块内部把 CS 接地了，不用我们管 */
static void Test_ComboC(const PinDef *cs, const PinDef *dc, const PinDef *sda, uint16_t color)
{
    Pins_HighExcept(sda);       /* 除 SDA 外全部拉高，RES 自然为高 */

    if (cs != 0)
    {
        HAL_GPIO_WritePin(cs->port, cs->pin, GPIO_PIN_RESET);
    }

    s_dc = *dc;
    TFT_InitFill(color);
}

/* 默认填绿 */
static void Test_Combo(const PinDef *cs, const PinDef *dc, const PinDef *sda)
{
    Test_ComboC(cs, dc, sda, C_GREEN);
}

/*==============================================================================
 * 八、Error_Handler（只为满足 stm32f4xx_it.c，永不调用）
 *============================================================================*/
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

/*==============================================================================
 * 九、MODE 1：自动扫 SDA x CS x DC
 *============================================================================*/
static void Mode1_Scan(void)
{
    PinDef  cand[POOL_N];
    PinDef  sda;
    uint8_t nCand;
    uint8_t ci, di, s;
    uint16_t idx = 0;

    for (s = 0; s < 2; s++)
    {
        if ((SDA_MASK & (1u << s)) == 0)
        {
            continue;
        }

        sda = Sda_Pin((uint8_t)(s == 1));
        SPI2_Init((uint8_t)(s == 1));
        nCand = BuildCand(cand, &sda);

        U_Send("\r\n========================================\r\n");
        U_Send(" SDA = ");
        U_Send(sda.name);
        U_Send("   SCK = PB10 (confirmed)\r\n");
        U_Send(" CS/DC candidates = ");
        U_Num3(nCand);
        U_Send(" pins\r\n");
        U_Send(" combos = ");
        U_Num3((uint16_t)(nCand * nCand));
        U_Send("   about ");
        U_Num3((uint16_t)((nCand * nCand) / 3u));
        U_Send(" seconds\r\n");
        U_Send("========================================\r\n");

        /* --- 先测「CS 模块内部接地」：不拉低任何脚 --- */
        for (di = 0; di < nCand; di++)
        {
            idx++;
            U_Send("#");
            U_Num3(idx);
            U_Send("  SDA=");  U_Send(sda.name);
            U_Send("  CS=none  DC="); U_Send(cand[di].name);
            U_Send("\r\n");
            Test_Combo(0, &cand[di], &sda);
        }

        /* --- 再测 CS 接某个脚 --- */
        for (ci = 0; ci < nCand; ci++)
        {
            for (di = 0; di < nCand; di++)
            {
                if (di == ci)
                {
                    continue;
                }
                idx++;
                U_Send("#");
                U_Num3(idx);
                U_Send("  SDA=");  U_Send(sda.name);
                U_Send("  CS=");   U_Send(cand[ci].name);
                U_Send("  DC=");   U_Send(cand[di].name);
                U_Send("\r\n");
                Test_Combo(&cand[ci], &cand[di], &sda);
            }
        }

        U_Send("\r\n---- SDA=");
        U_Send(sda.name);
        U_Send(" done ----\r\n");
        U_Send(">>> LOOK AT THE SCREEN: is it GREEN ?\r\n\r\n");
    }

    U_Send("\r\n########################################\r\n");
    U_Send("# scan done. total ");
    U_Num3(idx);
    U_Send(" combos\r\n");
    U_Send("#\r\n");
    U_Send("# SCREEN GREEN  -> found. the last ");
    U_Send("\"SDA=.. CS=.. DC=..\"\r\n");
    U_Send("#                  line BEFORE it turned green is the answer\r\n");
    U_Send("#\r\n");
    U_Send("# SCREEN STILL WHITE -> try in this order:\r\n");
    U_Send("#   1. SDA_MASK = 0x03   (also try SDA = PB15)\r\n");
    U_Send("#   2. check the module VCC is really 3.3V\r\n");
    U_Send("#   3. check SDA wire is actually connected\r\n");
    U_Send("#   4. the panel may not be ST7735\r\n");
    U_Send("########################################\r\n");

    while (1)
    {
        Blink(1, 3000000);
    }
}

/*==============================================================================
 * 十、MODE 2：手动确认一组，跑颜色测试
 *============================================================================*/
static void Mode2_Confirm(void)
{
    PinDef cs;
    PinDef dc;
    PinDef sda;

    cs.port = F_CS_PORT;  cs.pin = F_CS_PIN;  cs.name = F_CS_NAME;
    dc.port = F_DC_PORT;  dc.pin = F_DC_PIN;  dc.name = F_DC_NAME;
    sda = Sda_Pin(F_SDA_PB15);

    U_Send("\r\n========================================\r\n");
    U_Send(" CONFIRM : SDA="); U_Send(sda.name);
    U_Send("  CS=");           U_Send(cs.name);
    U_Send("  DC=");           U_Send(dc.name);
    U_Send("\r\n========================================\r\n");

    SPI2_Init(F_SDA_PB15);

    while (1)
    {
        Test_Combo(&cs, &dc, &sda);
        HAL_Delay(1200);

        TFT_FillAll(C_RED);
        HAL_Delay(1200);
        U_Send("RED\r\n");

        TFT_FillAll(C_GREEN);
        HAL_Delay(1200);
        U_Send("GREEN\r\n");

        TFT_FillAll(C_BLUE);
        HAL_Delay(1200);
        U_Send("BLUE\r\n");

        TFT_FillAll(C_WHITE);
        HAL_Delay(1200);
        U_Send("WHITE\r\n");

        TFT_FillAll(C_BLACK);
        HAL_Delay(1200);
        U_Send("BLACK\r\n");
    }
}

/*==============================================================================
 * 九点五、MODE 3：定点测试
 *
 *   已经确定：SCK = PB10、SDA = PC3（花屏证明了数据确实到了屏上）
 *   而且花屏出现在 #110~#127，那一段正好是 CS=PC14 / CS=PC15
 *   -> CS 就是 PC14 或 PC15 之一，DC 是另一个
 *
 *   所以只测：CS 取 {PC14, PC15, 不接}，DC 取其余 12 个
 *   37 组，每组停 0.9 秒，方便你逐个看清楚
 *============================================================================*/
static void Mode3_Focus(void)
{
    PinDef  cand[POOL_N];
    PinDef  sda;
    PinDef  csl[2];
    uint8_t nCand;
    uint8_t ci, di;
    uint16_t idx = 0;

    sda = Sda_Pin(0);                 /* PC3 */
    SPI2_Init(0);
    nCand = BuildCand(cand, &sda);

    csl[0].port = GPIOC; csl[0].pin = GPIO_PIN_14; csl[0].name = "PC14";
    csl[1].port = GPIOC; csl[1].pin = GPIO_PIN_15; csl[1].name = "PC15";

    U_Send("\r\n========================================\r\n");
    U_Send(" MODE 3 : focus test\r\n");
    U_Send(" SCK=PB10  SDA=PC3  (both confirmed)\r\n");
    U_Send(" SPI divider = ");
    if (SPI_BR == 0)
    {
        U_Send("/8 5.25MHz");
    }
    else if (SPI_BR == 1)
    {
        U_Send("/16 2.6MHz");
    }
    else if (SPI_BR == 2)
    {
        U_Send("/32 1.3MHz");
    }
    else
    {
        U_Send("/64 656kHz");
    }
    U_Send("\r\n");
    U_Send(" CS in {PC14, PC15, none} x DC in ");
    U_Num3(nCand);
    U_Send(" pins\r\n");
    U_Send(" 0.9 sec per combo, watch the SCREEN\r\n");
    U_Send("========================================\r\n");

    /* ---- CS = PC14 / PC15 ---- */
    for (ci = 0; ci < 2; ci++)
    {
        for (di = 0; di < nCand; di++)
        {
            if (cand[di].port == csl[ci].port && cand[di].pin == csl[ci].pin)
            {
                continue;
            }
            idx++;
            U_Send("#");
            U_Num3(idx);
            U_Send("  CS=");  U_Send(csl[ci].name);
            U_Send("  DC=");  U_Send(cand[di].name);
            U_Send("\r\n");
            Test_Combo(&csl[ci], &cand[di], &sda);
            HAL_Delay(900);
        }
    }

    /* ---- CS 不接（模块内部接地）---- */
    for (di = 0; di < nCand; di++)
    {
        idx++;
        U_Send("#");
        U_Num3(idx);
        U_Send("  CS=none  DC=");  U_Send(cand[di].name);
        U_Send("\r\n");
        Test_Combo(0, &cand[di], &sda);
        HAL_Delay(900);
    }

    U_Send("\r\n########################################\r\n");
    U_Send("# focus test done, ");
    U_Num3(idx);
    U_Send(" combos\r\n");
    U_Send("# WHICH number turned the screen GREEN ?\r\n");
    U_Send("# WHICH numbers showed GARBAGE ?\r\n");
    U_Send("# tell me both\r\n");
    U_Send("# if ALL white: SPI_BR = 3 (slower), rebuild\r\n");
    U_Send("########################################\r\n");

    while (1)
    {
        Blink(1, 3000000);
    }
}

/*==============================================================================
 * 九点八、MODE 4：8 个关键组合，每个组合一种颜色
 *
 *   已推出的结论：
 *     · 花屏从 #013（CS=PC15 第一组）开始 -> PC15 被拉低时面板才会执行命令
 *     · 那说明 PC15 就是 DC：它被当 CS 拉低 = 永远命令模式 -> 参数全乱 -> 花屏
 *     · #001~#012（CS=PC14）时 PC15 被拉高 = 数据模式 -> 面板没醒 -> 纯白
 *     · 于是 CS 只可能是在模块内部直接接地了，不由 MCU 控制
 *
 *   最看好的组合：CS 不接 + DC = PC15
 *
 *   判据：每个组合填一种不同的颜色，屏幕最后变成哪种颜色，就是哪一组成功。
 *         因为组合错误时改不动真屏，所以颜色不会被后面的组合覆盖掉。
 *============================================================================*/
typedef struct
{
    uint8_t       cs_none;
    GPIO_TypeDef *cs_port;
    uint16_t      cs_pin;
    const char   *cs_name;
    GPIO_TypeDef *dc_port;
    uint16_t      dc_pin;
    const char   *dc_name;
    uint16_t      color;
    const char   *color_name;
} ComboDef;

#define COMBO_N         8

static const ComboDef s_combo[COMBO_N] =
{
    { 1, 0,      0,            "none", GPIOC, GPIO_PIN_15, "PC15", 0xF800, "RED"     },
    { 1, 0,      0,            "none", GPIOC, GPIO_PIN_14, "PC14", 0x07E0, "GREEN"   },
    { 1, 0,      0,            "none", GPIOC, GPIO_PIN_13, "PC13", 0x001F, "BLUE"    },
    { 1, 0,      0,            "none", GPIOC, GPIO_PIN_1,  "PC1",  0xFFFF, "WHITE"   },
    { 1, 0,      0,            "none", GPIOC, GPIO_PIN_0,  "PC0",  0xFFE0, "YELLOW"  },
    { 1, 0,      0,            "none", GPIOE, GPIO_PIN_6,  "PE6",  0x07FF, "CYAN"    },
    { 0, GPIOC, GPIO_PIN_14, "PC14", GPIOC, GPIO_PIN_15, "PC15", 0xF81F, "MAGENTA" },
    { 0, GPIOC, GPIO_PIN_15, "PC15", GPIOC, GPIO_PIN_14, "PC14", 0xFD20, "ORANGE"  },
};

static void Mode4_KeyCombos(void)
{
    PinDef  sda;
    PinDef  csp;
    PinDef  dcp;
    uint8_t i;

    sda = Sda_Pin(0);                 /* PC3 */
    SPI2_Init(0);

    U_Send("\r\n========================================\r\n");
    U_Send(" MODE 4 : 8 key combos, 8 colours\r\n");
    U_Send(" SCK=PB10  SDA=PC3\r\n");
    U_Send(" SPI divider = ");
    if (SPI_BR == 0)
    {
        U_Send("/8 5.25MHz");
    }
    else if (SPI_BR == 1)
    {
        U_Send("/16 2.6MHz");
    }
    else if (SPI_BR == 2)
    {
        U_Send("/32 1.3MHz");
    }
    else
    {
        U_Send("/64 656kHz");
    }
    U_Send("\r\n each combo holds 4 sec\r\n");
    U_Send(" WATCH THE SCREEN COLOUR\r\n");
    U_Send("========================================\r\n");

    for (i = 0; i < COMBO_N; i++)
    {
        U_Send("\r\n>>> #");
        U_Num3((uint16_t)(i + 1));
        U_Send("  CS=");
        U_Send(s_combo[i].cs_name);
        U_Send("  DC=");
        U_Send(s_combo[i].dc_name);
        U_Send("  -> screen should be ");
        U_Send(s_combo[i].color_name);
        U_Send("\r\n");

        csp.port = s_combo[i].cs_port;
        csp.pin  = s_combo[i].cs_pin;
        csp.name = s_combo[i].cs_name;

        dcp.port = s_combo[i].dc_port;
        dcp.pin  = s_combo[i].dc_pin;
        dcp.name = s_combo[i].dc_name;

        if (s_combo[i].cs_none)
        {
            Test_ComboC(0, &dcp, &sda, s_combo[i].color);
        }
        else
        {
            Test_ComboC(&csp, &dcp, &sda, s_combo[i].color);
        }

        HAL_Delay(4000);
    }

    U_Send("\r\n########################################\r\n");
    U_Send("# all 8 done\r\n");
    U_Send("# what colour is the screen now ?\r\n");
    U_Send("#   RED     -> #1  CS=none DC=PC15\r\n");
    U_Send("#   GREEN   -> #2  CS=none DC=PC14\r\n");
    U_Send("#   BLUE    -> #3  CS=none DC=PC13\r\n");
    U_Send("#   WHITE   -> #4  CS=none DC=PC1\r\n");
    U_Send("#   YELLOW  -> #5  CS=none DC=PC0\r\n");
    U_Send("#   CYAN    -> #6  CS=none DC=PE6\r\n");
    U_Send("#   MAGENTA -> #7  CS=PC14 DC=PC15\r\n");
    U_Send("#   ORANGE  -> #8  CS=PC15 DC=PC14\r\n");
    U_Send("#   GARBAGE / WHITE / unchanged -> none worked\r\n");
    U_Send("#     then set SPI_BR = 3 and rebuild\r\n");
    U_Send("########################################\r\n");

    while (1)
    {
        Blink(1, 3000000);
    }
}

/*==============================================================================
 * 九点九、MODE 5：先清屏，再测，判据干净
 *
 *   已知：
 *     · #1~#6（CS=none）全白  -> CS 不是内部接地，必须由 MCU 拉低
 *     · #7（CS=PC14, DC=PC15）花屏 -> PC14 拉低时面板被选中
 *     · 但花屏是"粘"的，一旦点亮，后面的错误组合改不动它
 *
 *   所以这一版每一组之前都先 Clear_Panel()：
 *       全拉低 -> RES 被拉低 -> 面板复位 -> 白屏 -> 全拉高
 *   判据变成：变绿 = 对，还是白 = 错。不再被残留干扰。
 *
 *   组合：CS 取 {PC14, PC15}，DC 取其余 12 个 = 24 组，每组 2.5 秒，共约 60 秒
 *============================================================================*/
static void Mode5_CleanScan(void)
{
    PinDef  cand[POOL_N];
    PinDef  sda;
    PinDef  csl[2];
    uint8_t nCand;
    uint8_t ci, di;
    uint16_t idx = 0;

    sda = Sda_Pin(0);                 /* PC3 */
    SPI2_Init(0);
    nCand = BuildCand(cand, &sda);

    csl[0].port = GPIOC; csl[0].pin = GPIO_PIN_14; csl[0].name = "PC14";
    csl[1].port = GPIOC; csl[1].pin = GPIO_PIN_15; csl[1].name = "PC15";

    U_Send("\r\n========================================\r\n");
    U_Send(" MODE 5 : clear then test\r\n");
    U_Send(" SCK=PB10  SDA=PC3\r\n");
    U_Send(" each combo: screen goes WHITE, then\r\n");
    U_Send("   GREEN = this combo is correct\r\n");
    U_Send("   still WHITE = wrong\r\n");
    U_Send(" 24 combos, 2.5 sec each, about 60 sec\r\n");
    U_Send("========================================\r\n");

    for (ci = 0; ci < 2; ci++)
    {
        for (di = 0; di < nCand; di++)
        {
            if (cand[di].port == csl[ci].port && cand[di].pin == csl[ci].pin)
            {
                continue;
            }
            idx++;
            U_Send("#");
            U_Num3(idx);
            U_Send("  CS=");  U_Send(csl[ci].name);
            U_Send("  DC=");  U_Send(cand[di].name);
            U_Send("\r\n");

            Clear_Panel(&sda);
            Test_ComboC(&csl[ci], &cand[di], &sda, C_GREEN);
            HAL_Delay(2500);
        }
    }

    U_Send("\r\n########################################\r\n");
    U_Send("# done. did ANY of them turn GREEN ?\r\n");
    U_Send("# if yes -> tell me the number\r\n");
    U_Send("# if all white -> tell me, next step is\r\n");
    U_Send("#   measuring the module pins by hand\r\n");
    U_Send("########################################\r\n");

    while (1)
    {
        Blink(1, 3000000);
    }
}

/*==============================================================================
 * 九点九五、MODE 6：确诊模式（九步，CS=PC15 / DC=PC14）
 *
 *   引脚推导（按 2026-10-25 20:50 那一轮实测反推）：
 *     MODE 6 用 CS=PC14 / DC=PC15 跑出来的现象是
 *       STEP 2 就花、STEP 3/4 各闪一下、STEP 5~9 完全不变
 *     -> 命令全收到（SWRESET / SLPOUT / DISPON / INVON / INVOFF 都生效）
 *     -> 参数和像素全丢（0x2A/0x2B 的参数没了 -> 窗口停在 1 像素 -> 填色看不出变化）
 *     只有一种组合会这样：真 DC 恒定低（永远命令模式）+ 真 CS 在跟着跳（数据期取消选中）
 *     也就是：我们当 CS 拉低的 PC14 其实是 DC，我们当 DC 翻转的 PC15 其实是 CS。
 *   => CS = PC15，DC = PC14。
 *
 *   本函数按这个组合跑九步：
 *     STEP 3 的 INVON 确认命令通路（不依赖 MADCTL/COLMOD/窗口）
 *     STEP 5~9 确认像素通路（DC 的 bug 已修，应该出纯色）
 *============================================================================*/
static void Mode6_Diagnose(void)
{
    PinDef cs;
    PinDef dc;
    PinDef sda;
    uint8_t round;

    sda = Sda_Pin(0);                 /* PC3 */
    cs.port = GPIOC; cs.pin = GPIO_PIN_15; cs.name = "PC15";
    dc.port = GPIOC; dc.pin = GPIO_PIN_14; dc.name = "PC14";
    SPI2_Init(0);

    U_Send("\r\n========================================\r\n");
    U_Send(" MODE 6 : diagnose\r\n");
    U_Send(" SCK=PB10  SDA=PC3  CS=PC15  DC=PC14\r\n");
    U_Send(" SPI divider = ");
    if (SPI_BR == 0)
    {
        U_Send("/8 5.25MHz");
    }
    else if (SPI_BR == 1)
    {
        U_Send("/16 2.6MHz");
    }
    else if (SPI_BR == 2)
    {
        U_Send("/32 1.3MHz");
    }
    else
    {
        U_Send("/64 656kHz");
    }
    U_Send("\r\n 9 steps x 2.5 sec, 2 rounds\r\n");
    U_Send("========================================\r\n");

    for (round = 0; round < 2; round++)
    {
        U_Send("\r\n===== ROUND ");
        U_Num3((uint16_t)(round + 1));
        U_Send(" =====\r\n");

        /* ---- 1 清屏 ---- */
        Clear_Panel(&sda);
        U_Send("STEP 1  clear        -> screen should be WHITE\r\n");
        HAL_Delay(2500);

        /* ---- 2 初始化 + DISPON，不填色 ---- */
        Pins_HighExcept(&sda);
        HAL_GPIO_WritePin(cs.port, cs.pin, GPIO_PIN_RESET);
        s_dc = dc;
        TFT_InitOnly();
        U_Send("STEP 2  init+DISPON  -> white ? garbage ?\r\n");
        HAL_Delay(2500);

        /* ---- 3 反显：不依赖任何面板参数 ---- */
        TFT_Cmd(0x21);
        U_Send("STEP 3  INVON        -> BLACK (any change = init OK)\r\n");
        HAL_Delay(2500);

        /* ---- 4 取消反显 ---- */
        TFT_Cmd(0x20);
        U_Send("STEP 4  INVOFF       -> back to WHITE\r\n");
        HAL_Delay(2500);

        /* ---- 5~9 填色 ---- */
        TFT_FillAll(C_RED);
        U_Send("STEP 5  fill RED\r\n");
        HAL_Delay(2500);

        TFT_FillAll(C_GREEN);
        U_Send("STEP 6  fill GREEN\r\n");
        HAL_Delay(2500);

        TFT_FillAll(C_BLUE);
        U_Send("STEP 7  fill BLUE\r\n");
        HAL_Delay(2500);

        TFT_FillAll(C_BLACK);
        U_Send("STEP 8  fill BLACK\r\n");
        HAL_Delay(2500);

        TFT_FillAll(C_WHITE);
        U_Send("STEP 9  fill WHITE\r\n");
        HAL_Delay(2500);
    }

    U_Send("\r\n########################################\r\n");
    U_Send("# DC bug fixed: pixel bytes now sent with DC HIGH\r\n");
    U_Send("# expect STEP5 RED 6 GREEN 7 BLUE 8 BLACK 9 WHITE\r\n");
    U_Send("# report:\r\n");
    U_Send("#  1) STEP 5-9: solid colour or still garbage ?\r\n");
    U_Send("#  2) if solid but wrong colour: which one looks like what\r\n");
    U_Send("#  3) if still garbage: describe the pattern\r\n");
    U_Send("########################################\r\n");

    while (1)
    {
        Blink(1, 3000000);
    }
}

/*==============================================================================
 * 九点九六、MODE 7：INVON 反显扫描（最决定性的一次测试）
 *
 *   为什么用它替代继续猜：
 *     0x21 INVON 作用于整块面板，**不依赖 MADCTL / COLMOD / 窗口 / 行列偏移**。
 *     所以它只有一个判据：
 *         这一组停的 1.5 秒里，屏幕变黑了没有？
 *     变黑 = 命令真的进了面板 = CS 和 DC 同时对了。
 *     这条判据不依赖「花成什么样」的肉眼描述，比 MODE 4/5 可靠得多。
 *
 *   扫什么：
 *     CS 只试 PC14 / PC15（MODE 5 已经收窄到这两个）
 *     DC 试池子里除 SDA(PC3) 和当前 CS 之外的 12 个脚
 *     -> 2 x 12 = 24 组，每组约 3.5 秒，总约 90 秒
 *
 *   每组流程：
 *     Clear_Panel  -> 硬复位回白屏（解决「花屏是粘的」）
 *     Pins_HighExcept -> 除 SDA 外全拉高（RES 自然为高）
 *     CS 拉低
 *     完整初始化 -> 发 0x21 INVON -> 停 1.5 秒 -> 发 0x20 INVOFF
 *
 *   找到之后：把那一组的 CS / DC 填进 Mode6_Diagnose，再把 MODE 改成 6，
 *   用九步诊断检查像素通路（MADCTL / COLMOD / 偏移）。
 *============================================================================*/
static void Mode7_InvScan(void)
{
    PinDef cs_list[2];
    PinDef sda;
    PinDef dc;
    uint8_t c;
    uint8_t i;
    uint16_t idx;

    sda = Sda_Pin(0);                 /* PC3 */

    cs_list[0].port = GPIOC; cs_list[0].pin = GPIO_PIN_14; cs_list[0].name = "PC14";
    cs_list[1].port = GPIOC; cs_list[1].pin = GPIO_PIN_15; cs_list[1].name = "PC15";

    SPI2_Init(0);

    U_Send("\r\n========================================\r\n");
    U_Send(" MODE 7 : INVON scan\r\n");
    U_Send(" SCK=PB10  SDA=PC3\r\n");
    U_Send(" CS = PC14 or PC15\r\n");
    U_Send(" DC = 12 pins x 2 = 24 groups\r\n");
    U_Send(" watch: does it turn BLACK or CHANGE ?\r\n");
    U_Send("========================================\r\n");

    idx = 0;
    for (c = 0; c < 2; c++)
    {
        for (i = 0; i < POOL_N; i++)
        {
            /* 跳过 SDA（PC3）*/
            if (s_pool[i].port == sda.port && s_pool[i].pin == sda.pin)
            {
                continue;
            }
            /* 跳过当前这一组的 CS */
            if (s_pool[i].port == cs_list[c].port && s_pool[i].pin == cs_list[c].pin)
            {
                continue;
            }

            dc = s_pool[i];
            idx++;

            Clear_Panel(&sda);
            Pins_HighExcept(&sda);
            HAL_GPIO_WritePin(cs_list[c].port, cs_list[c].pin, GPIO_PIN_RESET);
            s_dc = dc;

            TFT_InitOnly();
            TFT_Cmd(0x21);                       /* INVON */

            U_Send("#");
            U_Num3(idx);
            U_Send("  CS=");
            U_Send(cs_list[c].name);
            U_Send("  DC=");
            U_Send(dc.name);
            U_Send("  -> BLACK / CHANGED ?\r\n");

            HAL_Delay(1500);

            TFT_Cmd(0x20);                       /* INVOFF，恢复给下一组 */
        }
    }

    U_Send("\r\n########################################\r\n");
    U_Send("# scan done. tell me:\r\n");
    U_Send("#   1) which number turned it BLACK (or changed it) ?\r\n");
    U_Send("#   2) if none: did ANY number change the screen at all ?\r\n");
    U_Send("# then set MODE = 6 and put that CS/DC into Mode6_Diagnose\r\n");
    U_Send("########################################\r\n");

    while (1)
    {
        Blink(1, 3000000);
    }
}

/*==============================================================================
 * 九点九七、MODE 8：两套引脚各填一遍色（一次烧录就能定案）
 *
 *   为什么还要这个：
 *     MODE 6（CS=PC14 / DC=PC15）的现象是「命令全通、参数和像素全丢」——
 *       STEP 2 就花（DISPON 收到，GRAM 是上电随机值）
 *       STEP 3 INVON 闪一下（命令收到）
 *       STEP 4 INVOFF 再闪一下（命令收到）
 *       STEP 5~9 一点不变（0x2A/0x2B 的参数丢了 -> 窗口停在 1 个像素 -> 看不出来）
 *     这个组合正是「真 DC 恒定低、真 CS 在跟着 DC 跳」的样子：
 *       我们拿 PC14 当 CS 恒定拉低 = 真 DC 恒低 = 永远命令模式（所以命令能进）
 *       我们拿 PC15 当 DC 翻转    = 真 CS 在跳   = 数据期间被取消选中（所以数据全丢）
 *     => 说明真 DC = PC14，真 CS = PC15，跟 MODE 6 里写的正好相反。
 *
 *   与其再赌一次，不如一次烧录把两套顺序都跑一遍：
 *     PHASE A：CS=PC15  DC=PC14   -> 红 绿 蓝 白
 *     PHASE B：CS=PC14  DC=PC15   -> 红 绿 蓝 白
 *   哪一半出现纯色，哪一半就是对的。
 *============================================================================*/
static void Mode8_BothOrders(void)
{
    PinDef  sda;
    PinDef  cs;
    PinDef  dc;
    uint8_t ph;
    uint8_t ci;

    sda = Sda_Pin(0);                 /* PC3 */
    SPI2_Init(0);

    U_Send("\r\n========================================\r\n");
    U_Send(" MODE 8 : both CS/DC orders\r\n");
    U_Send(" SCK=PB10  SDA=PC3\r\n");
    U_Send(" PHASE A  CS=PC15  DC=PC14\r\n");
    U_Send(" PHASE B  CS=PC14  DC=PC15\r\n");
    U_Send(" 4 colours x 2 sec per phase\r\n");
    U_Send(" solid colour = that phase is CORRECT\r\n");
    U_Send("========================================\r\n");

    for (ph = 0; ph < 2; ph++)
    {
        if (ph == 0)
        {
            cs.port = GPIOC; cs.pin = GPIO_PIN_15; cs.name = "PC15";
            dc.port = GPIOC; dc.pin = GPIO_PIN_14; dc.name = "PC14";
        }
        else
        {
            cs.port = GPIOC; cs.pin = GPIO_PIN_14; cs.name = "PC14";
            dc.port = GPIOC; dc.pin = GPIO_PIN_15; dc.name = "PC15";
        }

        U_Send("\r\n===== PHASE ");
        U_Send(ph == 0 ? "A  CS=PC15  DC=PC14" : "B  CS=PC14  DC=PC15");
        U_Send(" =====\r\n");

        for (ci = 0; ci < 4; ci++)
        {
            Clear_Panel(&sda);
            Pins_HighExcept(&sda);
            HAL_GPIO_WritePin(cs.port, cs.pin, GPIO_PIN_RESET);
            s_dc = dc;

            if (ci == 0)
            {
                TFT_InitFill(C_RED);
                U_Send("  fill RED\r\n");
            }
            else if (ci == 1)
            {
                TFT_InitFill(C_GREEN);
                U_Send("  fill GREEN\r\n");
            }
            else if (ci == 2)
            {
                TFT_InitFill(C_BLUE);
                U_Send("  fill BLUE\r\n");
            }
            else
            {
                TFT_InitFill(C_WHITE);
                U_Send("  fill WHITE\r\n");
            }

            HAL_Delay(2000);
        }
    }

    U_Send("\r\n########################################\r\n");
    U_Send("# tell me: PHASE A solid, or PHASE B solid ?\r\n");
    U_Send("#   A solid -> CS=PC15  DC=PC14\r\n");
    U_Send("#   B solid -> CS=PC14  DC=PC15\r\n");
    U_Send("#   neither -> data path still broken, tell me\r\n");
    U_Send("########################################\r\n");

    while (1)
    {
        Blink(1, 3000000);
    }
}

/*==============================================================================
 * 九点九八、MODE 9：全新接线 + SPI1 最小点亮（拆下来重接专用）
 *
 *   接线（8 脚模块，按丝印一一对应）：
 *     VCC  -> 3.3V
 *     GND  -> GND
 *     SCL  -> PB3    (SPI1_SCK,  AF5)
 *     SDA  -> PB5    (SPI1_MOSI, AF5)
 *     RES  -> PB10   (硬件复位，低有效)
 *     DC   -> PB1    (丝印可能写 A0 / RS)
 *     CS   -> PB0    (丝印可能写 CE)
 *     BLK  -> 3.3V   (丝印可能写 LED / BL；想调亮度以后接 PB8 做 PWM)
 *
 *   为什么换成这套脚：
 *     · 原来的 DC/CS 在 PC14/PC15，属于**备份域**，只能 3mA、跑不快，
 *       驱动能力弱，DC 翻转时上升沿可能被拖成三角波 -> 面板采样错
 *     · PB0/PB1/PB3/PB5/PB10 都是普通 IO，驱动能力强
 *     · SPI1 挂在 APB2（84MHz），/128 = 656kHz，余量充足
 *
 *   这个模式只做最小必要动作，不掺任何花活：
 *     硬件复位(RES) -> SWRESET -> SLPOUT -> COLMOD -> MADCTL -> DISPON
 *     -> CASET/RASET -> RAMWR -> 填色
 *   红 / 绿 / 蓝 / 白 / 黑 循环，每色 2 秒，一直循环，随时抬头就能看。
 *
 *   ★ 这个模式不依赖前面任何扫描逻辑，也不碰 PC14/PC15。
 *     如果它出纯色 -> 屏幕和驱动都没问题，问题全在旧接线上。
 *     如果它还是花   -> 那才需要考虑屏幕本身或模块丝印标错。
 *============================================================================*/
#define S1_CS_LOW()     (GPIOB->BSRR = ((uint32_t)GPIO_PIN_0  << 16))
#define S1_CS_HIGH()    (GPIOB->BSRR =  (uint32_t)GPIO_PIN_0)
#define S1_DC_LOW()     (GPIOB->BSRR = ((uint32_t)GPIO_PIN_1  << 16))
#define S1_DC_HIGH()    (GPIOB->BSRR =  (uint32_t)GPIO_PIN_1)
#define S1_RES_LOW()    (GPIOB->BSRR = ((uint32_t)GPIO_PIN_10 << 16))
#define S1_RES_HIGH()   (GPIOB->BSRR =  (uint32_t)GPIO_PIN_10)

static void SPI1_SendRaw(const uint8_t *buf, uint16_t len)
{
    uint16_t i;

    for (i = 0; i < len; i++)
    {
        while ((SPI1->SR & SPI_SR_TXE) == 0)
        {
        }
        *(__IO uint8_t *)&SPI1->DR = buf[i];

        while ((SPI1->SR & SPI_SR_RXNE) == 0)
        {
        }
        (void)*(__IO uint8_t *)&SPI1->DR;
    }

    while ((SPI1->SR & SPI_SR_BSY) != 0)
    {
    }
}

static void S1_Cmd(uint8_t c)
{
    S1_DC_LOW();
    SPI1_SendRaw(&c, 1);
}

static void S1_Data(const uint8_t *b, uint16_t n)
{
    S1_DC_HIGH();
    SPI1_SendRaw(b, n);
}

static void S1_Data1(uint8_t v)
{
    S1_Data(&v, 1);
}

static void Mode9_SPI1_Minimal(void)
{
    GPIO_InitTypeDef gi = {0};
    uint8_t  d[4];
    uint16_t i;
    uint16_t h;
    uint8_t  ci;
    uint16_t cols[5];

    cols[0] = C_RED;
    cols[1] = C_GREEN;
    cols[2] = C_BLUE;
    cols[3] = C_WHITE;
    cols[4] = C_BLACK;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_SPI1_CLK_ENABLE();

    /* PB3 / PB5 -> 复用 AF5 */
    GPIOB->MODER   &= ~((3u << 6) | (3u << 10));
    GPIOB->MODER   |=  ((2u << 6) | (2u << 10));
    GPIOB->OSPEEDR |=  ((3u << 6) | (3u << 10));
    GPIOB->PUPDR   &= ~((3u << 6) | (3u << 10));
    GPIOB->AFR[0]  &= ~((0xFu << 12) | (0xFu << 20));
    GPIOB->AFR[0]  |=  ((5u << 12) | (5u << 20));

    /* PB0 = CS, PB1 = DC, PB10 = RES */
    gi.Mode  = GPIO_MODE_OUTPUT_PP;
    gi.Pull  = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_HIGH;
    gi.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_10;
    HAL_GPIO_Init(GPIOB, &gi);

    S1_CS_HIGH();
    S1_DC_HIGH();
    S1_RES_HIGH();
    HAL_Delay(10);

    /* SPI1: MSTR | SSM | SSI | BR=110(/128 -> 656kHz) | SPE, Mode 0 */
    SPI1->CR1 = 0;
    SPI1->CR2 = 0;
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI
              | SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_SPE;

    U_Send("\r\n========================================\r\n");
    U_Send(" MODE 9 : SPI1 minimal  (new wiring)\r\n");
    U_Send(" SCL=PB3  SDA=PB5  RES=PB10  DC=PB1  CS=PB0\r\n");
    U_Send(" BLK=3.3V   SPI1 = 656kHz\r\n");
    U_Send(" colours loop forever, 2 sec each\r\n");
    U_Send("========================================\r\n");

    /* ---- 硬件复位 ---- */
    S1_RES_LOW();
    HAL_Delay(30);
    S1_RES_HIGH();
    HAL_Delay(150);

    S1_CS_LOW();

    S1_Cmd(0x01);  HAL_Delay(150);          /* SWRESET  */
    S1_Cmd(0x11);  HAL_Delay(150);          /* SLPOUT   */
    S1_Cmd(0x3A);  S1_Data1(0x05);          /* COLMOD  16bit */
    S1_Cmd(0x36);  S1_Data1(0x00);          /* MADCTL  默认方向 */
    S1_Cmd(0x29);  HAL_Delay(50);           /* DISPON   */

    U_Send("[init done] now filling colours\r\n");

    while (1)
    {
        for (ci = 0; ci < 5; ci++)
        {
            for (i = 0; i < GRAM_W; i++)
            {
                s_line[i * 2]     = (uint8_t)(cols[ci] >> 8);
                s_line[i * 2 + 1] = (uint8_t)(cols[ci] & 0xFF);
            }

            S1_Cmd(0x2A);
            d[0] = 0; d[1] = 0;
            d[2] = 0; d[3] = (uint8_t)(GRAM_W - 1);
            S1_Data(d, 4);

            S1_Cmd(0x2B);
            d[0] = 0; d[1] = 0;
            d[2] = 0; d[3] = (uint8_t)(GRAM_H - 1);
            S1_Data(d, 4);

            S1_Cmd(0x2C);
            /* 0x2C 之后全是像素数据，DC 必须是高 —— 这一步千万别漏 */
            S1_DC_HIGH();
            for (h = 0; h < GRAM_H; h++)
            {
                SPI1_SendRaw(s_line, (uint16_t)(GRAM_W * 2));
            }

            if (ci == 0)      { U_Send("  RED\r\n");   }
            else if (ci == 1) { U_Send("  GREEN\r\n"); }
            else if (ci == 2) { U_Send("  BLUE\r\n");  }
            else if (ci == 3) { U_Send("  WHITE\r\n"); }
            else              { U_Send("  BLACK\r\n"); }

            HAL_Delay(2000);
        }
    }
}

/*==============================================================================
 * 十一、main
 *============================================================================*/
int main(void)
{
    uint32_t csr;

    HAL_Init();

    LED_Init();
    Blink(3, 600000);                /* 3 次快闪 = 进了 main */

    USART1_InitRaw(16000000u, 115200u);

    U_Send("\r\n\r\n\r\n");
    U_Send("########################################\r\n");
    U_Send("# [1] BOOT OK  HSI 16MHz\r\n");
    U_Send("########################################\r\n");

    csr = RCC->CSR;
    U_Send("[1] RCC_CSR = 0x");
    U_Hex32(csr);
    U_Send("  ");
    if (csr & RCC_CSR_PORRSTF)  { U_Send("POR ");  }
    if (csr & RCC_CSR_PINRSTF)  { U_Send("PIN ");  }
    if (csr & RCC_CSR_BORRSTF)  { U_Send("BOR ");  }
    if (csr & RCC_CSR_SFTRSTF)  { U_Send("SFT ");  }
    if (csr & RCC_CSR_IWDGRSTF) { U_Send("IWDG "); }
    if (csr & RCC_CSR_WWDGRSTF) { U_Send("WWDG "); }
    if (csr & RCC_CSR_LPWRRSTF) { U_Send("LPWR "); }
    U_Send("\r\n");
    RCC->CSR |= RCC_CSR_RMVF;

    if (Clock_168MHz() != 0)
    {
        U_Send("[2] CLOCK FAIL, stay at HSI 16MHz\r\n");
        while (1)
        {
            Blink(2, 3000000);
        }
    }

    USART1_InitRaw(84000000u, 115200u);
    U_Send("[2] CLOCK 168MHz OK\r\n");
    Blink(5, 60000);                 /* 5 次快闪 = 时钟成功 */

    U_Send("[3] GPIO IDR: A=");
    U_Hex32(GPIOA->IDR & 0xFFFFu);
    U_Send(" B=");
    U_Hex32(GPIOB->IDR & 0xFFFFu);
    U_Send(" C=");
    U_Hex32(GPIOC->IDR & 0xFFFFu);
    U_Send(" E=");
    U_Hex32(GPIOE->IDR & 0xFFFFu);
    U_Send("\r\n");

    if (MODE == 1)
    {
        Mode1_Scan();
    }
    else if (MODE == 3)
    {
        Mode3_Focus();
    }
    else if (MODE == 4)
    {
        Mode4_KeyCombos();
    }
    else if (MODE == 5)
    {
        Mode5_CleanScan();
    }
    else if (MODE == 6)
    {
        Mode6_Diagnose();
    }
    else if (MODE == 7)
    {
        Mode7_InvScan();
    }
    else if (MODE == 8)
    {
        Mode8_BothOrders();
    }
    else if (MODE == 9)
    {
        Mode9_SPI1_Minimal();
    }
    else
    {
        Mode2_Confirm();
    }
}
