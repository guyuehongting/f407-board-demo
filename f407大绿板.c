/*==============================================================================
 *  f407大绿板  ——  STM32F407VET6 综合演示
 *  屏幕（ST7735S 1.8" 128x160）+ 板载 5 键，一个文件搞定
 *
 *  目标：STM32F407VET6  /  HAL 库  /  Keil MDK  /  ARMCC5
 *
 *  ---------------------------------------------------------------------------
 *  这个程序干什么
 *  ---------------------------------------------------------------------------
 *  1. 上电初始化屏幕，画一个 5 键状态面板
 *  2. 5 行，每行一个键：左边是引脚名（PD10~PD14），右边是状态条
 *  3. 按下某个键 -> 那一行的边框和状态条变亮绿，松手变回暗灰
 *  4. 左下角是累计按键次数（3 位数字，只重绘变化的位，所以不闪）
 *  5. 长按 800ms -> 状态条闪一下黄，串口打 LONG
 *  6. 长按 KEY5 满 3 秒 -> 重新初始化屏幕并重绘（万一花屏了，不用按复位）
 *  7. 串口同步打印每个按键事件
 *
 *  ---------------------------------------------------------------------------
 *  接线（2026-10-25 / 10-26 实测通过，别再改）
 *  ---------------------------------------------------------------------------
 *  屏幕（ST7735S，8 脚模块）
 *      模块丝印          接到         说明
 *      VCC              3.3V
 *      GND              GND
 *      SCL / SCK        PB10         SPI2_SCK   (AF5)
 *      SDA / MOSI       PC3          SPI2_MOSI  (AF5)
 *      CS  / CE         PC15         片选（低有效，程序里一直拉低）
 *      DC  / A0 / RS    PC14         命令=低 / 数据=高
 *      RES              不接          靠软件复位 0x01 SWRESET
 *      BLK / LED        3.3V          背光（想调亮度就接 PB8 做 PWM）
 *
 *  板载 5 键（高电平有效，板上有外部下拉电阻）
 *      KEY1 = PD10      KEY2 = PD11      KEY3 = PD12
 *      KEY4 = PD13      KEY5 = PD14
 *
 *  串口：USART1，PA9=TX PA10=RX，115200
 *
 *  ---------------------------------------------------------------------------
 *  用法
 *  ---------------------------------------------------------------------------
 *      把本文件覆盖到工程的 Core/Src/main.c（或改名叫 main.c 放进去），
 *      编译烧录即可。不需要改 Keil 分组，不需要动 CubeMX。
 *
 *  ---------------------------------------------------------------------------
 *  三个必须记住的坑（都踩过，血泪）
 *  ---------------------------------------------------------------------------
 *  1. TFT_Cmd() 会把 DC 拉低且不恢复。任何「发像素」的地方
 *     （0x2C 之后）必须自己先 DC_HIGH()，否则所有像素字节都被当命令解释，
 *     屏幕永远只有花屏、出不了纯色。
 *  2. CS / DC 千万别接反。接反的表现很特别：命令能进去（屏幕会闪），
 *     数据进不去（填色一点不变）。正确是 CS=PC15、DC=PC14。
 *  3. RES 悬空会被噪声拉低，面板跑几秒自己硬复位 -> 常白屏，按 MCU 复位也没用。
 *     解法是 TFT_ParkPinsHigh()：把候选脚配成「输入 + 上拉」，
 *     用几十微安把 RES 钉在高阻高电平。
 *     ★ 不要用推挽输出高：PE2~PE5 是按键（按下即对地短路）、
 *       PB13 是电机方向（会被打开）。
 *
 *  ---------------------------------------------------------------------------
 *  第一次跑起来之后，按现象调这几个开关
 *  ---------------------------------------------------------------------------
 *      画面上下/左右颠倒      -> MADCTL_VAL 换一个（0x00 / 0x60 / 0xA0 / 0xC0）
 *      红蓝互换               -> USE_BGR 改 1
 *      边缘少几列/多几列杂色  -> X_OFF / Y_OFF 试 2 / 1
 *      想跑快一点             -> SPI_BR 改 2（1.3MHz）或 1（2.6MHz）
 *      按键手感太灵敏         -> KEY_DEBOUNCE_CNT 改 3
 *      按键反应迟钝           -> KEY_DEBOUNCE_CNT 改 1
 *      不想要串口输出         -> USE_SERIAL 改 0
 *      屏幕老花，想自动重绘   -> 长按 KEY5 三秒（不用改代码）
 *============================================================================*/

#include "stm32f4xx_hal.h"

/*==============================================================================
 *                              开关区
 *============================================================================*/

/* SPI2 分频：APB1 = 42MHz
 *   0 = /8    -> 5.25 MHz
 *   1 = /16   -> 2.6  MHz
 *   2 = /32   -> 1.3  MHz
 *   3 = /64   -> 656  kHz   <- 默认，最稳
 *   4 = /128  -> 328  kHz
 *   5 = /256  -> 164  kHz
 * 注意：CS/DC 在 PC14/PC15，属于备份域，驱动能力只有 3mA，跑不快。
 *      想提速先别动这三个脚。                                        */
#define SPI_BR          3

/* MADCTL（0x36）取值，决定方向和镜像
 *   0xC0 = 上下左右都翻（本项目实测通过的默认值，先用它）
 *   0x00 = 正常          0x60 = 上下翻          0xA0 = 左右翻            */
#define MADCTL_VAL      0xC0

/* 红蓝通道互换：屏幕红色显示成蓝色时改成 1 */
#define USE_BGR         0

/* 行列偏移：1.8" 模块的可见区通常是 132x162 显存里的 128x160
 * 画面整体偏左上/右下、边缘有杂色时，试 X_OFF=2 / Y_OFF=1               */
#define X_OFF           0
#define Y_OFF           0

/* 伽马校正：颜色发白、发暗、过渡不自然时改成 1 */
#define USE_GAMMA       0

/* 串口进度打印（USART1，PA9=TX PA10=RX，115200）。不需要就改 0 */
#define USE_SERIAL      1

/* --- 按键参数 --- */
#define KEY_NUM            5       /* 按键数量 */
#define KEY_ACTIVE_HIGH    1       /* 1 = 高电平有效（本板就是这个）0 = 低电平有效 */
#define KEY_DEBOUNCE_CNT   2       /* 连续 N 次采样一致才认：2 x 10ms = 20ms */
#define KEY_LONG_CNT       80      /* 按住 N 次采样算长按：80 x 10ms = 800ms */
#define KEY_RECOVER_CNT    300     /* KEY5 按住 N 次采样触发重绘：300 x 10ms = 3s */

/*==============================================================================
 *                              尺寸
 *============================================================================*/
#define GRAM_W          132         /* 控制器显存宽（ST7735 都是 132） */
#define GRAM_H          162         /* 控制器显存高 */
#define LCD_W           128         /* 可见区宽 */
#define LCD_H           160         /* 可见区高 */

/*==============================================================================
 *                              引脚定义
 *============================================================================*/

/* --- 屏幕 --- */
#define PIN_CS_PORT     GPIOC
#define PIN_CS_PIN      GPIO_PIN_15
#define PIN_DC_PORT     GPIOC
#define PIN_DC_PIN      GPIO_PIN_14

#define CS_LOW()        (PIN_CS_PORT->BSRR = ((uint32_t)PIN_CS_PIN << 16))
#define CS_HIGH()       (PIN_CS_PORT->BSRR =  (uint32_t)PIN_CS_PIN)
#define DC_LOW()        (PIN_DC_PORT->BSRR = ((uint32_t)PIN_DC_PIN << 16))
#define DC_HIGH()       (PIN_DC_PORT->BSRR =  (uint32_t)PIN_DC_PIN)

/* --- 按键 --- */
#define KEY_PORT        GPIOD

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
 *                              颜色
 *============================================================================*/
#define C_BLACK         0x0000
#define C_WHITE         0xFFFF
#define C_RED           0xF800
#define C_GREEN         0x07E0
#define C_BLUE          0x001F
#define C_YELLOW        0xFFE0
#define C_CYAN          0x07FF
#define C_MAGENTA       0xF81F
#define C_GRAY          0x8410
#define C_ORANGE        0xFD20

/* 界面配色（"大绿板"主题，用绿色做高亮） */
#define C_BG            0x0000      /* 背景：黑 */
#define C_HEAD          0x0320      /* 顶栏：深绿 */
#define C_BOX_OFF       0x7BEF      /* 边框（未按）：浅灰 */
#define C_BAR_OFF       0x18E3      /* 状态条（未按）：暗灰 */
#define C_ON            0x07E0      /* 高亮：亮绿 */
#define C_TXT           0xFFFF      /* 文字：白 */

/*==============================================================================
 *                              界面布局
 *============================================================================*/
#define ROW_Y0          24          /* 第一行的 y */
#define ROW_STEP        23          /* 行距 */
#define ROW_X           2           /* 行左边框 x */
#define ROW_W           124         /* 行宽 */
#define ROW_H           22          /* 行高 */
#define BAR_X           38          /* 状态条 x */
#define BAR_Y_OFF       4           /* 状态条相对行顶的偏移 */
#define BAR_W           84          /* 状态条宽 */
#define BAR_H           14          /* 状态条高 */
#define CNT_Y           140         /* 计数器 y */

/*==============================================================================
 *                         一、时钟（HSI 16M -> 168MHz）
 *   用 HAL_RCC_ClockConfig 而不是直接写寄存器，
 *   这样 HAL 会顺便重算 SysTick，HAL_Delay 才是准的。
 *============================================================================*/
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    osc.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState            = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState        = RCC_PLL_ON;
    osc.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    osc.PLL.PLLM            = 16;
    osc.PLL.PLLN            = 336;
    osc.PLL.PLLP            = RCC_PLLP_DIV2;
    osc.PLL.PLLQ            = 7;
    (void)HAL_RCC_OscConfig(&osc);

    clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                       | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV4;
    clk.APB2CLKDivider = RCC_HCLK_DIV2;
    (void)HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5);
}

/*==============================================================================
 *                         二、串口（纯寄存器，115200）
 *============================================================================*/
#if USE_SERIAL

static void USART1_InitRaw(uint32_t pclk2, uint32_t baud)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    GPIOA->MODER   &= ~((3u << 18) | (3u << 20));
    GPIOA->MODER   |=  ((2u << 18) | (2u << 20));
    GPIOA->OSPEEDR |=  ((3u << 18) | (3u << 20));
    GPIOA->PUPDR   &= ~((3u << 18) | (3u << 20));
    GPIOA->AFR[1]  &= ~((0xFu << 4) | (0xFu << 8));
    GPIOA->AFR[1]  |=  ((7u << 4) | (7u << 8));

    USART1->CR1 = 0;
    USART1->BRR = (pclk2 + (baud / 2u)) / baud;
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

static void U_Send(const char *s)
{
    while (*s != 0)
    {
        while ((USART1->SR & USART_SR_TXE) == 0)
        {
        }
        USART1->DR = (uint8_t)(*s);
        s++;
    }
}

static void U_Num(uint32_t v)
{
    char b[11];
    int8_t i = 10;

    b[10] = 0;
    if (v == 0)
    {
        U_Send("0");
        return;
    }
    while (v > 0 && i > 0)
    {
        i--;
        b[i] = (char)('0' + (v % 10u));
        v /= 10u;
    }
    U_Send(&b[i]);
}

#define U_LOG(x)        U_Send(x)
#define U_LOG_N(x)      U_Num((uint32_t)(x))

#else

#define U_LOG(x)
#define U_LOG_N(x)

#endif

/*==============================================================================
 *                         三、SPI2（纯寄存器，AF5）
 *============================================================================*/
#if   (SPI_BR == 0)
#define BR_BITS         SPI_CR1_BR_1
#elif (SPI_BR == 1)
#define BR_BITS         (SPI_CR1_BR_1 | SPI_CR1_BR_0)
#elif (SPI_BR == 2)
#define BR_BITS         SPI_CR1_BR_2
#elif (SPI_BR == 3)
#define BR_BITS         (SPI_CR1_BR_2 | SPI_CR1_BR_0)
#elif (SPI_BR == 4)
#define BR_BITS         (SPI_CR1_BR_2 | SPI_CR1_BR_1)
#else
#define BR_BITS         (SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0)
#endif

static void SPI2_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_SPI2_CLK_ENABLE();

    /* PB10 = SPI2_SCK  (AF5)，AFR[1] bits[11:8] */
    GPIOB->MODER   &= ~(3u << 20);
    GPIOB->MODER   |=  (2u << 20);
    GPIOB->OSPEEDR |=  (3u << 20);
    GPIOB->PUPDR   &= ~(3u << 20);
    GPIOB->AFR[1]  &= ~(0xFu << 8);
    GPIOB->AFR[1]  |=  (5u << 8);

    /* PC3 = SPI2_MOSI (AF5)，AFR[0] bits[15:12] */
    GPIOC->MODER   &= ~(3u << 6);
    GPIOC->MODER   |=  (2u << 6);
    GPIOC->OSPEEDR |=  (3u << 6);
    GPIOC->PUPDR   &= ~(3u << 6);
    GPIOC->AFR[0]  &= ~(0xFu << 12);
    GPIOC->AFR[0]  |=  (5u << 12);

    /* Mode 0：CPOL=0 CPHA=0，8 位，MSB 先出 */
    SPI2->CR1 = 0;
    SPI2->CR2 = 0;
    SPI2->I2SCFGR &= (uint16_t)~SPI_I2SCFGR_I2SMOD;
    SPI2->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI
              | BR_BITS | SPI_CR1_SPE;
}

static void SPI_Send(const uint8_t *buf, uint16_t len)
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
 *                         四、ST7735 底层
 *============================================================================*/

static uint8_t s_line[GRAM_W * 2];

/* 发命令：DC 拉低。★ 本函数结束后 DC 停在「低」，调用者要发数据必须自己拉高 */
static void TFT_Cmd(uint8_t cmd)
{
    DC_LOW();
    SPI_Send(&cmd, 1);
}

static void TFT_Data(const uint8_t *buf, uint16_t len)
{
    DC_HIGH();
    SPI_Send(buf, len);
}

static void TFT_Data1(uint8_t v)
{
    TFT_Data(&v, 1);
}

/* 设窗口（逻辑坐标，会自动加 X_OFF / Y_OFF） */
static void TFT_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t d[4];

    x0 = (uint16_t)(x0 + X_OFF);
    x1 = (uint16_t)(x1 + X_OFF);
    y0 = (uint16_t)(y0 + Y_OFF);
    y1 = (uint16_t)(y1 + Y_OFF);

    TFT_Cmd(0x2A);
    d[0] = (uint8_t)(x0 >> 8); d[1] = (uint8_t)(x0 & 0xFF);
    d[2] = (uint8_t)(x1 >> 8); d[3] = (uint8_t)(x1 & 0xFF);
    TFT_Data(d, 4);

    TFT_Cmd(0x2B);
    d[0] = (uint8_t)(y0 >> 8); d[1] = (uint8_t)(y0 & 0xFF);
    d[2] = (uint8_t)(y1 >> 8); d[3] = (uint8_t)(y1 & 0xFF);
    TFT_Data(d, 4);
}

/*------------------------------------------------------------------------------
 *  ★ RES 悬空的补救
 *  这块屏的 RES 没接，悬空脚会被噪声拉低，面板跑几秒就自己硬复位 -> 常白屏。
 *  做法：把「可能是 RES 的那一批脚」全部配成 输入 + 上拉，
 *  用几十微安的弱上拉把 RES 钉在高阻高电平。
 *
 *  为什么不用推挽输出高：
 *    PE2~PE5 是板载按键，按键一按就是直接对地短路，推挽输出高会烧；
 *    PB13 是电机方向脚，推挽输出高可能把电机驱动打开。
 *  输入上拉只有几十微安，既能给 RES 确定电平，又绝不会短路。
 *---------------------------------------------------------------------------*/
static void TFT_ParkPinsHigh(void)
{
    GPIO_InitTypeDef gi = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    gi.Mode  = GPIO_MODE_INPUT;
    gi.Pull  = GPIO_PULLUP;
    gi.Speed = GPIO_SPEED_FREQ_LOW;

    /* GPIOE: PE2 PE3 PE4 PE5 PE6 */
    gi.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
    HAL_GPIO_Init(GPIOE, &gi);

    /* GPIOC: PC0 PC1 PC2 PC13（PC3=MOSI / PC14=DC / PC15=CS 不能用） */
    gi.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_13;
    HAL_GPIO_Init(GPIOC, &gi);

    /* GPIOB: PB13 PB15（PB10=SCK 不能用） */
    gi.Pin = GPIO_PIN_13 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOB, &gi);
}

static void TFT_Init(void)
{
    GPIO_InitTypeDef gi = {0};
    uint8_t d3[3];
    uint8_t d2[2];

    __HAL_RCC_GPIOC_CLK_ENABLE();       /* PC14/PC15 的时钟必须先开，再配置 */

    /* CS / DC 都是推挽输出 */
    gi.Mode  = GPIO_MODE_OUTPUT_PP;
    gi.Pull  = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_HIGH;
    gi.Pin   = PIN_CS_PIN | PIN_DC_PIN;
    HAL_GPIO_Init(GPIOC, &gi);

    CS_HIGH();
    DC_HIGH();
    HAL_Delay(20);

    TFT_ParkPinsHigh();             /* ★ RES 必须被钉在高电平，否则面板会自己复位 */

    SPI2_Init();

    CS_LOW();                       /* 总线上只有一个设备，片选常低 */

    TFT_Cmd(0x01);  HAL_Delay(150);                 /* SWRESET  软件复位 */
    TFT_Cmd(0x11);  HAL_Delay(150);                 /* SLPOUT   退出睡眠 */

    TFT_Cmd(0xB1);  d3[0]=0x01; d3[1]=0x2C; d3[2]=0x2D; TFT_Data(d3, 3);   /* 帧率 */
    TFT_Cmd(0xC0);  d3[0]=0xA2; d3[1]=0x02; d3[2]=0x84; TFT_Data(d3, 3);   /* 电源 */
    TFT_Cmd(0xC1);  TFT_Data1(0xC5);
    TFT_Cmd(0xC2);  d2[0]=0x0A; d2[1]=0x00; TFT_Data(d2, 2);
    TFT_Cmd(0xC3);  d2[0]=0x8A; d2[1]=0x2A; TFT_Data(d2, 2);
    TFT_Cmd(0xC4);  d2[0]=0x8A; d2[1]=0xEE; TFT_Data(d2, 2);
    TFT_Cmd(0xC5);  TFT_Data1(0x0E);

    TFT_Cmd(0x20);                                  /* INVOFF  关反显 */

#if USE_BGR
    TFT_Cmd(0x36);  TFT_Data1((uint8_t)(MADCTL_VAL | 0x08));
#else
    TFT_Cmd(0x36);  TFT_Data1((uint8_t)MADCTL_VAL);
#endif

    TFT_Cmd(0x3A);  TFT_Data1(0x05);                /* COLMOD  16bit RGB565 */

#if USE_GAMMA
    {
        uint8_t gp[16];

        gp[0]=0x02; gp[1]=0x1C; gp[2]=0x07; gp[3]=0x12;
        gp[4]=0x37; gp[5]=0x32; gp[6]=0x29; gp[7]=0x2D;
        gp[8]=0x29; gp[9]=0x25; gp[10]=0x2B; gp[11]=0x39;
        gp[12]=0x00; gp[13]=0x01; gp[14]=0x03; gp[15]=0x10;
        TFT_Cmd(0xE0);  TFT_Data(gp, 16);           /* 正极性伽马 */

        gp[0]=0x03; gp[1]=0x1D; gp[2]=0x07; gp[3]=0x06;
        gp[4]=0x2E; gp[5]=0x2C; gp[6]=0x29; gp[7]=0x2D;
        gp[8]=0x2E; gp[9]=0x2E; gp[10]=0x37; gp[11]=0x3F;
        gp[12]=0x00; gp[13]=0x00; gp[14]=0x02; gp[15]=0x10;
        TFT_Cmd(0xE1);  TFT_Data(gp, 16);           /* 负极性伽马 */
    }
#endif

    TFT_Cmd(0x13);  HAL_Delay(10);                  /* NORON   普通显示模式 */
    TFT_Cmd(0x29);  HAL_Delay(50);                  /* DISPON  开显示 */

    U_LOG("[TFT] init done\r\n");
}

/*==============================================================================
 *                         五、绘图 API
 *============================================================================*/

/* 填满整个控制器显存（0..131 / 0..161），不带偏移，用来彻底消掉边缘杂色 */
static void TFT_ClearAll(uint16_t color)
{
    uint16_t i, h;
    uint8_t d[4];

    for (i = 0; i < GRAM_W; i++)
    {
        s_line[i * 2]     = (uint8_t)(color >> 8);
        s_line[i * 2 + 1] = (uint8_t)(color & 0xFF);
    }

    TFT_Cmd(0x2A);
    d[0]=0; d[1]=0; d[2]=(uint8_t)((GRAM_W - 1) >> 8); d[3]=(uint8_t)((GRAM_W - 1) & 0xFF);
    TFT_Data(d, 4);

    TFT_Cmd(0x2B);
    d[0]=0; d[1]=0; d[2]=(uint8_t)((GRAM_H - 1) >> 8); d[3]=(uint8_t)((GRAM_H - 1) & 0xFF);
    TFT_Data(d, 4);

    TFT_Cmd(0x2C);
    DC_HIGH();                       /* ★ 0x2C 之后全是像素，DC 必须是高 */

    for (h = 0; h < GRAM_H; h++)
    {
        SPI_Send(s_line, (uint16_t)(GRAM_W * 2));
    }
}

/* 填充矩形（逻辑坐标 0..127 / 0..159） */
static void TFT_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    uint16_t i, j;

    if (w == 0 || h == 0) return;
    if (x >= LCD_W || y >= LCD_H) return;
    if ((uint32_t)x + w > LCD_W) w = (uint16_t)(LCD_W - x);
    if ((uint32_t)y + h > LCD_H) h = (uint16_t)(LCD_H - y);

    for (i = 0; i < w; i++)
    {
        s_line[i * 2]     = (uint8_t)(color >> 8);
        s_line[i * 2 + 1] = (uint8_t)(color & 0xFF);
    }

    TFT_SetWindow(x, y, (uint16_t)(x + w - 1), (uint16_t)(y + h - 1));
    DC_HIGH();                       /* ★ 同上，漏了这一行就永远花屏 */

    for (j = 0; j < h; j++)
    {
        SPI_Send(s_line, (uint16_t)(w * 2));
    }
}

/* 画矩形边框（w/h 含边框本身） */
static void TFT_DrawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    TFT_FillRect(x, y, w, 1, color);
    TFT_FillRect(x, (uint16_t)(y + h - 1), w, 1, color);
    TFT_FillRect(x, y, 1, h, color);
    TFT_FillRect((uint16_t)(x + w - 1), y, 1, h, color);
}

/*==============================================================================
 *                         六、5x7 字模
 *   每字节一列，bit0 = 最上面那一行。bit0..bit6 对应 7 行。
 *   只做「界面用得到的字符」，省 Flash。
 *============================================================================*/
static const uint8_t FONT_DIGIT[10][5] =
{
    { 0x3E, 0x51, 0x49, 0x45, 0x3E },   /* 0 */
    { 0x00, 0x42, 0x7F, 0x40, 0x00 },   /* 1 */
    { 0x42, 0x61, 0x51, 0x49, 0x46 },   /* 2 */
    { 0x21, 0x41, 0x45, 0x4B, 0x31 },   /* 3 */
    { 0x18, 0x14, 0x12, 0x7F, 0x10 },   /* 4 */
    { 0x27, 0x45, 0x45, 0x45, 0x39 },   /* 5 */
    { 0x3C, 0x4A, 0x49, 0x49, 0x30 },   /* 6 */
    { 0x01, 0x71, 0x09, 0x05, 0x03 },   /* 7 */
    { 0x36, 0x49, 0x49, 0x49, 0x36 },   /* 8 */
    { 0x06, 0x49, 0x49, 0x29, 0x1E }    /* 9 */
};

static const uint8_t FONT_F[5] = { 0x7F, 0x09, 0x09, 0x09, 0x01 };
static const uint8_t FONT_P[5] = { 0x7F, 0x09, 0x09, 0x09, 0x06 };
static const uint8_t FONT_D[5] = { 0x7F, 0x41, 0x41, 0x41, 0x3E };

/* 画一个字形：cols 是 5 列 */
static void TFT_DrawGlyph(uint16_t x, uint16_t y, const uint8_t *cols,
                          uint16_t fg, uint16_t bg, uint8_t scale)
{
    uint8_t c, r, bits;

    for (c = 0; c < 5u; c++)
    {
        bits = cols[c];
        for (r = 0; r < 7u; r++)
        {
            TFT_FillRect((uint16_t)(x + (uint16_t)c * scale),
                         (uint16_t)(y + (uint16_t)r * scale),
                         scale, scale,
                         ((bits & (1u << r)) != 0u) ? fg : bg);
        }
    }
}

/* 画一个字符，返回下一个字符的 x。不认识的字符画成空格 */
static uint16_t TFT_DrawChar(uint16_t x, uint16_t y, char ch,
                             uint16_t fg, uint16_t bg, uint8_t scale)
{
    const uint8_t *g = 0;

    if (ch >= '0' && ch <= '9')     { g = FONT_DIGIT[ch - '0']; }
    else if (ch == 'F')             { g = FONT_F; }
    else if (ch == 'P')             { g = FONT_P; }
    else if (ch == 'D')             { g = FONT_D; }

    if (g != 0)
    {
        TFT_DrawGlyph(x, y, g, fg, bg, scale);
    }
    else
    {
        TFT_FillRect(x, y, (uint16_t)(5u * scale), (uint16_t)(7u * scale), bg);
    }

    return (uint16_t)(x + 6u * scale);
}

/* 画一串字符 */
static void TFT_DrawText(uint16_t x, uint16_t y, const char *s,
                         uint16_t fg, uint16_t bg, uint8_t scale)
{
    while (*s != 0)
    {
        x = TFT_DrawChar(x, y, *s, fg, bg, scale);
        s++;
    }
}

/* 画一个数字（0~9），scale 是放大倍数 */
static void TFT_DrawDigit(uint16_t x, uint16_t y, uint8_t d,
                          uint16_t fg, uint16_t bg, uint8_t scale)
{
    if (d > 9u) return;
    TFT_DrawGlyph(x, y, FONT_DIGIT[d], fg, bg, scale);
}

/*==============================================================================
 *                         七、按键驱动
 *
 *  实测：PD10~PD14，静止读 0（板上有外部下拉），按下读 1（接 3.3V）。
 *        所以是「高电平有效」，内部再开下拉，抗干扰。
 *
 *  API：
 *      Key_Init()          上电调一次
 *      Key_Poll()          每 10ms 调一次
 *      Key_Pressed()       本次新按下的位掩码，读后清零
 *      Key_Released()      本次新松开的位掩码，读后清零
 *      Key_Held()          当前按住哪些键（不清零）
 *      Key_Long()          本次触发长按的位掩码，读后清零
 *      位掩码 bit0 = KEY1(PD10) ... bit4 = KEY5(PD14)
 *============================================================================*/

static uint8_t  s_stable;               /* 消抖后的稳定状态 */
static uint8_t  s_cnt[KEY_NUM];         /* 消抖计数 */
static uint16_t s_hold[KEY_NUM];        /* 按住时长（采样次数） */
static uint8_t  s_ev_pressed;
static uint8_t  s_ev_released;
static uint8_t  s_ev_long;

/* 读一个键的逻辑电平：1 = 按下 */
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

static void Key_Init(void)
{
    GPIO_InitTypeDef gi = {0};
    uint8_t i;

    __HAL_RCC_GPIOD_CLK_ENABLE();

    gi.Mode  = GPIO_MODE_INPUT;
    gi.Speed = GPIO_SPEED_FREQ_LOW;

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

static void Key_Poll(void)
{
    uint8_t i;
    uint8_t bit;
    uint8_t raw;

    for (i = 0; i < KEY_NUM; i++)
    {
        bit = (uint8_t)(1u << i);
        raw = Key_RawBit(i);

        if (raw == (((s_stable & bit) != 0u) ? 1u : 0u))
        {
            s_cnt[i] = 0u;

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
            s_cnt[i]++;
            if (s_cnt[i] >= KEY_DEBOUNCE_CNT)
            {
                s_cnt[i]  = 0u;
                s_hold[i] = 0u;
                s_stable ^= bit;

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

static uint8_t Key_Pressed(void)
{
    uint8_t v = s_ev_pressed;
    s_ev_pressed = 0u;
    return v;
}

static uint8_t Key_Released(void)
{
    uint8_t v = s_ev_released;
    s_ev_released = 0u;
    return v;
}

static uint8_t Key_Long(void)
{
    uint8_t v = s_ev_long;
    s_ev_long = 0u;
    return v;
}

static uint8_t Key_Held(void)
{
    return s_stable;
}

/*==============================================================================
 *                         八、界面
 *
 *  重要：SPI 只有 656kHz，整屏刷一次要 0.5 秒。
 *        所以绝对不能每帧重绘整屏 —— 只重绘「变化的那一小块」。
 *        下面每个函数都只碰自己那点区域。
 *============================================================================*/

static uint8_t  s_shown;        /* 屏幕上当前显示的按住状态 */
static uint16_t s_count;        /* 累计按下次数 */
static uint16_t s_count_shown;  /* 屏幕上当前显示的次数 */

/* 某一行 y 坐标 */
static uint16_t Row_Y(uint8_t i)
{
    return (uint16_t)(ROW_Y0 + (uint16_t)i * ROW_STEP);
}

/* 画一行的引脚名 "PD10".."PD14" */
static void Panel_DrawRowLabel(uint8_t i, uint16_t fg, uint16_t bg)
{
    char buf[5];

    buf[0] = 'P';
    buf[1] = 'D';
    buf[2] = '1';
    buf[3] = (char)('0' + i);
    buf[4] = 0;

    TFT_DrawText((uint16_t)(ROW_X + 6), (uint16_t)(Row_Y(i) + 7), buf, fg, bg, 1);
}

/* 重绘一行（边框 + 状态条），on=1 高亮 */
static void Panel_UpdateRow(uint8_t i, uint8_t on)
{
    uint16_t y0 = Row_Y(i);

    if (on != 0u)
    {
        TFT_DrawRect(ROW_X, y0, ROW_W, ROW_H, C_ON);
        TFT_FillRect(BAR_X, (uint16_t)(y0 + BAR_Y_OFF), BAR_W, BAR_H, C_ON);
    }
    else
    {
        TFT_DrawRect(ROW_X, y0, ROW_W, ROW_H, C_BOX_OFF);
        TFT_FillRect(BAR_X, (uint16_t)(y0 + BAR_Y_OFF), BAR_W, BAR_H, C_BAR_OFF);
    }
}

/* 只重绘「和屏幕不一致」的行 */
static void Panel_Sync(void)
{
    uint8_t held = Key_Held();
    uint8_t diff = (uint8_t)(held ^ s_shown);
    uint8_t i;

    for (i = 0; i < KEY_NUM; i++)
    {
        if ((diff & (uint8_t)(1u << i)) != 0u)
        {
            Panel_UpdateRow(i, (uint8_t)((held >> i) & 1u));
        }
    }
    s_shown = held;
}

/* 显示次数，只重绘变化的那一位 */
static void Panel_ShowCount(uint16_t v)
{
    uint16_t pv = s_count_shown;
    uint8_t d;

    if (v == pv)
    {
        return;
    }

    d = (uint8_t)((v / 100u) % 10u);
    if (d != (uint8_t)((pv / 100u) % 10u))
    {
        TFT_DrawDigit(4, CNT_Y, d, C_YELLOW, C_BG, 2);
    }
    d = (uint8_t)((v / 10u) % 10u);
    if (d != (uint8_t)((pv / 10u) % 10u))
    {
        TFT_DrawDigit(16, CNT_Y, d, C_YELLOW, C_BG, 2);
    }
    d = (uint8_t)(v % 10u);
    if (d != (uint8_t)(pv % 10u))
    {
        TFT_DrawDigit(28, CNT_Y, d, C_YELLOW, C_BG, 2);
    }

    s_count_shown = v;
}

/* 画静态部分：顶栏 + 5 行边框 + 引脚名 + 计数器底 */
static void Panel_DrawStatic(void)
{
    uint8_t i;
    uint16_t y0;

    /* 顶栏 */
    TFT_FillRect(0, 0, LCD_W, 22, C_HEAD);
    TFT_DrawText(6, 5, "F407", C_TXT, C_HEAD, 2);

    /* 5 行 */
    for (i = 0; i < KEY_NUM; i++)
    {
        y0 = Row_Y(i);
        TFT_DrawRect(ROW_X, y0, ROW_W, ROW_H, C_BOX_OFF);
        Panel_DrawRowLabel(i, C_TXT, C_BG);
        TFT_FillRect(BAR_X, (uint16_t)(y0 + BAR_Y_OFF), BAR_W, BAR_H, C_BAR_OFF);
    }

    s_shown       = 0u;
    s_count_shown = 0xFFFFu;        /* 强制第一次把三位都画出来 */
    Panel_ShowCount(s_count);
}

/* 整屏重绘（先彻底清显存，消掉边缘杂色） */
static void Panel_RedrawAll(void)
{
    TFT_ClearAll(C_BG);
    Panel_DrawStatic();

    /* 如果重绘时正好有键按着，把它们显示出来 */
    s_shown = 0u;
    Panel_Sync();
}

/* 长按反馈：状态条闪一下黄，再恢复 */
static void Panel_FlashRow(uint8_t i)
{
    uint16_t y0 = Row_Y(i);

    TFT_FillRect(BAR_X, (uint16_t)(y0 + BAR_Y_OFF), BAR_W, BAR_H, C_YELLOW);
    HAL_Delay(180);
    TFT_FillRect(BAR_X, (uint16_t)(y0 + BAR_Y_OFF), BAR_W, BAR_H,
                 (((Key_Held() >> i) & 1u) != 0u) ? C_ON : C_BAR_OFF);
}

/*==============================================================================
 *                         九、串口日志
 *============================================================================*/
static void Log_KeyEvent(uint8_t mask, const char *what)
{
    uint8_t i;

    for (i = 0; i < KEY_NUM; i++)
    {
        if ((mask & (uint8_t)(1u << i)) != 0u)
        {
            U_LOG("[KEY] ");
            U_LOG(s_key_name[i]);
            U_LOG(" ");
            U_LOG(what);
            U_LOG("\r\n");
        }
    }
}

/*==============================================================================
 * 十、CubeMX 生成代码需要的钩子
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
 *                         十一、main
 *============================================================================*/
int main(void)
{
    uint8_t  p;
    uint8_t  r;
    uint8_t  l;
    uint8_t  i;
    uint8_t  held;
    uint16_t recover;

    HAL_Init();
    SystemClock_Config();

#if USE_SERIAL
    USART1_InitRaw(84000000u, 115200u);
    U_LOG("\r\n\r\n");
    U_LOG("########################################\r\n");
    U_LOG("#  f407 big green board  ST7735 + 5 keys\r\n");
    U_LOG("#  TFT : SCK=PB10 SDA=PC3 CS=PC15 DC=PC14 RES=nc\r\n");
    U_LOG("#  KEYS: PD10 PD11 PD12 PD13 PD14  (active HIGH)\r\n");
    U_LOG("#  SPI_BR=");
    U_LOG_N(SPI_BR);
    U_LOG("  MADCTL=");
    U_LOG_N(MADCTL_VAL);
    U_LOG("\r\n");
    U_LOG("########################################\r\n");
#endif

    Key_Init();

    TFT_Init();
    Panel_RedrawAll();

    U_LOG("[OK] panel ready, press any key\r\n");
    U_LOG("     hold KEY5 for 3s = redraw screen\r\n");

    recover = 0u;

    while (1)
    {
        Key_Poll();

        p = Key_Pressed();
        r = Key_Released();
        l = Key_Long();
        held = Key_Held();

        /* 1) 按住状态变化 -> 只重绘变化的那几行 */
        if ((p | r) != 0u)
        {
            Panel_Sync();
        }

        /* 2) 新按下 -> 次数 +1，只重绘变化的数字位 */
        if (p != 0u)
        {
            if (s_count < 999u)
            {
                s_count++;
            }
            Panel_ShowCount(s_count);
            Log_KeyEvent(p, "PRESS");
        }

        /* 3) 长按 -> 状态条闪黄 */
        if (l != 0u)
        {
            for (i = 0; i < KEY_NUM; i++)
            {
                if ((l & (uint8_t)(1u << i)) != 0u)
                {
                    Panel_FlashRow(i);
                }
            }
            Log_KeyEvent(l, "LONG");
        }

        if (r != 0u)
        {
            Log_KeyEvent(r, "RELEASE");
        }

        /* 4) KEY5 按住 3 秒 -> 重新初始化屏幕（万一花屏，不用按复位键） */
        if ((held & (uint8_t)(1u << (KEY_NUM - 1))) != 0u)
        {
            if (recover < 0xFFFFu)
            {
                recover++;
            }
            if (recover == KEY_RECOVER_CNT)
            {
                U_LOG("[TFT] KEY5 held 3s -> re-init and redraw\r\n");
                TFT_Init();
                Panel_RedrawAll();
            }
        }
        else
        {
            recover = 0u;
        }

        HAL_Delay(10);
    }
}
