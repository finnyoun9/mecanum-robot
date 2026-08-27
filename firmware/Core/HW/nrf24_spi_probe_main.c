/**
 * @file nrf24_spi_probe_main.c
 * @brief NRF24 SPI diagnostic with USART1 text output + GDB global vars.
 *
 * Pin mapping:
 *   CE=PA8  CSN=PA15  SCK=PB3  MISO=PB4  MOSI=PB5
 *   USART1_TX=PA9 (115200 baud @ 64 MHz)  LED=PC13 (active low)
 *
 * Global variables (GDB-readable):
 *   g_phase         — current test phase (1-4)
 *   g_miso_pullup   — MISO read with pull-up, NRF24 powered (0 or 1)
 *   g_status_fast   — STATUS reg via fast bit-bang SPI
 *   g_status_slow   — STATUS reg via slow (~100 kHz) SPI
 *
 * USART1 outputs human-readable text for each phase.
 * After all phases, enters infinite loop; GDB can attach to read globals.
 */
#include "stm32f1xx_hal.h"

#define NRF_CE_PORT   GPIOA
#define NRF_CE_PIN    GPIO_PIN_8
#define NRF_CSN_PORT  GPIOA
#define NRF_CSN_PIN   GPIO_PIN_15
#define NRF_SCK_PORT  GPIOB
#define NRF_SCK_PIN   GPIO_PIN_3
#define NRF_MISO_PORT GPIOB
#define NRF_MISO_PIN  GPIO_PIN_4
#define NRF_MOSI_PORT GPIOB
#define NRF_MOSI_PIN  GPIO_PIN_5

#define LED_PORT      GPIOC
#define LED_PIN       GPIO_PIN_13

/* ===== GDB-readable globals ===== */
volatile uint8_t g_phase         = 0;
volatile uint8_t g_miso_pullup   = 0xFF;
volatile uint8_t g_status_fast   = 0xFF;
volatile uint8_t g_status_slow   = 0xFF;

/* ===== Timing ===== */
static void delay_ms(uint32_t ms)
{
    uint32_t n = ms * 16000;
    while (n--) __asm volatile("nop");
}

static void delay_us(uint32_t us)
{
    uint32_t n = us * 16;
    while (n--) __asm volatile("nop");
}

/* ===== LED ===== */
static void led_on(void)  { HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET); }
static void led_off(void) { HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET); }

static void blink(uint8_t n, uint32_t on_ms)
{
    for (uint8_t i = 0; i < n; i++) {
        led_on();  delay_ms(on_ms);
        led_off(); delay_ms(on_ms);
    }
    delay_ms(300);
}

/* ===== USART1 (PA9 TX, 115200 @ 64 MHz APB2) — direct register ===== */
static void uart1_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    /* PA9 → AF push-pull 50 MHz */
    GPIOA->CRH &= ~(GPIO_CRH_CNF9 | GPIO_CRH_MODE9);
    GPIOA->CRH |= GPIO_CRH_MODE9_0 | GPIO_CRH_MODE9_1;  /* 50 MHz */
    GPIOA->CRH |= GPIO_CRH_CNF9_1;                        /* AF push-pull */

    USART1->BRR = 64000000 / 115200;   /* 555 */
    USART1->CR1 = USART_CR1_TE | USART_CR1_UE;
}

static void uart_putc(char c)
{
    while (!(USART1->SR & USART_SR_TXE)) {}
    USART1->DR = (uint8_t)c;
}

static void uart_puts(const char *s)
{
    while (*s) uart_putc(*s++);
}

static void uart_hex(uint8_t v)
{
    const char hex[] = "0123456789ABCDEF";
    uart_putc('0');
    uart_putc('x');
    uart_putc(hex[(v >> 4) & 0x0F]);
    uart_putc(hex[v & 0x0F]);
}

static void uart_dec(uint8_t v)
{
    if (v >= 100) { uart_putc('0' + v / 100); v %= 100; }
    if (v >= 10)  { uart_putc('0' + v / 10);  v %= 10; }
    uart_putc('0' + v);
}

/* ===== NRF24 GPIO ===== */
static void w_ce(uint8_t v)   { HAL_GPIO_WritePin(NRF_CE_PORT,   NRF_CE_PIN,   v ? GPIO_PIN_SET : GPIO_PIN_RESET); }
static void w_csn(uint8_t v)  { HAL_GPIO_WritePin(NRF_CSN_PORT,  NRF_CSN_PIN,  v ? GPIO_PIN_SET : GPIO_PIN_RESET); }
static void w_sck(uint8_t v)  { HAL_GPIO_WritePin(NRF_SCK_PORT,  NRF_SCK_PIN,  v ? GPIO_PIN_SET : GPIO_PIN_RESET); }
static void w_mosi(uint8_t v) { HAL_GPIO_WritePin(NRF_MOSI_PORT, NRF_MOSI_PIN, v ? GPIO_PIN_SET : GPIO_PIN_RESET); }
static uint8_t r_miso(void)   { return HAL_GPIO_ReadPin(NRF_MISO_PORT, NRF_MISO_PIN) == GPIO_PIN_SET; }

/* ===== Bit-bang SPI (fast) ===== */
static uint8_t spi_swap(uint8_t byte)
{
    for (int i = 0; i < 8; i++) {
        w_mosi((byte & 0x80) ? 1 : 0);
        byte <<= 1;
        w_sck(1);
        if (r_miso()) byte |= 0x01;
        w_sck(0);
    }
    return byte;
}

static uint8_t read_reg(uint8_t reg)
{
    w_csn(0);
    spi_swap(0x00 | reg);
    uint8_t data = spi_swap(0xFF);
    w_csn(1);
    return data;
}

/* ===== Bit-bang SPI (slow, ~100 kHz) ===== */
static uint8_t spi_swap_slow(uint8_t byte)
{
    for (int i = 0; i < 8; i++) {
        w_mosi((byte & 0x80) ? 1 : 0);
        delay_us(5);
        byte <<= 1;
        w_sck(1);
        delay_us(5);
        if (r_miso()) byte |= 0x01;
        delay_us(5);
        w_sck(0);
        delay_us(5);
    }
    return byte;
}

static uint8_t read_reg_slow(uint8_t reg)
{
    w_csn(0);
    delay_us(10);
    spi_swap_slow(0x00 | reg);
    uint8_t data = spi_swap_slow(0xFF);
    delay_us(10);
    w_csn(1);
    return data;
}

/* ===== GPIO init ===== */
static void gpio_init_all(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* Disable JTAG to free PA15, PB3, PB4 */
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_AFIO_REMAP_SWJ_NOJTAG();

    /* LED PC13 */
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Pin   = LED_PIN;
    HAL_GPIO_Init(LED_PORT, &gpio);
    led_off();

    /* CE, CSN — push-pull outputs */
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Pin   = NRF_CE_PIN | NRF_CSN_PIN;
    HAL_GPIO_Init(NRF_CE_PORT, &gpio);

    /* SCK, MOSI — push-pull outputs */
    gpio.Pin   = NRF_SCK_PIN | NRF_MOSI_PIN;
    HAL_GPIO_Init(NRF_SCK_PORT, &gpio);

    /* MISO — input with pull-up */
    gpio.Mode  = GPIO_MODE_INPUT;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Pin   = NRF_MISO_PIN;
    HAL_GPIO_Init(NRF_MISO_PORT, &gpio);

    /* Idle states */
    w_ce(0);
    w_csn(1);
    w_sck(0);
    w_mosi(0);
}

int main(void)
{
    /* --- System clock: HSI + PLL @ 64 MHz --- */
    __HAL_RCC_HSI_ENABLE();
    while (!__HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY)) {}
    RCC->CFGR = 0x00160000;  /* PLLMUL=16, PPRE1=2 */
    RCC->CR |= RCC_CR_PLLON;
    while (!__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY)) {}
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {}
    SystemCoreClock = 64000000;

    gpio_init_all();
    uart1_init();

    uart_puts("\r\n=== NRF24 SPI Probe ===\r\n");
    uart_puts("Clock: 64 MHz HSI+PLL\r\n");
    uart_puts("Pins: CE=PA8 CSN=PA15 SCK=PB3 MISO=PB4 MOSI=PB5\r\n");
    uart_puts("UART: PA9 TX 115200 8N1\r\n\r\n");

    /* ======= Phase 1: Output pin toggle ======= */
    g_phase = 1;
    uart_puts("[Phase 1] Output pin toggle test (2s)...\r\n");
    blink(1, 50);

    for (int i = 0; i < 200; i++) {
        w_ce(1);  w_csn(0);  w_sck(1);  w_mosi(1);
        delay_ms(5);
        w_ce(0);  w_csn(1);  w_sck(0);  w_mosi(0);
        delay_ms(5);
    }
    w_ce(0);  w_csn(1);  w_sck(0);  w_mosi(0);
    uart_puts("  Done. Scope each pin for square wave.\r\n\r\n");

    /* ======= Phase 2: MISO pull-up read ======= */
    g_phase = 2;
    uart_puts("[Phase 2] MISO pull-up read (NRF24 powered)...\r\n");
    blink(2, 50);

    delay_ms(100);
    g_miso_pullup = r_miso();
    uart_puts("  MISO = ");
    uart_dec(g_miso_pullup);
    if (g_miso_pullup) {
        uart_puts(" (HIGH = pull-up OK, line not stuck low)\r\n\r\n");
    } else {
        uart_puts(" (LOW = stuck low! Module pulling MISO down or PB4 locked)\r\n\r\n");
    }

    /* ======= Phase 3: Fast SPI STATUS read ======= */
    g_phase = 3;
    uart_puts("[Phase 3] Fast SPI read STATUS...\r\n");
    blink(3, 50);

    delay_ms(10);
    g_status_fast = read_reg(0x07);
    uart_puts("  STATUS (fast) = ");
    uart_hex(g_status_fast);
    if (g_status_fast == 0x0E) {
        uart_puts(" → OK! Module alive at full speed.\r\n");
    } else if (g_status_fast == 0x00) {
        uart_puts(" → FAIL (0x00). Trying slow SPI...\r\n");
    } else {
        uart_puts(" → Unexpected. See value above.\r\n");
    }
    uart_puts("\r\n");

    /* ======= Phase 4: Slow SPI STATUS read ======= */
    g_phase = 4;
    uart_puts("[Phase 4] Slow SPI read STATUS (~100 kHz)...\r\n");
    blink(4, 50);

    delay_ms(10);
    g_status_slow = read_reg_slow(0x07);
    uart_puts("  STATUS (slow) = ");
    uart_hex(g_status_slow);
    if (g_status_slow == 0x0E) {
        uart_puts(" → OK! Module alive at low speed (speed issue).\r\n");
    } else if (g_status_slow == 0x00) {
        uart_puts(" → FAIL (0x00). Module not responding.\r\n");
    } else {
        uart_puts(" → Unexpected. See value above.\r\n");
    }

    /* ======= Summary ======= */
    uart_puts("\r\n=== Summary ===\r\n");
    uart_puts("MISO pull-up: "); uart_dec(g_miso_pullup); uart_puts("\r\n");
    uart_puts("STATUS fast:  "); uart_hex(g_status_fast); uart_puts("\r\n");
    uart_puts("STATUS slow:  "); uart_hex(g_status_slow); uart_puts("\r\n");

    if (g_status_fast == 0x0E || g_status_slow == 0x0E) {
        uart_puts("VERDICT: Module alive!\r\n");
    } else {
        uart_puts("VERDICT: Module dead or wiring issue.\r\n");
    }
    uart_puts("================\r\n");

    /* Infinite loop — GDB can attach to read globals */
    while (1) {
        led_on();  delay_ms(1000);
        led_off(); delay_ms(1000);
    }
}
