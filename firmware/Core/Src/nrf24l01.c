/**
 * @file nrf24l01.c
 * @brief NRF24L01 receiver driver — bit-banged SPI, polled RX.
 *
 * Logic and register configuration ported verbatim from the 江协科技
 * balance-car receiver driver (firmware/remote_controller/), which is
 * known to work with the matching hand-held controller. Only the low-level
 * GPIO calls were rewritten from the STM32 SPL to the HAL API.
 *
 * PIN MAPPING (default — matches the 江协 balance-car receiver):
 *   CE   = PA8
 *   CSN  = PA15
 *   SCK  = PB3
 *   MISO = PB4
 *   MOSI = PB5
 *
 * PA15/PB3/PB4 are JTAG pins; the driver disables JTAG so they work as GPIO.
 * Adjust the macros below + the GPIO init in nrf24l01_gpio_init() for your
 * board.
 */

#include "nrf24l01.h"

#include "main.h"  /* CubeMX HAL — GPIO definitions */

/* ==================================================================== */
/*  Pin configuration                                                    */
/* ==================================================================== */

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

static void w_ce(uint8_t v)    { HAL_GPIO_WritePin(NRF_CE_PORT,   NRF_CE_PIN,   v ? GPIO_PIN_SET : GPIO_PIN_RESET); }
static void w_csn(uint8_t v)   { HAL_GPIO_WritePin(NRF_CSN_PORT,  NRF_CSN_PIN,  v ? GPIO_PIN_SET : GPIO_PIN_RESET); }
static void w_sck(uint8_t v)   { HAL_GPIO_WritePin(NRF_SCK_PORT,  NRF_SCK_PIN,  v ? GPIO_PIN_SET : GPIO_PIN_RESET); }
static void w_mosi(uint8_t v)  { HAL_GPIO_WritePin(NRF_MOSI_PORT, NRF_MOSI_PIN, v ? GPIO_PIN_SET : GPIO_PIN_RESET); }
static uint8_t r_miso(void)    { return HAL_GPIO_ReadPin(NRF_MISO_PORT, NRF_MISO_PIN) == GPIO_PIN_SET; }

/* ==================================================================== */
/*  Command / register definitions (NRF24L01 datasheet)                  */
/* ==================================================================== */

#define NRF_R_REGISTER    0x00
#define NRF_W_REGISTER    0x20
#define NRF_R_RX_PAYLOAD  0x61
#define NRF_W_TX_PAYLOAD  0xA0
#define NRF_FLUSH_TX      0xE1
#define NRF_FLUSH_RX      0xE2
#define NRF_NOP           0xFF

#define REG_CONFIG        0x00
#define REG_EN_AA         0x01
#define REG_EN_RXADDR     0x02
#define REG_SETUP_AW      0x03
#define REG_SETUP_RETR    0x04
#define REG_RF_CH         0x05
#define REG_RF_SETUP      0x06
#define REG_STATUS        0x07
#define REG_RX_ADDR_P0    0x0A
#define REG_TX_ADDR       0x10
#define REG_RX_PW_P0      0x11

/* Status register bits */
#define ST_MAX_RT  0x10
#define ST_TX_DS   0x20
#define ST_RX_DR   0x40

/* Radio config — MUST match the remote controller */
static const uint8_t rx_addr[5] = {0x11, 0x22, 0x33, 0x44, 0x55};

static uint8_t rx_packet[NRF24L01_PACKET_WIDTH];

/* ==================================================================== */
/*  Bit-banged SPI                                                       */
/* ==================================================================== */

static uint8_t spi_swap(uint8_t byte)
{
    /* SPI mode 0: data shifted out on MOSI on falling-to-rising SCK,
     * data sampled on MISO on the rising edge. MSB first. */
    for (int i = 0; i < 8; i++) {
        w_mosi((byte & 0x80) ? 1 : 0);
        byte <<= 1;
        w_sck(1);
        if (r_miso()) byte |= 0x01;
        w_sck(0);
    }
    return byte;
}

/* ==================================================================== */
/*  Register access                                                      */
/* ==================================================================== */

static uint8_t read_reg(uint8_t reg)
{
    w_csn(0);
    spi_swap(NRF_R_REGISTER | reg);
    uint8_t data = spi_swap(NRF_NOP);
    w_csn(1);
    return data;
}

static void write_reg(uint8_t reg, uint8_t data)
{
    w_csn(0);
    spi_swap(NRF_W_REGISTER | reg);
    spi_swap(data);
    w_csn(1);
}

static void write_regs(uint8_t reg, const uint8_t *data, uint8_t count)
{
    w_csn(0);
    spi_swap(NRF_W_REGISTER | reg);
    for (uint8_t i = 0; i < count; i++) spi_swap(data[i]);
    w_csn(1);
}

static void read_rx_payload(uint8_t *data, uint8_t count)
{
    w_csn(0);
    spi_swap(NRF_R_RX_PAYLOAD);
    for (uint8_t i = 0; i < count; i++) data[i] = spi_swap(NRF_NOP);
    w_csn(1);
}

static void flush_tx(void) { w_csn(0); spi_swap(NRF_FLUSH_TX); w_csn(1); }
static void flush_rx(void) { w_csn(0); spi_swap(NRF_FLUSH_RX); w_csn(1); }

static uint8_t read_status(void) { return read_reg(REG_STATUS); }

/* ==================================================================== */
/*  Radio modes                                                          */
/* ==================================================================== */

static void rx_mode(void)
{
    /* PRIM_RX = 1 + PWR_UP = 1, then raise CE to start receiving */
    w_ce(0);
    uint8_t config = read_reg(REG_CONFIG);
    if (config == 0xFF) return;  /* Device not present / bus fault */
    write_reg(REG_CONFIG, config | 0x03);
    w_ce(1);
}

/* ==================================================================== */
/*  Public API                                                           */
/* ==================================================================== */

static void gpio_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    /* PA15 / PB3 / PB4 are JTAG pins — remap them to GPIO (release JTAG). */
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_AFIO_REMAP_SWJ_NOJTAG();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Pin   = NRF_CE_PIN | NRF_CSN_PIN;
    HAL_GPIO_Init(NRF_CE_PORT, &gpio);
    gpio.Pin   = NRF_SCK_PIN | NRF_MOSI_PIN;
    HAL_GPIO_Init(NRF_SCK_PORT, &gpio);

    gpio.Mode  = GPIO_MODE_INPUT;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Pin   = NRF_MISO_PIN;
    HAL_GPIO_Init(NRF_MISO_PORT, &gpio);

    w_ce(0);   /* Exit RX/TX mode */
    w_csn(1);  /* Deselect */
    w_sck(0);  /* SPI mode 0 idle state */
    w_mosi(0);
}

void nrf24l01_init(void)
{
    gpio_init();

    /* Same register set as the remote controller's NRF24L01_Init().
     * Communication fails unless both ends use identical parameters. */
    write_reg(REG_CONFIG, 0x08);    /* CRC 1 byte, PWR_UP=0, PRIM_RX=0 */
    write_reg(REG_EN_AA, 0x3F);     /* Auto-ACK on all pipes */
    write_reg(REG_EN_RXADDR, 0x01); /* Only pipe 0 */
    write_reg(REG_SETUP_AW, 0x03);  /* 5-byte address */
    write_reg(REG_SETUP_RETR, 0x03);/* 250us retry, 3 retransmits */
    write_reg(REG_RF_CH, 0x02);     /* 2400+2 = 2402 MHz */
    write_reg(REG_RF_SETUP, 0x0E);  /* 2 Mbps, 0 dBm */
    write_reg(REG_RX_PW_P0, NRF24L01_PACKET_WIDTH);
    write_regs(REG_RX_ADDR_P0, rx_addr, 5);

    flush_tx();
    flush_rx();
    write_reg(REG_STATUS, 0x70);    /* Clear MAX_RT / TX_DS / RX_DR */

    rx_mode();
}

bool nrf24l01_receive(void)
{
    uint8_t status = read_status();
    uint8_t config = read_reg(REG_CONFIG);

    if ((config & 0x02) == 0x00) {
        /* PWR_UP cleared — device dropped out of RX mode; re-init. */
        nrf24l01_init();
        return false;
    }
    if ((status & (ST_MAX_RT | ST_TX_DS)) == (ST_MAX_RT | ST_TX_DS)) {
        /* Bogus state — re-init to recover. */
        nrf24l01_init();
        return false;
    }
    if ((status & ST_RX_DR) == 0) {
        return false;  /* No new packet */
    }

    read_rx_payload(rx_packet, NRF24L01_PACKET_WIDTH);
    write_reg(REG_STATUS, ST_RX_DR);  /* Clear RX_DR */
    flush_rx();
    return true;
}

const uint8_t *nrf24l01_rx_packet(void)
{
    return rx_packet;
}
