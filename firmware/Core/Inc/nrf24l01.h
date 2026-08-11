/**
 * @file nrf24l01.h
 * @brief NRF24L01 wireless transceiver — receiver driver (HAL).
 *
 * Ported from the 江协科技 (Jiangxie) balance-car receiver driver with the
 * same radio configuration, so it is drop-in compatible with the matching
 * hand-held controller firmware in firmware/remote_controller/.
 *
 * Bit-banged SPI (mode 0) on 5 GPIOs, polled interface (no IRQ).
 */

#ifndef NRF24L01_H
#define NRF24L01_H

#include <stdint.h>
#include <stdbool.h>

/* --- Radio parameters — MUST match the remote controller --- */
#define NRF24L01_PACKET_WIDTH 32  /* Static payload width, 1..32 */

/* --- Receiver API --- */

/** Initialise GPIO + radio, enter RX mode. Call once at startup. */
void nrf24l01_init(void);

/**
 * @brief Poll for a received packet (non-blocking).
 * @return true if a new packet was moved into the rx buffer.
 */
bool nrf24l01_receive(void);

/** Pointer to the most recently received packet (PACKET_WIDTH bytes). */
const uint8_t *nrf24l01_rx_packet(void);

#endif /* NRF24L01_H */
