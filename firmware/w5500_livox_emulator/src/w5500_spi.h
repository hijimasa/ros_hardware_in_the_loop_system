#ifndef W5500_SPI_H
#define W5500_SPI_H

#include <stdint.h>
#include <stdbool.h>

/*
 * W5500 SPI interface for W5500-EVB-Pico2
 *
 * W5500-EVB-Pico pin assignments:
 *   SPI0: SCK=GP18, MOSI=GP19, MISO=GP16, CS=GP17
 *   W5500 RST=GP20, INT=GP21
 */

/* W5500-EVB-Pico pin definitions */
#define W5500_SPI_PORT    spi0
#define W5500_PIN_SCK     18
#define W5500_PIN_MOSI    19
#define W5500_PIN_MISO    16
#define W5500_PIN_CS      17
#define W5500_PIN_RST     20
#define W5500_PIN_INT     21

/* SPI clock speed */
#define W5500_SPI_CLK_HZ  33000000  /* 33 MHz */

/* Initialize W5500 SPI and hardware reset */
void w5500_spi_init(void);

/* Configure W5500 network: IP, subnet, gateway, MAC */
void w5500_network_init(const uint8_t ip[4], const uint8_t subnet[4],
                        const uint8_t gateway[4], const uint8_t mac[6]);

/* Open a UDP socket. Returns socket number (0-7) or -1 on error. */
int w5500_udp_open(uint16_t local_port);

/* Send UDP datagram. Returns bytes sent or -1 on error. */
int w5500_udp_sendto(int sock, const uint8_t *data, uint16_t len,
                     const uint8_t dest_ip[4], uint16_t dest_port);

/* Receive UDP datagram. Returns bytes received or 0 if none, -1 on error. */
int w5500_udp_recvfrom(int sock, uint8_t *buf, uint16_t buf_size,
                       uint8_t src_ip[4], uint16_t *src_port);

/* Close UDP socket */
void w5500_udp_close(int sock);

/* W5500 SPI callback functions (called by ioLibrary) */
void w5500_cs_select(void);
void w5500_cs_deselect(void);
uint8_t w5500_spi_readbyte(void);
void w5500_spi_writebyte(uint8_t byte);
void w5500_spi_readburst(uint8_t *buf, uint16_t len);
void w5500_spi_writeburst(const uint8_t *buf, uint16_t len);

#endif /* W5500_SPI_H */
