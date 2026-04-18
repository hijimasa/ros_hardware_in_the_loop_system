/*
 * W5500 SPI driver for W5500-EVB-Pico2
 *
 * Provides low-level SPI communication and simplified UDP socket API
 * for W5500 Ethernet controller. Uses direct register access rather
 * than ioLibrary_Driver to minimize dependencies.
 *
 * W5500 register map reference:
 *   Common registers: block 0x00
 *   Socket n registers: block (n*4 + 1) << 3
 *   Socket n TX buffer: block (n*4 + 2) << 3
 *   Socket n RX buffer: block (n*4 + 3) << 3
 */

#include "w5500_spi.h"

#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

#include <string.h>
#include <stdio.h>

/* ── W5500 register definitions ── */

/* Block Select Bits (BSB) */
#define W5500_COMMON_REG    0x00
#define W5500_SOCK_REG(n)   ((n) * 4 + 1)
#define W5500_SOCK_TX(n)    ((n) * 4 + 2)
#define W5500_SOCK_RX(n)    ((n) * 4 + 3)

/* Control byte: BSB[4:0] << 3 | RWB | OM[1:0] */
#define W5500_CTRL(block, rw)  (((block) << 3) | ((rw) << 2) | 0x00)
#define W5500_READ   0
#define W5500_WRITE  1

/* Common registers */
#define W5500_MR        0x0000  /* Mode Register */
#define W5500_GAR       0x0001  /* Gateway Address (4 bytes) */
#define W5500_SUBR      0x0005  /* Subnet Mask (4 bytes) */
#define W5500_SHAR      0x0009  /* Source MAC Address (6 bytes) */
#define W5500_SIPR      0x000F  /* Source IP Address (4 bytes) */
#define W5500_PHYCFGR   0x002E  /* PHY Configuration */

/* Socket registers (offsets within socket register block) */
#define Sn_MR       0x0000  /* Mode */
#define Sn_CR       0x0001  /* Command */
#define Sn_IR       0x0002  /* Interrupt */
#define Sn_SR       0x0003  /* Status */
#define Sn_PORT     0x0004  /* Source Port (2 bytes) */
#define Sn_DIPR     0x000C  /* Destination IP (4 bytes) */
#define Sn_DPORT    0x0010  /* Destination Port (2 bytes) */
#define Sn_TXBUF_SIZE  0x001F
#define Sn_RXBUF_SIZE  0x001E
#define Sn_TX_FSR   0x0020  /* TX Free Size (2 bytes) */
#define Sn_TX_RD    0x0022  /* TX Read Pointer (2 bytes) */
#define Sn_TX_WR    0x0024  /* TX Write Pointer (2 bytes) */
#define Sn_RX_RSR   0x0026  /* RX Received Size (2 bytes) */
#define Sn_RX_RD    0x0028  /* RX Read Pointer (2 bytes) */
#define Sn_RX_WR    0x002A  /* RX Write Pointer (2 bytes) */

/* Socket commands */
#define SOCK_CMD_OPEN   0x01
#define SOCK_CMD_CLOSE  0x10
#define SOCK_CMD_SEND   0x20
#define SOCK_CMD_RECV   0x40

/* Socket modes */
#define SOCK_MR_UDP     0x02

/* Socket status */
#define SOCK_STATUS_CLOSED  0x00
#define SOCK_STATUS_UDP     0x22

/* ── SPI low-level ── */

static void spi_write_reg(uint8_t block, uint16_t addr,
                           const uint8_t *data, uint16_t len)
{
    uint8_t hdr[3];
    hdr[0] = (addr >> 8) & 0xFF;
    hdr[1] = addr & 0xFF;
    hdr[2] = W5500_CTRL(block, W5500_WRITE);

    gpio_put(W5500_PIN_CS, 0);
    spi_write_blocking(W5500_SPI_PORT, hdr, 3);
    spi_write_blocking(W5500_SPI_PORT, data, len);
    gpio_put(W5500_PIN_CS, 1);
}

static void spi_read_reg(uint8_t block, uint16_t addr,
                          uint8_t *data, uint16_t len)
{
    uint8_t hdr[3];
    hdr[0] = (addr >> 8) & 0xFF;
    hdr[1] = addr & 0xFF;
    hdr[2] = W5500_CTRL(block, W5500_READ);

    gpio_put(W5500_PIN_CS, 0);
    spi_write_blocking(W5500_SPI_PORT, hdr, 3);
    spi_read_blocking(W5500_SPI_PORT, 0x00, data, len);
    gpio_put(W5500_PIN_CS, 1);
}

static void write_reg8(uint8_t block, uint16_t addr, uint8_t val) {
    spi_write_reg(block, addr, &val, 1);
}

static uint8_t read_reg8(uint8_t block, uint16_t addr) {
    uint8_t val;
    spi_read_reg(block, addr, &val, 1);
    return val;
}

static uint16_t read_reg16(uint8_t block, uint16_t addr) {
    uint8_t buf[2];
    spi_read_reg(block, addr, buf, 2);
    return ((uint16_t)buf[0] << 8) | buf[1];
}

static void write_reg16(uint8_t block, uint16_t addr, uint16_t val) {
    uint8_t buf[2] = { (val >> 8) & 0xFF, val & 0xFF };
    spi_write_reg(block, addr, buf, 2);
}

/* ── Public API ── */

void w5500_spi_init(void)
{
    /* Initialize SPI0 */
    spi_init(W5500_SPI_PORT, W5500_SPI_CLK_HZ);
    gpio_set_function(W5500_PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(W5500_PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(W5500_PIN_MISO, GPIO_FUNC_SPI);

    /* CS pin - manual control */
    gpio_init(W5500_PIN_CS);
    gpio_set_dir(W5500_PIN_CS, GPIO_OUT);
    gpio_put(W5500_PIN_CS, 1);

    /* RST pin */
    gpio_init(W5500_PIN_RST);
    gpio_set_dir(W5500_PIN_RST, GPIO_OUT);

    /* Hardware reset */
    gpio_put(W5500_PIN_RST, 0);
    sleep_ms(10);
    gpio_put(W5500_PIN_RST, 1);
    sleep_ms(100);

    /* Software reset */
    write_reg8(W5500_COMMON_REG, W5500_MR, 0x80);
    sleep_ms(100);

    printf("[W5500] SPI initialized, hardware reset complete\n");
}

void w5500_network_init(const uint8_t ip[4], const uint8_t subnet[4],
                        const uint8_t gateway[4], const uint8_t mac[6])
{
    spi_write_reg(W5500_COMMON_REG, W5500_SHAR, mac, 6);
    spi_write_reg(W5500_COMMON_REG, W5500_GAR, gateway, 4);
    spi_write_reg(W5500_COMMON_REG, W5500_SUBR, subnet, 4);
    spi_write_reg(W5500_COMMON_REG, W5500_SIPR, ip, 4);

    printf("[W5500] Network: %d.%d.%d.%d\n", ip[0], ip[1], ip[2], ip[3]);
}

int w5500_udp_open(uint16_t local_port)
{
    /* Find a free socket (0-7) */
    for (int s = 0; s < 8; s++) {
        uint8_t sr = read_reg8(W5500_SOCK_REG(s), Sn_SR);
        if (sr == SOCK_STATUS_CLOSED) {
            /* Configure as UDP */
            write_reg8(W5500_SOCK_REG(s), Sn_MR, SOCK_MR_UDP);
            write_reg16(W5500_SOCK_REG(s), Sn_PORT, local_port);
            /* Set buffer sizes: 2KB each (default) */
            write_reg8(W5500_SOCK_REG(s), Sn_TXBUF_SIZE, 2);
            write_reg8(W5500_SOCK_REG(s), Sn_RXBUF_SIZE, 2);
            /* Open */
            write_reg8(W5500_SOCK_REG(s), Sn_CR, SOCK_CMD_OPEN);
            sleep_ms(1);

            sr = read_reg8(W5500_SOCK_REG(s), Sn_SR);
            if (sr == SOCK_STATUS_UDP) {
                printf("[W5500] Socket %d opened, port %d\n", s, local_port);
                return s;
            } else {
                write_reg8(W5500_SOCK_REG(s), Sn_CR, SOCK_CMD_CLOSE);
                return -1;
            }
        }
    }
    return -1;  /* No free sockets */
}

int w5500_udp_sendto(int sock, const uint8_t *data, uint16_t len,
                     const uint8_t dest_ip[4], uint16_t dest_port)
{
    if (sock < 0 || sock > 7 || len == 0) return -1;

    uint8_t block = W5500_SOCK_REG(sock);

    /* Set destination */
    spi_write_reg(block, Sn_DIPR, dest_ip, 4);
    write_reg16(block, Sn_DPORT, dest_port);

    /* Wait for TX buffer space */
    uint16_t freesize;
    int timeout = 1000;
    do {
        freesize = read_reg16(block, Sn_TX_FSR);
        if (--timeout == 0) return -1;
    } while (freesize < len);

    /* Write data to TX buffer */
    uint16_t ptr = read_reg16(block, Sn_TX_WR);
    uint16_t offset = ptr & 0x07FF;  /* 2KB buffer mask */

    uint8_t tx_block = W5500_SOCK_TX(sock);
    if (offset + len <= 0x0800) {
        spi_write_reg(tx_block, offset, data, len);
    } else {
        /* Wrap around */
        uint16_t first = 0x0800 - offset;
        spi_write_reg(tx_block, offset, data, first);
        spi_write_reg(tx_block, 0, data + first, len - first);
    }

    /* Update write pointer and send */
    write_reg16(block, Sn_TX_WR, ptr + len);
    write_reg8(block, Sn_CR, SOCK_CMD_SEND);

    /* Wait for send complete */
    timeout = 1000;
    while (--timeout > 0) {
        uint8_t ir = read_reg8(block, Sn_IR);
        if (ir & 0x10) {  /* SEND_OK */
            write_reg8(block, Sn_IR, 0x10);
            return len;
        }
        if (ir & 0x08) {  /* TIMEOUT */
            write_reg8(block, Sn_IR, 0x08);
            return -1;
        }
    }
    return -1;
}

int w5500_udp_recvfrom(int sock, uint8_t *buf, uint16_t buf_size,
                       uint8_t src_ip[4], uint16_t *src_port)
{
    if (sock < 0 || sock > 7) return -1;

    uint8_t block = W5500_SOCK_REG(sock);
    uint16_t rx_size = read_reg16(block, Sn_RX_RSR);
    if (rx_size == 0) return 0;

    /* Read from RX buffer */
    uint16_t ptr = read_reg16(block, Sn_RX_RD);
    uint16_t offset = ptr & 0x07FF;

    uint8_t rx_block = W5500_SOCK_RX(sock);

    /* UDP header in W5500 RX buffer: 4 bytes IP + 2 bytes port + 2 bytes data_len */
    uint8_t udp_header[8];
    if (offset + 8 <= 0x0800) {
        spi_read_reg(rx_block, offset, udp_header, 8);
    } else {
        uint16_t first = 0x0800 - offset;
        spi_read_reg(rx_block, offset, udp_header, first);
        spi_read_reg(rx_block, 0, udp_header + first, 8 - first);
    }

    if (src_ip) memcpy(src_ip, udp_header, 4);
    if (src_port) *src_port = ((uint16_t)udp_header[4] << 8) | udp_header[5];
    uint16_t data_len = ((uint16_t)udp_header[6] << 8) | udp_header[7];

    if (data_len > buf_size) data_len = buf_size;

    /* Read payload */
    uint16_t data_offset = (offset + 8) & 0x07FF;
    if (data_offset + data_len <= 0x0800) {
        spi_read_reg(rx_block, data_offset, buf, data_len);
    } else {
        uint16_t first = 0x0800 - data_offset;
        spi_read_reg(rx_block, data_offset, buf, first);
        spi_read_reg(rx_block, 0, buf + first, data_len - first);
    }

    /* Update read pointer */
    write_reg16(block, Sn_RX_RD, ptr + 8 + data_len);
    write_reg8(block, Sn_CR, SOCK_CMD_RECV);

    return data_len;
}

void w5500_udp_close(int sock)
{
    if (sock < 0 || sock > 7) return;
    write_reg8(W5500_SOCK_REG(sock), Sn_CR, SOCK_CMD_CLOSE);
    printf("[W5500] Socket %d closed\n", sock);
}
