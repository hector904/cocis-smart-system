// Simulated MFRC522 (RC522) RFID reader for Wokwi.
//
// - Talks the real RC522 SPI register protocol that main.c uses
//   (address byte, then one data byte; read = bit 7 of the address set).
// - VersionReg returns 0x92, so main.c detects the reader (RF:OK).
// - A virtual card with UID A1B2C3D4 (the registered demo card) is "in the
//   field" while the CARD pin is held LOW. Wire a pushbutton between CARD and
//   GND and press it to tap the card.
// - After the card was read and halted it will not answer again until the
//   button is released, exactly like a real HLTA'd card.

#include "wokwi-api.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define REG_COMMAND 0x01
#define REG_COMIRQ 0x04
#define REG_DIVIRQ 0x05
#define REG_ERROR 0x06
#define REG_FIFODATA 0x09
#define REG_FIFOLEVEL 0x0A
#define REG_BITFRAMING 0x0D
#define REG_CRC_H 0x21
#define REG_CRC_L 0x22
#define REG_VERSION 0x37

#define CMD_CALCCRC 0x03
#define CMD_TRANSCEIVE 0x0C
#define CMD_SOFTRESET 0x0F

static const uint8_t CARD_UID[4] = {0xA1, 0xB2, 0xC3, 0xD4};

typedef struct
{
  pin_t cs;
  pin_t card;
  spi_dev_t spi;
  uint8_t buf[1];
  bool addr_phase;
  bool is_read;
  uint8_t reg;
  uint8_t regs[64];
  uint8_t fifo[64];
  uint8_t rd;
  uint8_t wr;
  bool halted;
} chip_state_t;

/* CRC_A (ISO/IEC 14443-A), preset 0x6363 */
static void crc_a(const uint8_t *data, uint8_t len, uint8_t *lo, uint8_t *hi)
{
  uint16_t crc = 0x6363;
  for (uint8_t i = 0; i < len; i++)
  {
    uint8_t b = data[i];
    b = b ^ (uint8_t)(crc & 0xFF);
    b = b ^ (uint8_t)(b << 4);
    crc = (uint16_t)((crc >> 8) ^ ((uint16_t)b << 8) ^ ((uint16_t)b << 3) ^ ((uint16_t)b >> 4));
  }
  *lo = (uint8_t)(crc & 0xFF);
  *hi = (uint8_t)(crc >> 8);
}

static void fifo_flush(chip_state_t *c)
{
  c->rd = 0;
  c->wr = 0;
}

static void fifo_push(chip_state_t *c, uint8_t v)
{
  if (c->wr < sizeof(c->fifo))
    c->fifo[c->wr++] = v;
}

static void chip_soft_reset(chip_state_t *c)
{
  memset(c->regs, 0, sizeof(c->regs));
  c->regs[REG_COMMAND] = 0x20; /* RcvOff set, PowerDown clear */
  c->regs[REG_VERSION] = 0x92;
  fifo_flush(c);
}

/* Card side of one Transceive command */
static void do_transceive(chip_state_t *c, uint8_t last_bits)
{
  uint8_t tx[64];
  uint8_t n = (uint8_t)(c->wr - c->rd);
  memcpy(tx, &c->fifo[c->rd], n);
  fifo_flush(c);
  c->regs[REG_ERROR] = 0;

  uint8_t rsp[8];
  uint8_t rl = 0;
  bool present = (pin_read(c->card) == LOW) && !c->halted;

  if (present)
  {
    if (n == 1 && (tx[0] == 0x26 || tx[0] == 0x52) && last_bits == 7)
    {
      /* REQA / WUPA -> ATQA */
      rsp[0] = 0x04;
      rsp[1] = 0x00;
      rl = 2;
    }
    else if (n == 2 && tx[0] == 0x93 && tx[1] == 0x20)
    {
      /* Anticollision cascade level 1 -> UID + BCC */
      memcpy(rsp, CARD_UID, 4);
      rsp[4] = CARD_UID[0] ^ CARD_UID[1] ^ CARD_UID[2] ^ CARD_UID[3];
      rl = 5;
    }
    else if (n == 9 && tx[0] == 0x93 && tx[1] == 0x70)
    {
      /* SELECT -> SAK + CRC_A (UID complete, bit 3 cascade = 0) */
      rsp[0] = 0x08;
      crc_a(rsp, 1, &rsp[1], &rsp[2]);
      rl = 3;
    }
    else if (n == 4 && tx[0] == 0x50 && tx[1] == 0x00)
    {
      /* HLTA: card goes silent and never answers */
      c->halted = true;
    }
  }

  if (rl)
  {
    for (uint8_t i = 0; i < rl; i++)
      fifo_push(c, rsp[i]);
    c->regs[REG_COMIRQ] = 0x30; /* RxIRq | IdleIRq */
  }
  else
  {
    c->regs[REG_COMIRQ] = 0x01; /* TimerIRq: no answer */
  }
  c->regs[REG_COMMAND] = 0x00;
}

static uint8_t reg_read(chip_state_t *c, uint8_t reg)
{
  if (reg == REG_FIFODATA)
    return (c->rd < c->wr) ? c->fifo[c->rd] : 0x00;
  if (reg == REG_FIFOLEVEL)
    return (uint8_t)(c->wr - c->rd);
  return c->regs[reg];
}

static void reg_read_done(chip_state_t *c, uint8_t reg)
{
  if (reg == REG_FIFODATA && c->rd < c->wr)
    c->rd++;
}

static void reg_write(chip_state_t *c, uint8_t reg, uint8_t val)
{
  switch (reg)
  {
  case REG_COMMAND:
    if ((val & 0x0F) == CMD_SOFTRESET)
    {
      chip_soft_reset(c);
    }
    else if ((val & 0x0F) == CMD_CALCCRC)
    {
      uint8_t lo, hi;
      crc_a(&c->fifo[c->rd], (uint8_t)(c->wr - c->rd), &lo, &hi);
      fifo_flush(c);
      c->regs[REG_CRC_L] = lo;
      c->regs[REG_CRC_H] = hi;
      c->regs[REG_DIVIRQ] |= 0x04; /* CRCIRq */
      c->regs[REG_COMMAND] = 0x00;
    }
    else
    {
      c->regs[REG_COMMAND] = val;
    }
    break;

  case REG_COMIRQ:
  case REG_DIVIRQ:
    if (val & 0x80)
      c->regs[reg] |= (val & 0x7F);
    else
      c->regs[reg] &= (uint8_t)~val;
    break;

  case REG_FIFODATA:
    fifo_push(c, val);
    break;

  case REG_FIFOLEVEL:
    if (val & 0x80)
      fifo_flush(c);
    break;

  case REG_BITFRAMING:
    c->regs[reg] = val;
    if ((val & 0x80) && (c->regs[REG_COMMAND] & 0x0F) == CMD_TRANSCEIVE)
      do_transceive(c, val & 0x07);
    break;

  case REG_VERSION:
    break; /* read-only */

  default:
    c->regs[reg] = val;
    break;
  }
}

/* One SPI byte finished. The master sends 2-byte transactions: address, then data. */
static void chip_spi_done(void *user_data, uint8_t *buffer, uint32_t count)
{
  chip_state_t *c = (chip_state_t *)user_data;
  if (!count)
    return; /* transfer was cancelled because CS went high */

  if (c->addr_phase)
  {
    uint8_t b = buffer[0];
    c->is_read = (b & 0x80) != 0;
    c->reg = (uint8_t)((b >> 1) & 0x3F);
    c->addr_phase = false;
    buffer[0] = c->is_read ? reg_read(c, c->reg) : 0x00;
    spi_start(c->spi, buffer, 1); /* second byte: data in / data out */
  }
  else
  {
    if (c->is_read)
      reg_read_done(c, c->reg);
    else
      reg_write(c, c->reg, buffer[0]);
    c->addr_phase = true;
    buffer[0] = 0x00;
  }
}

static void chip_pin_change(void *user_data, pin_t pin, uint32_t value)
{
  chip_state_t *c = (chip_state_t *)user_data;

  if (pin == c->cs)
  {
    if (value == LOW)
    {
      c->addr_phase = true;
      c->buf[0] = 0x00;
      spi_start(c->spi, c->buf, 1);
    }
    else
    {
      spi_stop(c->spi);
    }
  }
  else if (pin == c->card && value == HIGH)
  {
    c->halted = false; /* card left the field */
  }
}

void chip_init(void)
{
  chip_state_t *c = (chip_state_t *)calloc(1, sizeof(chip_state_t));
  chip_soft_reset(c);
  c->addr_phase = true;

  c->cs = pin_init("SDA", INPUT_PULLUP);
  c->card = pin_init("CARD", INPUT_PULLUP);

  const pin_watch_config_t watch = {
      .edge = BOTH,
      .pin_change = chip_pin_change,
      .user_data = c,
  };
  pin_watch(c->cs, &watch);
  pin_watch(c->card, &watch);

  const spi_config_t spi_config = {
      .sck = pin_init("SCK", INPUT),
      .mosi = pin_init("MOSI", INPUT),
      .miso = pin_init("MISO", INPUT),
      .mode = 0,
      .done = chip_spi_done,
      .user_data = c,
  };
  c->spi = spi_init(&spi_config);
}
