#include "dw3000.h"

#define PIN_RST 27
#define PIN_IRQ 34
#define PIN_SS 4

#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

#define CIR_SIZE 1016
#define ACCUM_BUFFER_LEN (CIR_SIZE * 4)
#define FRAME_LEN 12

uint8_t rx_buffer[127];
uint8_t acc_buffer[ACCUM_BUFFER_LEN];
uint8_t tx_frame[FRAME_LEN] = {0xC5, 0, 0, 0, 'H', 'E', 'L', 'L', 'O', 'U', 'W', 'B'};

extern dwt_txconfig_t txconfig_options;

/* Default communication configuration. We use default non-STS DW mode. */
static dwt_config_t config = {
    5,                /* Channel number. */
    DWT_PLEN_128,     /* Preamble length. Used in TX only. */
    DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
    9,                /* TX preamble code. Used in TX only. */
    9,                /* RX preamble code. Used in RX only. */
    1,                /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,       /* Data rate. */
    DWT_PHRMODE_STD,  /* PHY header mode. */
    DWT_PHRRATE_STD,  /* PHY header rate. */
    (129 + 8 - 8),    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF, /* STS disabled */
    DWT_STS_LEN_64,   /* STS length see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0       /* PDOA mode off */
};

void setup() {
  UART_init();
  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_SS);

  Serial.begin(9600);

  delay(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)

  while (!dwt_checkidlerc()) // Need to make sure DW IC is in IDLE_RC before proceeding
  {
    UART_puts("IDLE FAILED\r\n");
    while (1)
      ;
  }

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
  {
    UART_puts("INIT FAILED\r\n");
    while (1)
      ;
  }

  // Enabling LEDs here for debug so that for each TX the D1 LED will flash on DW3000 red eval-shield boards.
  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

  /* Configure DW IC. See NOTE 6 below. */
  if (dwt_configure(&config)) // if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device
  {
    UART_puts("CONFIG FAILED\r\n");
    while (1)
      ;
  }

  /* Configure the TX spectrum parameters (power, PG delay and PG count) */
  dwt_configuretxrf(&txconfig_options);

  /* Apply default antenna delay value. See NOTE 2 below. */
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);

  /* Next can enable TX/RX states output on GPIOs 5 and 6 to help debug, and also TX/RX LEDs
   * Note, in real low power applications the LEDs should not be used. */
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

  Serial.println("CIR Extractor");
  Serial.println("Setup over........");
}

void loop() {
  // Step 1: Load and transmit frame
  dwt_writetxdata(FRAME_LEN, tx_frame, 0);
  dwt_writetxfctrl(FRAME_LEN, 0, 1);  // Enable ranging bit
  dwt_starttx(DWT_START_TX_IMMEDIATE);

  // Step 2: Immediately enable RX
  dwt_rxenable(DWT_START_RX_IMMEDIATE);

  // Step 3: Wait for RX
  uint32_t status = dwt_read32bitreg(SYS_STATUS_ID);
  if (status & SYS_STATUS_RXFCG_BIT) {
    uint16_t len = dwt_read16bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
    dwt_readrxdata(rx_buffer, len, 0);
    dwt_readaccdata(acc_buffer, ACCUM_BUFFER_LEN, 0);

    Serial.println("START_CIR");
    for (int i = 0; i < CIR_SIZE; i++) {
      int16_t real = ((int16_t*)acc_buffer)[i * 2];
      int16_t imag = ((int16_t*)acc_buffer)[i * 2 + 1];
      Serial.print(real);
      Serial.print(",");
      Serial.println(imag);
    }
    Serial.println("END_CIR");

    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_GOOD);
  } else {
    Serial.println("RX timeout or error.");
  }
}
