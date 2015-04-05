/* Operator definitions.
 */
#define CALLSIGN "KC3XM "
#define LOCATOR  "FM19"
#define POWER    30

/* Transceiver frequency definitions.
 * BAND_CENTER      Center frequency for WSPR band of interest, in Hz.
 * RX_FREQ          Si5351 frequency in receive mode, in Hz.
 * TX_FREQ          Si5351 center frequency in transmit mode, in Hz.
 * CALIBRATE_FREQ   Si5351 frequency in calibrate mode, in Hz.
 */
#define BAND_CENTER           10140200
#define RX_FREQ               10125000
#define TX_FREQ               10140200
#define CALIBRATE_FREQ         9985000

/* Reference frequency corrections, in parts/1x10^7
 * TX_CORRECTION        Correction applied when in receive/transmit mode.
 * CALIBRATE_CORRECTION Correction applied when in calibrate mode.
 */
#define TX_CORRECTION         -936
#define CALIBRATE_CORRECTION  -936

/* WSPR data
 * KC3XM FM19 30
 */
uint8 wsprData[] = {
    3, 3, 2, 0, 0, 0, 0, 0, 3, 0, 0, 2, 3, 1, 3, 0, 0, 2, 
    3, 2, 2, 3, 2, 3, 1, 3, 1, 2, 2, 0, 0, 0, 2, 0, 3, 2, 
    2, 1, 0, 3, 2, 2, 2, 0, 2, 2, 3, 2, 3, 1, 0, 0, 3, 3, 
    0, 3, 0, 0, 0, 1, 3, 2, 1, 0, 0, 0, 2, 1, 1, 0, 3, 2, 
    1, 2, 3, 2, 1, 0, 0, 3, 0, 0, 1, 2, 3, 3, 2, 2, 0, 3,
    3, 2, 3, 2, 1, 2, 2, 0, 1, 2, 0, 0, 0, 0, 3, 0, 0, 1, 
    0, 2, 1, 1, 1, 0, 3, 1, 2, 2, 1, 1, 0, 1, 2, 2, 0, 1, 
    1, 1, 2, 0, 2, 2, 0, 3, 0, 1, 2, 0, 3, 1, 0, 2, 0, 2, 
    0, 2, 0, 3, 1, 0, 1, 2, 3, 1, 2, 0, 2, 3, 3, 2, 0, 2
};
