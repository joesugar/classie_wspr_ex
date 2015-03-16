#include "si5351.h"
#include "Wire.h"

/*
 * Si5351 WSPR Transmitter
 *
 * Copyright (C) 2014 Joseph Consugar (joesugar@erols.com)
 *
 * Advanced controller for the Classie WSPR transceiver.
 *
 * This program is placed in the public domain and is distributed
 * in the hope that it will be useful but WITHOUT ANY WARRANTY, 
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR 
 * A PARTICULAR PURPOSE.
 *
 * Uses the Goertzel algorithm to look for a transmit tone from the
 * the WSPR software.  Goes to transmit when tone is received.
 *
 * Goertzel algorithm taken from 
 * http://www.embedded.com/design/configurable-systems/4024443/The-Goertzel-Algorithm
 *
 * Moving average filter taken from 
 * http://skovholm.com/cwdecoder
 *
 * High speed A2D code taken from
 * http://www.instructables.com/id/Arduino-Audio-Input/?ALLSTEPS
 */
 
#define uint32  unsigned long
#define uint16  unsigned int
#define uint8   unsigned char
#define int32   long
#define int16   int
#define int8    char

#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE ~FALSE
#endif

#define VOX_HIGH  1
#define VOX_LOW   0

#define CALLSIGN "KC3XM "
#define LOCATOR  "FM19"
#define POWER    30

/* Transmitter declarations.
 *
 * Defines
 */
#define BIT_PERIOD_NUMBER   8192
#define BIT_PERIOD_DENOM    12000
#define BIT_PERIOD_MULT     12

#define TRANSMIT_PERIOD_NUMBER  120000
#define TRANSMIT_PERIOD_DENOM   1
#define TRANSMIT_PERCENTAGE     25

#define RX_FREQ             10125000
#define TX_FREQ             10140200
#define CALIBRATE_FREQ       9985000
//#define TX_FREQ_SHIFT       (BIT_PERIOD_DENOM / BIT_PERIOD_NUMBER)
#define TX_FREQ_SHIFT       2
#define TX_CORRECTION       -880
#define ONE_SECOND          1000
#define NUMBER_OF_MSG_BITS  162

#define WAIT_FOR_TRANSMIT_TIMEOUT   0
#define WAIT_FOR_TRANSMIT_START     1
#define WAIT_FOR_TRANSMIT_COMPLETE  2
#define WAIT_FOR_CALIBRATE_COMPLETE 3

#define POWER_UP            0x00
#define POWER_DOWN          0x04

#define CALIBRATE           8
#define VCONTROL            9
#define LED                 13

#define ENABLE_TRANSMIT     digitalWrite(VCONTROL, LOW)
#define ENABLE_TRANSMIT_LED digitalWrite(LED, HIGH)
#define ENABLE_RECEIVE      digitalWrite(VCONTROL, HIGH)
#define ENABLE_RECEIVE_LED  digitalWrite(LED, LOW)

/* Variable declarations.
 */
uint32 lastTime = 0;
uint32 currentTime = 0;
uint32 deltaTime = 0;

uint32 bitTimer = BIT_PERIOD_NUMBER;
uint32 deltaBitTimer = 0;

uint32 deltaTransmitTimer = 0;
uint16 transmitAcc = 0;

uint32 transmitStartTimer = 0;
uint8  transmitState = WAIT_FOR_TRANSMIT_TIMEOUT;

uint32 frequencyMin = 0;
uint8 bitIndex = 0;

/* Goertzel algorithm declarations.
 *
 * Defines
 */
#define AUDIO_PIN   A1

/* Variable declarations.
 *
 * N is the number of audio samples to be processed.
 * decimation is the sample decimation rate.
 * k is the frequency band of interest.
 * bandwidth = sampling frequency / (N * decimation) 
 * band center frequency = k * bandwidth
 *
 * The target frequency corresponds to the transmitted
 * frequency of 15200 after decimation.
 */
const float sampling_freq = 38462.0;
const float target_freq = 777.0;
const int N = 25;
const int decimation = 8;

/* Define various ADC prescaler
 */
const unsigned char PS_16  = (1 << ADPS2);
const unsigned char PS_32  = (1 << ADPS2) | (1 << ADPS0);
const unsigned char PS_64  = (1 << ADPS2) | (1 << ADPS1);
const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

float magnitude;
int magnitudelimit = 100;
int magnitudelimit_low = 100;
int toneFound = LOW;

float coeff;
float cosine;
float sine;

float Q0 = 0;
float Q1 = 0;
float Q2 = 0;

byte incomingAudio = 0;
int number_of_samples = 0;
int decimation_counter = 0;

volatile int sample_index = 0;
volatile unsigned int testData[N * decimation];
volatile boolean doSampling = false; 
volatile unsigned int sample = 0;

uint8   vox_last_sample = VOX_LOW;
boolean vox_rising_edge = false;
boolean first_test_time = false;

/* WSPR data
 * KC3XM FM19 10
 */
/*
uint8 wsprData[] = {
    3, 3, 2, 2, 0, 0, 0, 0, 3, 0, 0, 2, 3, 3, 3, 0, 0, 2,
    3, 0, 2, 1, 2, 1, 1, 1, 1, 0, 2, 0, 0, 2, 2, 0, 3, 2, 
    2, 1, 0, 3, 2, 2, 2, 2, 2, 2, 3, 0, 3, 3, 0, 2, 3, 1, 
    0, 3, 0, 0, 0, 1, 3, 2, 1, 0, 0, 0, 2, 1, 1, 0, 3, 0, 
    1, 0, 3, 2, 1, 2, 0, 1, 0, 0, 1, 2, 3, 3, 2, 0, 0, 1,
    3, 0, 3, 0, 1, 2, 2, 0, 1, 2, 0, 0, 0, 2, 3, 0, 0, 3, 
    0, 2, 1, 3, 1, 2, 3, 1, 2, 0, 1, 3, 0, 1, 2, 2, 0, 3, 
    1, 1, 2, 0, 2, 0, 0, 1, 0, 3, 2, 2, 3, 3, 0, 0, 0, 2, 
    0, 2, 0, 3, 1, 2, 1, 0, 3, 3, 2, 2, 2, 3, 3, 2, 0, 2
};
*/

/* WSPR data
 * KC3XM FM19 23
 */
/*
uint8 wsprData[] = {
    3, 3, 2, 2, 0, 0, 0, 0, 3, 2, 0, 0, 3, 1, 3, 2, 0, 2,
    3, 2, 2, 3, 2, 3, 1, 1, 1, 0, 2, 2, 0, 2, 2, 0, 3, 2, 
    2, 3, 0, 1, 2, 0, 2, 2, 2, 0, 3, 0, 3, 1, 0, 0, 3, 3, 
    0, 1, 0, 2, 0, 1, 3, 0, 1, 2, 0, 0, 2, 1, 1, 2, 3, 0, 
    1, 2, 3, 2, 1, 2, 0, 3, 0, 0, 1, 2, 3, 3, 2, 2, 0, 1,
    3, 2, 3, 2, 1, 0, 2, 0, 1, 0, 0, 2, 0, 0, 3, 0, 0, 3, 
    0, 2, 1, 3, 1, 0, 3, 3, 2, 0, 1, 1, 0, 3, 2, 0, 0, 3, 
    1, 3, 2, 0, 2, 0, 0, 1, 0, 3, 2, 2, 3, 1, 0, 0, 0, 0, 
    0, 2, 0, 3, 1, 2, 1, 2, 3, 3, 2, 0, 2, 3, 3, 0, 0, 2
};
*/

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

/* Si5351 setup routine.
 */   
Si5351 si5351;

void setup()
{
    /* Initialize transmit variables.
     */
    setupGoertzel();

    /* Initialize the output pins.
     */
    pinMode(VCONTROL, OUTPUT);
    pinMode(LED, OUTPUT);
    pinMode(CALIBRATE, INPUT_PULLUP); 
    ENABLE_RECEIVE;
    ENABLE_RECEIVE_LED;   

    /* Initialize the transmit state.
     */    
    lastTime = millis();
    transmitState = (digitalRead(CALIBRATE) == LOW) ? 
        WAIT_FOR_CALIBRATE_COMPLETE : WAIT_FOR_TRANSMIT_TIMEOUT;
    transmitAcc = 0;     
    
    /* Initialize the Si5351
     */
    si5351.init(SI5351_CRYSTAL_LOAD_8PF);
    si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_2MA);
    
    /* Enable/disable the clocks.
     */
    si5351.clock_enable(SI5351_CLK0, 0);
    si5351.clock_enable(SI5351_CLK1, 0);
    si5351.clock_enable(SI5351_CLK2, 1);
  
    /* Start in receive mode.
     */
    SetFrequency(RX_FREQ, POWER_DOWN);

    /* Configure the ADC
     */
    setupADC(); 
}

/* Interrupt service routine.
 */
ISR(ADC_vect) 
{
    if (doSampling)
    {
        /* Update value with high 8 bits from the ADC.
         */
        incomingAudio = ADCH;        
        testData[sample_index++] = (((unsigned int)incomingAudio) << 2);
        doSampling = (sample_index < (N * decimation));
    }
}

/* WSPR main loop.
 */
void loop()
{
    /* Update the values used to track time.
     */
    currentTime = millis();
    deltaTime = currentTime - lastTime;
    lastTime = currentTime;

    /* Check the vox.
     */
    doGoertzel();
    uint8 transmitFlag = GetTransmitFlag();
    vox_rising_edge = ((transmitFlag) && (vox_last_sample == VOX_LOW));
    vox_last_sample = (transmitFlag) ? VOX_HIGH : VOX_LOW;
    
    /* Update the timer values.
     *
     * The idea is to decrement time timer by an amount corresponding
     * to the amount of time that's passed.  When the timer goes < 0
     * it's time to process the timer event.
     *
     * Rather than wait for the timer to go < 0 we compare the current
     * value to the value by which it is to be decremented.  If the 
     * current value is < decrement value add the period to the timer.
     * That way the value will stay > 0 when the subtraction is done.
     *
     * Also, we preserve the remainder in the timer.  The result is that
     * while we may not get exact single bits it will be correct on
     * the average.
     */
    if (WAIT_FOR_CALIBRATE_COMPLETE == transmitState)
    {
        /* Trying to calibrate the oscillator.  Check the calibrate
         * pin level to decide what to do here.
         */
        int calibrate = digitalRead(CALIBRATE);
        if (calibrate == LOW)
        {
            /* Still trying to calibrate the oscillator.
             */
            if (first_test_time == TRUE)
            {
                /* First time in the state so set the correction to 0.
                 * Set the oscillator to the calibration frequency.
                 */
                si5351.set_correction(0);
                SetFrequency(CALIBRATE_FREQ, POWER_DOWN);
                first_test_time = false;
            }
        }
        else
        {
            /* Level not set so you are no longer trying to 
             * calibrate the oscillator. Go to the next state.
             */
            si5351.set_correction(TX_CORRECTION);
            SetFrequency(RX_FREQ, POWER_DOWN);             
            transmitState = WAIT_FOR_TRANSMIT_TIMEOUT;
        }
    }   
    else if (WAIT_FOR_TRANSMIT_TIMEOUT == transmitState)
    {
        /* If the vox indicates a tone is being transmitted set up the
         * delay before transmission.
         */
        if (vox_rising_edge)
        {
            ENABLE_TRANSMIT_LED;
            SetFrequencyList(TX_FREQ);
            transmitStartTimer = ONE_SECOND / 4;
            transmitState = WAIT_FOR_TRANSMIT_START;
        }
    }
    else if (WAIT_FOR_TRANSMIT_START == transmitState)
    {
        /* At the beginning of a 2 minute transmit period.
         *
         * At this point we're going to check for false alarms.  Each time
         * through we're going to check the status of the transmit tone.  
         * If this is a legitimate transmit time the tone should still be there.
         * If it's not we'll assume it was a false alarm and go back to waiting 
         * for the transmit tone.
         */
        uint8 transmitStartTimedOut = UpdateTransmitStartTimer(&transmitStartTimer, deltaTime);
        if (transmitStartTimedOut)
        {
            /* End of transmit delay.  Begin transmission.
             */
            bitIndex = 1;
            bitTimer = BIT_PERIOD_NUMBER;
            SetFrequency(frequencyList(wsprData[0]), POWER_UP);
            transmitState = WAIT_FOR_TRANSMIT_COMPLETE;
        }
        else if (!transmitFlag)
        {
            /* Still within the transmit delay.  If tone is not there it must have
             * been a false alarm so jump back to wait for transmit.
             */
            ENABLE_RECEIVE_LED;
            transmitState = WAIT_FOR_TRANSMIT_TIMEOUT;
        }
    }
    else if (WAIT_FOR_TRANSMIT_COMPLETE == transmitState)
    {
        /* You are within a transmit period.
         */
        if (vox_rising_edge)
        {
            /* If you receive the rising edge of the vox during a transmission
             * there must be a timing eror.  Cancel the transmission.
             */
            SetFrequency(RX_FREQ, POWER_DOWN);
            ENABLE_RECEIVE_LED;
            transmitState = WAIT_FOR_TRANSMIT_TIMEOUT;
        }
        else
        {
            /* Check to see if it's time to move to the next bit
             */
            uint8 bitTimedOut = UpdateBitTimer(&bitTimer, deltaTime);
            if (bitTimedOut)
            {
                if (bitIndex <= NUMBER_OF_MSG_BITS)
                {
                    /* Still sending bits so set the frequency and update the index.
                     */
                    SetFrequency(frequencyList(wsprData[bitIndex]), POWER_UP);
                    bitIndex += 1;
                }
                else
                {
                    /* Done sending bits so move to the next state.
                     */
                    SetFrequency(RX_FREQ, POWER_DOWN);
                    ENABLE_RECEIVE_LED;
                    transmitState = WAIT_FOR_TRANSMIT_TIMEOUT;
                }
            }
        }
    }
}
    
/* Update the transmit start timer.
 */
uint8 UpdateTransmitStartTimer(uint32 *transmitStartTimer, uint32 deltaStartTimer)
{
    uint8 startTimerTimedOut = (*transmitStartTimer <= deltaStartTimer) ? TRUE : FALSE;
    if (startTimerTimedOut)
        *transmitStartTimer = *transmitStartTimer + ONE_SECOND;
    *transmitStartTimer = *transmitStartTimer - deltaStartTimer;
    return startTimerTimedOut;
}

/* Update the bit timer value.
 */
uint8 UpdateBitTimer(uint32 *bitTimer, uint32 deltaBitTimer)
{
    /* The change in the bit timer involves a multiplier because the timer
     * accumulator is scaled.  The period of a bit is (8192 / 12000).
     * This gives a condition for the timer of:
     *   acc + delta(sec) > (8192 / 12000).
     * To eliminate the fraction multiply through by 12000:
     *   acc + 12000 * delta(sec) > 8192 
     * delta(sec) = millis() / 1000.  Substituting, we get the condition:
     *   acc + 12 * millis() > 8192.
     * We actually do a decrement from the accumulator instead of an increment
     * but the idea still holds.
     */
    deltaBitTimer = BIT_PERIOD_MULT * deltaTime;
    uint8 bitTimedOut = (*bitTimer <= deltaBitTimer) ? TRUE : FALSE;
    if (bitTimedOut)
        *bitTimer += BIT_PERIOD_NUMBER;
    *bitTimer = *bitTimer - deltaBitTimer;
    return bitTimedOut;
}

/* Return the transmit flag.
 * The toneFound variable is set within the Goertzel algorithm routine.
 */
uint8 GetTransmitFlag()
{
    return (toneFound == HIGH);
}

/* Initialize the frequency list.
 */
void SetFrequencyList(uint32 frequency)
{
    frequencyMin = frequency - TX_FREQ_SHIFT - (TX_FREQ_SHIFT / 2);
}

/* Return the WSPR frequency for the specified index.
 */
uint32 frequencyList(uint8 index)
{
    return frequencyMin + index * TX_FREQ_SHIFT;
}

/* Set the DDS frequency.
 */
void SetFrequency(uint32 frequency, int8 power_up_down)
{
    /* Set the output frequency.
     */
    si5351.set_freq(frequency, 0, SI5351_CLK2); 

    /* Apply power to/from the amplifier.
     */
    if (power_up_down == POWER_UP)
        ENABLE_TRANSMIT;
    else
        ENABLE_RECEIVE;
}


/*-----------------------------------
 * ADC sampling routines.
 *-----------------------------------
 */
 
/* Setup for the Arduino ADC
 * When you leave this routine the ADC interrupt will not
 * be enabled.
 */
void setupADC()
{
    /* Set up continuous sampling of analog pin 0
     */
    cli();
    
    /* Clear ADCSRA and ADCSRB registers
     */
    ADCSRA = 0;
    ADCSRB = 0;
  
    /* Set the reference voltage.
     * Left align the ADC value so we can read the highest
     * 8 bits from the ADCH register.
     * Input on A1.
     */
    ADMUX |= (1 << REFS0);
    ADMUX |= (1 << ADLAR);
    ADMUX &= 0xF0;
    ADMUX |= 0x01;
  
    /* Set the ADC clock for clock / 16
     * 8 MHz / 16 = 500 kHz.
     */
    ADCSRA &= ~PS_128; 
    ADCSRA |= PS_16;
    
    /* Enable autotrigger.
     */
    ADCSRA |= (1 << ADATE);
    
    /* Enable interrupts when measurement is complete.
     */
    ADCSRA |= (1 << ADIE);
      
    /* Enable the ADC.
     */
    ADCSRA |= (1 << ADEN);
    
    /* Start the measurements.
     */
    ADCSRA |= (1 << ADSC);
    sei();
}

/* Set values to indicate start of ADC sampling.
 */
void startADCSamples()
{
    /* Initialize counter indices.
     */
    decimation_counter = decimation;
    number_of_samples = N;
    sample_index = 0;
    doSampling = true;
}


/*-----------------------------------
 * Goertzel algorithm routines.
 *-----------------------------------
 */
 
/* Setup for the Goertzel algorithm.
 */
void setupGoertzel()
{
    int k = (int)(.5 + (decimation * N * target_freq) / sampling_freq);
    float w = 2.0 * PI * (float)k / (float)N;
    
    cosine = cos(w);
    sine = sin(w);
    coeff = 2 * cosine;
}
    
/* Processing for the Goertzel algorithm.
 */
void doGoertzel() 
{
    /*
     * Sample the data.
     */ 
    startADCSamples();      
    while (doSampling);
    
    /*
     * Run it through the algorithm.
     */
    Q1 = 0;
    Q2 = 0;
    for (int i = 0, index = 7; i < N; i++, index += decimation) 
    {
        Q0 = coeff * Q1 - Q2 + testData[index];
        Q2 = Q1;
        Q1 = Q0;
    }
  
    /* 
     * Final calculations.
     * We're only interested in the magnitude.
     */   
    float magnitudeSquared = Q1 * Q1 + Q2 * Q2 - Q1 * Q2 * coeff;
    magnitude = sqrt(magnitudeSquared);

    /*
     * Here we'll try to set the magnitude limit automatic
     * using a moving average filter.
     */
    if (magnitude > magnitudelimit_low) 
    {
        magnitudelimit = (magnitudelimit + ((magnitude - magnitudelimit) / 6));
    }
    if (magnitudelimit < magnitudelimit_low) 
    {
        magnitudelimit = magnitudelimit_low;
    }
    
    /*
     * Now we check for the magnitude 
     */
    toneFound = (magnitude > magnitudelimit * 0.6) ? HIGH : LOW;
}
