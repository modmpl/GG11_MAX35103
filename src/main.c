#include <stdio.h>
#include <unistd.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "rtcdriver.h"
#include "spidrv.h"
#include "uartdrv.h"
#include "gpiointerrupt.h"

#include "max_macros.h"
#include "int_2hex.h"
#include <time.h>

/* @var SAVE_HEX_DATA/SAVE_ASCII_DATA  Specifies format in which MAX data is transmitted */
#define SAVE_ASCII_DATA

/* ----- SPI Declarations ----- */

/* @var SPI_TX_CONFIG_BUF_LENGTH  Configuration requires 3 bytes transferred */
#define SPI_TX_CONFIG_BUF_LENGTH 3
/* @var SPI_TX_BUF_LENGTH  OP code commands only transfer 1 byte and receive 2 bytes */
#define SPI_TX_BUF_LENGTH 1
/* @var SPI_RX_BUF_LENGTH  Arbitrary, 3 bytes per data register read */
// Needs 3 bytes per register despite 2 bytes of actual data because otherwise data gets overwritten
#define SPI_RX_BUF_LENGTH 21

SPIDRV_HandleData_t spi_handleData;
SPIDRV_Handle_t spi_handle = &spi_handleData;

uint8_t spi_tx_config_buffer[SPI_TX_CONFIG_BUF_LENGTH];
uint8_t spi_tx_buffer[SPI_TX_BUF_LENGTH];

/*******************************************************************************
 * @var spi_rx_buffer
 * @abstract Stores information received from the MAX board
 * @discussion The measurements and values from the MAX board are stored in the
 *             address locations:
 *             spi_rx_buffer[0:2]   Interrupt Status Register
 *             spi_rx_buffer[3:5]   TOF Int
 *             spi_rx_buffer[6:8]   TOF Frac
 *             spi_rx_buffer[9:11]  RTC Month_Year
 *             spi_rx_buffer[12:14] RTC Day_Date
 *             spi_rx_buffer[15:17] RTC Min_Hours
 *             spi_rx_buffer[18:20] RTC Seconds
 ******************************************************************************/
uint8_t spi_rx_buffer[SPI_RX_BUF_LENGTH];

// MAX SPI Transfer
#define MAX_SPI_TX_Config(x)    SPIDRV_MTransmitB(spi_handle, x, 3);
#define MAX_SPI_TX(x)           SPIDRV_MTransmitB(spi_handle, x, 1);
#define MAX_SPI_RX(x)           SPIDRV_MReceiveB(spi_handle, x, 1);
#define MAX_SPI_TXRX(x,y)       SPIDRV_MTransferB(spi_handle, x, y, 3);


/* ----- UART Declarations ----- */

DEFINE_BUF_QUEUE(EMDRV_UARTDRV_MAX_CONCURRENT_RX_BUFS, rxBufferQueue);
DEFINE_BUF_QUEUE(EMDRV_UARTDRV_MAX_CONCURRENT_TX_BUFS, txBufferQueue);

UARTDRV_HandleData_t uart_handleData;
UARTDRV_Handle_t uart_handle = &uart_handleData;

#ifdef SAVE_HEX_DATA

    #define UART_TX_BUF_LENGTH 12
    /*******************************************************************************
     * @var uart_tx_buffer
     * @abstract Stores information received from the MAX board
     * @discussion The measurements and values from the MAX board are stored in the
     *             address locations:
     *             uart_rx_buffer[0]  TOF Int
     *             uart_rx_buffer[1]  Delimiter
     *             uart_rx_buffer[2]  TOF Frac
     *             uart_rx_buffer[3]  Delimiter
     *             uart_rx_buffer[4]  RTC Month_Year
     *             uart_rx_buffer[5]  Delimiter
     *             uart_rx_buffer[6]  RTC Day_Date
     *             uart_rx_buffer[7]  Delimiter
     *             uart_rx_buffer[8]  RTC Min_Hours
     *             uart_rx_buffer[9]  Delimiter
     *             uart_rx_buffer[10] RTC Seconds
     *             uart_rx_buffer[11] Delimiter
     ******************************************************************************/
     uint32_t uart_tx_buffer[UART_TX_BUF_LENGTH];
     uint32_t delimiter = 0x00000020;

#else

    #define UART_TX_BUF_LENGTH 40
    #define TOF_DATA_LEN 12
    /*******************************************************************************
     * @var uart_tx_buffer
     * @abstract Stores information received from the MAX board
     * @discussion The measurements and values from the MAX board are stored in the
     *             address locations:
     *             uart_tx_buffer[0]     10 Month
     *             uart_tx_buffer[1]     Month
     *             uart_tx_buffer[2]     '/'
     *             uart_tx_buffer[3]     10 Date
     *             uart_tx_buffer[4]     Date
     *             uart_tx_buffer[5]     '/'
     *             uart_tx_buffer[6]     10 Year
     *             uart_tx_buffer[7]     Year
     *             uart_tx_buffer[8]     '\t'
     *             uart_tx_buffer[9]     10 Hour
     *             uart_tx_buffer[10]    Hour
     *             uart_tx_buffer[11]    ':'
     *             uart_tx_buffer[12]    10 Minute
     *             uart_tx_buffer[13]    Minute
     *             uart_tx_buffer[14]    ':'
     *             uart_tx_buffer[15]    10 Seconds
     *             uart_tx_buffer[16]    Seconds
     *             uart_tx_buffer[17]    ':'max
     *             uart_tx_buffer[18]    Tenths of Seconds
     *             uart_tx_buffer[19]    Hundredths of Seconds
     *             uart_tx_buffer[20]    '\t'
     *             uart_tx_buffer[21:39] TOF Diff
     ******************************************************************************/
    uint8_t uart_tx_buffer[UART_TX_BUF_LENGTH];
#endif

/* ----- RTC Declarations ----- */
RTCDRV_TimerID_t rtc_id;

void pollRTC();
void pollTOF();
void processRTC_HEX();
void processRTC_ASCII();
void processTOF_HEX();
void processTOF_ASCII();

/*******************************************************************************
 * @function    MAX_Init()
 * @abstract    Initialize MAX35103 settings
 * @discussion  Set up TOF1/2/3/4/5/6/7 registers and initialize MAX board
 *
 * @return      void
 ******************************************************************************/
void MAX_Init()
{
	//spi_tx_buffer[0] = RESET;            // Initialize
	//MAX_SPI_TX(&spi_tx_buffer[0]);

    // ----- Begin configuration register setup ----- */

	// we might not need set alarm, but we do it for the same with MAX board configuration
	// for now, we set alarm time and RTC time is 0
    spi_tx_config_buffer[0] = WRITE_RTC_SECS;
    spi_tx_config_buffer[1] = 0x28;
    spi_tx_config_buffer[2] = 0x45;
    MAX_SPI_TX_Config(&spi_tx_config_buffer[0]);

    spi_tx_config_buffer[0] = WRITE_RTC_MIN_HRS;
    spi_tx_config_buffer[1] = 0x48;
    spi_tx_config_buffer[2] = 0x14;
    MAX_SPI_TX_Config(&spi_tx_config_buffer[0]);

    spi_tx_config_buffer[0] = WRITE_RTC_DAY_DATE;
    spi_tx_config_buffer[1] = 0x03;
    spi_tx_config_buffer[2] = 0x25;
    MAX_SPI_TX_Config(&spi_tx_config_buffer[0]);

    spi_tx_config_buffer[0] = WRITE_RTC_M_Y;
    spi_tx_config_buffer[1] = 0x09;
    spi_tx_config_buffer[2] = 0x18;
    MAX_SPI_TX_Config(&spi_tx_config_buffer[0]);

    spi_tx_config_buffer[0] = WRITE_WD_ALARM_CNT;
    spi_tx_config_buffer[1] = 0x00;
    spi_tx_config_buffer[2] = 0x00;
    MAX_SPI_TX_Config(&spi_tx_config_buffer[0]);

    spi_tx_config_buffer[0] = WRITE_ALARM;
    spi_tx_config_buffer[1] = 0x00;
    spi_tx_config_buffer[2] = 0x00;
    MAX_SPI_TX_Config(&spi_tx_config_buffer[0]);

    /***************************************************************************
     * TOF1 Register - basic operating parameters for TOF measurements
     * TOF1[15:8]   Pulse Launcher Size
     * TOF1[7:4]    Pulse Launch Divider
     * TOF1[3]      Stop Polarity
     * TOF1[2]      Reserved (No effect)
     * TOF1[1:0]    Bias Charge Time
     **************************************************************************/
    spi_tx_config_buffer[0] = WRITE_TOF1;
    spi_tx_config_buffer[1] = 0x0C;
    spi_tx_config_buffer[2] = 0x10;
    MAX_SPI_TX_Config(&spi_tx_config_buffer[0]);

    /***************************************************************************
     * TOF2 Register - details of how TOF will be measured
     * TOF2[15:13]  Stop Hits
     * TOF2[12:7]   T2 Wave Selection
     * TOF2[6:4]    TOF Duty Cycle
     * TOF2[3]      Reserved (No effect)
     * TOF2[2:0]    Timeout
     **************************************************************************/
    spi_tx_config_buffer[0] = WRITE_TOF2;
    spi_tx_config_buffer[1] = 0xA1;
    spi_tx_config_buffer[2] = 0x00;
    MAX_SPI_TX_Config(&spi_tx_config_buffer[0]);

    /***************************************************************************
     * TOF3 Register - select which waves will be used in time measurements
     * TOF3[15:14]  Reserved
     * TOF3[13:8]   HIT1 Wave Select
     * TOF3[7:6]    Reserved
     * TOF3[5:0]    HIT2 Wave Select
     **************************************************************************/
    spi_tx_config_buffer[0] = WRITE_TOF3;
    spi_tx_config_buffer[1] = 0x05;
    spi_tx_config_buffer[2] = 0x06;
    MAX_SPI_TX_Config(&spi_tx_config_buffer[0]);

    /***************************************************************************
     * TOF4 Register - select which waves will be used in time measurements
     * TOF4[15:14]  Reserved
     * TOF4[13:8]   HIT3 Wave Select
     * TOF4[7:6]    Reserved
     * TOF4[5:0]    HIT4 Wave Select
     **************************************************************************/
    spi_tx_config_buffer[0] = WRITE_TOF4;
    spi_tx_config_buffer[1] = 0x07;
    spi_tx_config_buffer[2] = 0x08;
    MAX_SPI_TX_Config(&spi_tx_config_buffer[0]);

    /***************************************************************************
     * TOF5 Register - select which waves will be used in time measurements
     * TOF5[15:14]  Reserved
     * TOF5[13:8]   HIT5 Wave Select
     * TOF5[7:6]    Reserved
     * TOF5[5:0]    HIT6 Wave Select
     **************************************************************************/
    spi_tx_config_buffer[0] = WRITE_TOF5;
    spi_tx_config_buffer[1] = 0x09;
    spi_tx_config_buffer[2] = 0x0A;
    MAX_SPI_TX_Config(&spi_tx_config_buffer[0]);

    /***************************************************************************
     * TOF6 Register - comparator upstream
     * TOF6[15:8]   Comparator Return Offset Upstream
     * TOF6[7:0]    Comparator Offset Upstream
     **************************************************************************/
    spi_tx_config_buffer[0] = WRITE_TOF6;
    spi_tx_config_buffer[1] = 0x23;
    spi_tx_config_buffer[2] = 0x0A;
    MAX_SPI_TX_Config(&spi_tx_config_buffer[0]);

    /***************************************************************************
     * TOF7 Register - comparator downstream
     * TOF7[15:8]   Comparator Return Offset Downstream
     * TOF7[7:0]    Comparator Offset Downstream
     **************************************************************************/
    spi_tx_config_buffer[0] = WRITE_TOF7;
    spi_tx_config_buffer[1] = 0x23;
    spi_tx_config_buffer[2] = 0x0A;
    MAX_SPI_TX_Config(&spi_tx_config_buffer[0]);

    /***************************************************************************
     * TOF Measurement Delay - delay between start of pulse launch and receiver
     *                         enable
     * DLY[15:8]    Delay
     **************************************************************************/
    spi_tx_config_buffer[0] = WRITE_TOF_MEAS_DELAY;
    spi_tx_config_buffer[1] = 0x00;
    spi_tx_config_buffer[2] = 0xC8;
    MAX_SPI_TX_Config(&spi_tx_config_buffer[0]);

    /***************************************************************************
     * Calibration and Control Register - calibration settings
     * CLBRT[15:11] Temperature Measurement Cycles
     * CLBRT[10]    Calibration Usage
     * CLBRT[9:7]   Calibration Configuration
     * CLBRT[6:5]   Temperature Port
     * CLBRT[4:2]   Preamble Temperature Cycle
     * CLBRT[1:0]   Port Cycle Time
     **************************************************************************/
    spi_tx_config_buffer[0] = WRITE_CLBRT_CTRL;
    spi_tx_config_buffer[1] = 0x0F;
    spi_tx_config_buffer[2] = 0xDF;
    MAX_SPI_TX_Config(&spi_tx_config_buffer[0]);


    /* ----- End configuration register setup ----- */

    spi_tx_buffer[0] = TX_CONFIG_FLASH;       // Transfer Configuration to Flash Command
    MAX_SPI_TX(&spi_tx_buffer[0]);

    spi_tx_buffer[0] = INITIALIZE;            // Initialize
    MAX_SPI_TX(&spi_tx_buffer[0]);
}

/*******************************************************************************
 * @function    SPI_Init()
 * @abstract    Set up SPI
 * @discussion  SPI transfer between Wonder Gecko and MAX35103
 *              USART0 Location 0 - pins PE10 (TX), PE11 (RX), PE12(CLK), PE13(CE)
 * @return      void
 ******************************************************************************/
void SPI_Init() {
    SPIDRV_Init_t initData = {                                                        \
    		  USART0,                       /* USART port                       */    \
    		  _USART_ROUTELOC0_TXLOC_LOC0,  /* USART Tx pin location number     */    \
    		  _USART_ROUTELOC0_RXLOC_LOC0,  /* USART Rx pin location number     */    \
    		  _USART_ROUTELOC0_CLKLOC_LOC0, /* USART Clk pin location number    */    \
    		  _USART_ROUTELOC0_CSLOC_LOC0,  /* USART Cs pin location number     */    \
    		  1000000,                      /* Bitrate                          */    \
    		  8,                            /* Frame length                     */    \
    		  0,                            /* Dummy tx value for rx only funcs */    \
    		  spidrvMaster,                 /* SPI mode                         */    \
    		  spidrvBitOrderMsbFirst,       /* Bit order on bus                 */    \
    		  spidrvClockMode1,             /* SPI clock/phase mode             */    \
    		  spidrvCsControlAuto,          /* CS controlled by the driver      */    \
    		  spidrvSlaveStartImmediate     /* Slave start transfers immediately*/    \
    };

    // Initialize a SPI driver instance
    SPIDRV_Init( spi_handle, &initData );
}


/*******************************************************************************
 * @function    UART_Init()
 * @abstract    Set up USART
 * @discussion  USART transfer between Wonder Gecko and computer/other devices
 *              USART4 Location 2 - pins PI0 (TX), PI1 (RX)
 *
 * @return      void
 ******************************************************************************/
void UART_Init() {
	UARTDRV_InitUart_t uartInitData =   {                  \
		    USART4,                                        \
		    115200,                                        \
		    _USART_ROUTELOC0_TXLOC_LOC2,                   \
		    _USART_ROUTELOC0_RXLOC_LOC2,                   \
		    usartStopbits1,                                \
		    usartNoParity,                                 \
		    usartOVS16,                                    \
		    false,                                         \
		    uartdrvFlowControlNone,                        \
		    gpioPortA,                                     \
		    4,                                             \
		    gpioPortA,                                     \
		    5,                                             \
		    (UARTDRV_Buffer_FifoQueue_t *)&rxBufferQueue,  \
		    (UARTDRV_Buffer_FifoQueue_t *)&txBufferQueue,  \
		    _USART_ROUTELOC1_CTSLOC_LOC2,                  \
		    _USART_ROUTELOC1_RTSLOC_LOC2                   \
		  };

	UARTDRV_InitUart(uart_handle, &uartInitData);
}

// Function required for non-blocking transmit
void callback_UARTTX(UARTDRV_Handle_t handle,
                           Ecode_t transferStatus,
                           uint8_t *data,
                           UARTDRV_Count_t transferCount)
{
  (void)handle;
  (void)transferStatus;
  (void)data;
  (void)transferCount;
}

// Function required for non-blocking receive
void callback_UARTRX(UARTDRV_Handle_t handle,
                           Ecode_t transferStatus,
                           uint8_t *data,
                           UARTDRV_Count_t transferCount)
{
    (void)handle;
    (void)transferStatus;
    (void)data;
    (void)transferCount;
}


void callback_RTC( RTCDRV_TimerID_t id, void * user )
{
    (void) user; // unused argument

    // TOF Interrupt (bit 12)
    if((0x10 & spi_rx_buffer[1]) == 0x10) {

        // Read data from MAX board registers
        pollRTC();
        pollTOF();

        #ifdef SAVE_HEX_DATA
            // Convert data into hex format
            processRTC_HEX();
            processTOF_HEX();
            UARTDRV_Transmit(uart_handle, uart_tx_buffer, sizeof(uint32_t) * UART_TX_BUF_LENGTH, callback_UARTTX);

        #else
            // Convert data into ASCII format
            processRTC_ASCII();
            processTOF_ASCII();
            UARTDRV_Transmit(uart_handle, uart_tx_buffer, sizeof(uint8_t) * strlen(uart_tx_buffer), callback_UARTTX);

        #endif

        // Send each register individually
        // TODO: Include delimiter in uart_TX buffer and send all registers at once
        //       Potentially will fix bug where last two registers are not properly transmitted

                //for(int i = 0; i < UART_TX_BUF_LENGTH; i++) {
        //    UARTDRV_Transmit(uart_handle, &uart_tx_buffer[i], sizeof(uint32_t), callback_UARTTX);
        //    UARTDRV_Transmit(uart_handle, &delimiter, sizeof(uint8_t), callback_UARTTX);
        //}

        // Reset interrupt
        spi_rx_buffer[1] = 0x00;
        GPIO_IntEnable(0x0010);

        // Make new TOF measurement
        spi_tx_buffer[0] = TOF_DIFF;
        MAX_SPI_TX(&spi_tx_buffer[0]);

    }
    else {
        spi_tx_buffer[0] = READ_INT_STAT_REG;
        MAX_SPI_TXRX(&spi_tx_buffer[0], &spi_rx_buffer[0]);
    }

}

void GPIOINT_callback(void) {
    // TODO Multiple interrupts on EVEN_IRQHandler

    GPIO_IntDisable(0x0010);

    spi_tx_buffer[0] = READ_INT_STAT_REG;
    MAX_SPI_TXRX(&spi_tx_buffer[0], &spi_rx_buffer[0]);      // Read status register

    GPIO_IntClear(0x0010);
}


/*******************************************************************************
 * @function    setupGPIOInt()
 * @abstract    Enable GPIO Interrupts
 * @discussion
 *
 * @return      void
 ******************************************************************************/
void setupGPIOInt() {

    GPIO_PinModeSet(gpioPortA, 8, gpioModeInput, 1);  // MAX Interrupt
    GPIO_ExtIntConfig(gpioPortA, 8, 8, true, true, true);

    GPIOINT_Init();
    GPIOINT_CallbackRegister(8, (GPIOINT_IrqCallbackPtr_t) GPIOINT_callback);

    GPIO_IntEnable(0x0010);
}


/*******************************************************************************
 * @function    main()
 * @abstract    Set up communication with MAX board, poll for measurements
 * @discussion  Initialize WonderGecko, SPIDRV, UART, MAX board, check interrupt
 *              status, start RTC Timer
 *
 * @return      void
 ******************************************************************************/
int main(void) {

    /* Chip errata */
    CHIP_Init();

    SPI_Init();
    UART_Init();
    MAX_Init();
    setupGPIOInt();

    // Initialization of RTCDRV driver
    RTCDRV_Init();
    // Reserve a timer
    Ecode_t max_timer = RTCDRV_AllocateTimer( &rtc_id );

    #ifdef SAVE_HEX_DATA
        uart_tx_buffer[1] = uart_tx_buffer[3] = uart_tx_buffer[5] = uart_tx_buffer[7] = uart_tx_buffer[9] = uart_tx_buffer[11] = delimiter;
    #else
        uart_tx_buffer[2] = uart_tx_buffer[5] = '/';
        uart_tx_buffer[8] = uart_tx_buffer[20] = 0x20;
        uart_tx_buffer[11] = uart_tx_buffer[14] = ':';
        uart_tx_buffer[17] = '.';
        //uart_tx_buffer[33] = 0x0D;
        //uart_tx_buffer[34] = 0x0A;
    #endif

    time_t rawtime;
    struct tm * timeinfo;

    time ( &rawtime );
    timeinfo = gmtime ( &rawtime );
    printf ( "Current local time and date: %s", asctime (timeinfo) );

    // Initial measurement
    spi_tx_buffer[0] = TOF_DIFF;
    MAX_SPI_TX(&spi_tx_buffer[0]);

    spi_tx_buffer[0] = READ_INT_STAT_REG;
    MAX_SPI_TXRX(&spi_tx_buffer[0], &spi_rx_buffer[0]);

    // Start a periodic timer with 1000 millisecond timeout
    RTCDRV_StartTimer( rtc_id, rtcdrvTimerTypePeriodic, 1, callback_RTC, NULL );

}




void pollRTC() {
	spi_tx_buffer[0] = READ_RTC_M_Y;
    MAX_SPI_TXRX(&spi_tx_buffer[0] , &spi_rx_buffer[9]);

	spi_tx_buffer[0] = READ_RTC_DAY_DATE;
    MAX_SPI_TXRX(&spi_tx_buffer[0] , &spi_rx_buffer[12]);

	spi_tx_buffer[0] = READ_RTC_MIN_HRS;
    MAX_SPI_TXRX(&spi_tx_buffer[0] , &spi_rx_buffer[15]);

	spi_tx_buffer[0] = READ_RTC_SECS;
    MAX_SPI_TXRX(&spi_tx_buffer[0] , &spi_rx_buffer[18]);
}

void pollTOF() {
	spi_tx_buffer[0] = TOF_DIFF_INT;
    MAX_SPI_TXRX(&spi_tx_buffer[0], &spi_rx_buffer[3]);

    spi_tx_buffer[0] = TOF_DIFF_FRAC;
    MAX_SPI_TXRX(&spi_tx_buffer[0], &spi_rx_buffer[6]);
}

void processRTC_HEX(){
    uart_tx_buffer[4] = int16_2hex(*((uint16_t*)&spi_rx_buffer[10]));
    uart_tx_buffer[6] = int16_2hex(*((uint16_t*)&spi_rx_buffer[13]));
    uart_tx_buffer[8] = int16_2hex(*((uint16_t*)&spi_rx_buffer[16]));
    uart_tx_buffer[10] = int16_2hex(*((uint16_t*)&spi_rx_buffer[19]));
}

void processTOF_HEX(){
    uart_tx_buffer[0] = int16_2hex(*((uint16_t*)&spi_rx_buffer[4]));
    uart_tx_buffer[2] = int16_2hex(*((uint16_t*)&spi_rx_buffer[7]));
}

void processRTC_ASCII(){
    // Bitwise operations to separate data in each register
    uart_tx_buffer[0] = ((spi_rx_buffer[10] & 0x10) >> 4) + 0x30;   // 10 Month
    uart_tx_buffer[1] = (spi_rx_buffer[10] & 0x0F) + 0x30;          // Month
    uart_tx_buffer[3] = ((spi_rx_buffer[14] & 0x30) >> 4) + 0x30;   // 10 Date
    uart_tx_buffer[4] = (spi_rx_buffer[14] & 0x0F) + 0x30;          // Date
    uart_tx_buffer[6] = ((spi_rx_buffer[11] & 0xF0) >> 4) + 0x30;   // 10 Year
    uart_tx_buffer[7] = (spi_rx_buffer[11] & 0x0F) + 0x30;          // Year
	uart_tx_buffer[9] = ((spi_rx_buffer[17] & 0x30) >> 4) + 0x30;   // 10 Hour (tens digit stays the same regardless 12/24 hr)
    if((spi_rx_buffer[17] & 0x40) == 0x40){                         // if 12 hour mode
    	uart_tx_buffer[10] = (spi_rx_buffer[17] & 0x0F) + 0x32;      // Hour (add 2)
    }
    else {                                                          // if 24 hour mode
    	uart_tx_buffer[10] = (spi_rx_buffer[17] & 0x0F) + 0x30;      // Hour
    }
    uart_tx_buffer[12] = ((spi_rx_buffer[16] & 0x70) >> 4) + 0x30;   // 10 Minute
    uart_tx_buffer[13] = (spi_rx_buffer[16] & 0x0F) + 0x30;          // Minute
    uart_tx_buffer[15] = ((spi_rx_buffer[20] & 0x70) >> 4) + 0x30;  // 10 Second
    uart_tx_buffer[16] = (spi_rx_buffer[20] & 0x0F) + 0x30;         // Second
    uart_tx_buffer[18] = ((spi_rx_buffer[19] & 0xF0) >> 4) + 0x30;  // Tenth of Second
    uart_tx_buffer[19] = (spi_rx_buffer[19] & 0x0F) + 0x30;         // Hundredth of Second
}

void processTOF_ASCII(){
	char tmp_buffer[TOF_DATA_LEN] = {0};

	int16_t tofDiffInt = ((spi_rx_buffer[4] & 0x00FF) << 8) | (spi_rx_buffer[5] & 0x00FF);
	uint16_t tofDiffFrac = ((spi_rx_buffer[7] & 0x00FF) << 8) | (spi_rx_buffer[8] & 0x00FF);

	float tofValue = (float) tofDiffInt;
	float tofValueFrac = ((float) tofDiffFrac) / (65536);
	tofValue += tofValueFrac;

	gcvt(tofValue, TOF_DATA_LEN, tmp_buffer);
	sprintf(&uart_tx_buffer[21], "%s", tmp_buffer);
}
