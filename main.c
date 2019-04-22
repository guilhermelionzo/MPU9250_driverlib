/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdint.h>

#include <stdbool.h>
//#include <//printf.h>
#include <time.h>
#include <MPU9250.h>
#define RED     255
#define GREEN   255
#define BLUE    255

#define MCLK_FREQUENCY 12000000
#define PWM_PERIOD (MCLK_FREQUENCY/5000)

#define SLAVE_ADDRESS             0x68
#define SLAVE_ADDRESS_CAM         0x42
#define NUM_OF_REC_BYTES    14

#define ARRAY_LENGTH( x ) ( sizeof( x )/ ( x[0] ) )

#define MPU6050_PWR1_DEVICE_RESET_BIT   7
#define MPU6050_PWR1_SLEEP_BIT          6
#define MPU6050_PWR1_CYCLE_BIT          5
#define MPU6050_PWR1_GYRO_STANDBY_BIT   4
#define MPU6050_PWR1_TEMP_DIS_BIT       3
#define MPU6050_PWR1_CLKSEL_BIT         2
#define MPU6050_PWR1_CLKSEL_LENGTH      3

#define PWR_MGMT_1_SLEEP

/* UART Configuration Parameter. These are the configuration parameters to
 * make the eUSCI A UART module to operate with a 9600 baud rate. These
 * values were calculated using the online calculator that TI provides
 * at:
 *software-dl.ti.com/.../index.html
 */

/* Statics  for I2C working with the MPU6050 sensor*/
int i = 0;
uint8_t TXData[ 16 ] = { 0x6B, 0x00, 0x38, 0x40, 0x68, 0xC0, 0x6B, 0x16, 0x41,
                       0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48 };

static uint8_t RXData[NUM_OF_REC_BYTES];
uint8_t countRxData = 0;

static volatile bool stopSent;
static uint8_t TXByteCtr;

void EUSCIA0_IRQHandler(void);

const uint8_t port_mapping[] = {
        //Port P2:
        PM_TA0CCR1A,
        PM_TA0CCR2A, PM_TA0CCR3A, PM_NONE, PM_TA1CCR1A, PM_NONE,
        PM_NONE, PM_NONE };

const eUSCI_UART_Config uartConfig = {
EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
        78,                                     // BRDIV = 78
        2,                                       // UCxBRF = 2
        0,                                       // UCxBRS = 0
        EUSCI_A_UART_NO_PARITY,                  // No Parity
        EUSCI_A_UART_LSB_FIRST,                  // MSB First
        EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
        EUSCI_A_UART_MODE,                       // UART mode
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION  // Oversampling
        };

/* I2C Master Configuration Parameter */
const eUSCI_I2C_MasterConfig _i2cConfig = {
EUSCI_B_I2C_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
        3000000,                                // SMCLK = 3MHz
        EUSCI_B_I2C_SET_DATA_RATE_100KBPS,      // Desired I2C Clock of 100khz
        0,                                      // No byte counter threshold
        EUSCI_B_I2C_NO_AUTO_STOP                // No Autostop
        };

//void mpuInit(void){}

void ov7670_init(void)
{

}

int main(void)
{

    int countTxData;

    /* Halting WDT and disabling master interrupts */
    MAP_WDT_A_holdTimer();
    //MAP_Interrupt_disableMaster();

    /* Setting DCO to 12MHz */
    //CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);

    // Set P1.0 to output direction
    GPIO_setAsOutputPin(GPIO_PORT_P1,
                        GPIO_PIN0);

    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);

    /* Selecting P1.2 and P1.3 in UART mode and P1.0 as output (LED)
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(
            GPIO_PORT_P1,
            GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3,
            GPIO_PRIMARY_MODULE_FUNCTION);*/

    /* Selecting P3.2 and P3.3 in UART mode
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(
            GPIO_PORT_P3,
            GPIO_PIN2 | GPIO_PIN3,
            GPIO_PRIMARY_MODULE_FUNCTION);*/

    /* Select Port 1 for I2C - Set Pin 6, 7 to Secondary Module Function,
     *   (UCB3SIMO/UCB3SDA, UCB3SOMI/UCB3SCL).

    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(
            GPIO_PORT_P1,
            GPIO_PIN6 + GPIO_PIN7,
            GPIO_PRIMARY_MODULE_FUNCTION);*/

    /* Initialize main clock to 12MHz */
    MAP_CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);
    MAP_CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    MAP_CS_initClockSignal(CS_HSMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    MAP_CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    /* Configuring UART Module */
    //MAP_UART_initModule(EUSCI_A0_BASE, &uartConfig);

    //MAP_UART_initModule(EUSCI_A2_BASE, &uartConfig);

    /* Enable UART module */
    //MAP_UART_enableModule(EUSCI_A0_BASE);

    //MAP_UART_enableModule(EUSCI_A2_BASE);

    //UART_registerInterrupt(EUSCI_A0_BASE, EUSCIA0_IRQHandler);

    /* GPIO Setup for Pins 2.0-2.2 */
    //MAP_PMAP_configurePorts((const uint8_t *) port_mapping, PMAP_P2MAP, 1,
    //PMAP_DISABLE_RECONFIGURATION);

    /*MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(
            GPIO_PORT_P2,
            GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2,
            GPIO_PRIMARY_MODULE_FUNCTION);*/

    /* Confinguring P1.1 & P1.4 as an input and enabling interrupts
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1,
                                             GPIO_PIN1 | GPIO_PIN4);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN4);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN4);
    MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN4,
                                 GPIO_HIGH_TO_LOW_TRANSITION);*/

    /* Configure TimerA0 without Driverlib (CMSIS style register access)
    TIMER_A0->CCR[0] = PWM_PERIOD;
    TIMER_A0->CCTL[1] = TIMER_A_CCTLN_OUTMOD_7;                // CCR1 reset/set
    TIMER_A0->CCR[1] = PWM_PERIOD * (RED / 255);         // CCR1 PWM duty cycle
    TIMER_A0->CCTL[2] = TIMER_A_CCTLN_OUTMOD_7;                // CCR2 reset/set
    TIMER_A0->CCR[2] = PWM_PERIOD * (0 / 255);           // CCR2 PWM duty cycle
    TIMER_A0->CCTL[3] = TIMER_A_CCTLN_OUTMOD_7;                // CCR3 reset/set
    TIMER_A0->CCR[3] = PWM_PERIOD * (0 / 255);           // CCR3 PWM duty cycle
    TIMER_A0->CTL = TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_MC__UP
            | TIMER_A_CTL_CLR;  // SMCLK, up mode, clear TAR
*/
    /* Configure and enable SysTick
    MAP_SysTick_setPeriod(1500000);
    MAP_SysTick_enableModule();
    MAP_SysTick_enableInterrupt();

    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN4);
    MAP_Interrupt_enableInterrupt(INT_PORT1);*/

    /* Enabling interrupts
    MAP_UART_enableInterrupt(
            EUSCI_A0_BASE,
            EUSCI_A_UART_RECEIVE_INTERRUPT
                    | EUSCI_A_UART_RECEIVE_ERRONEOUSCHAR_INTERRUPT
                    | EUSCI_A_UART_BREAKCHAR_INTERRUPT);

    MAP_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    //MAP_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_TRANSMIT_INTERRUPT);
    MAP_UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);


    //enable interrupts for each module
    //........MAP_Interrupt_enableInterrupt(INT_EUSCIA0);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA2);
   // MAP_Interrupt_enableInterrupt(INT_EUSCIB0);*/

    //--------------------  I2C

    int16_t ax, ay, az;
    int16_t gx, gy, gz;


    MPU9250_initialize();

    while(1){
        MPU9250_getMotion9(&ax, &ay, &az, &gx, &gy, &gz);

    }

}

/* EUSCI A0 UART ISR - Echoes data back to PC host */
void EUSCIA0_IRQHandler(void)
{
    //char receiveByte = UCA0RXBUF;
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A0_BASE);

    MAP_UART_clearInterruptFlag(EUSCI_A0_BASE, status);

    if (EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        MAP_UART_transmitData(EUSCI_A0_BASE, MAP_UART_receiveData(EUSCI_A0_BASE));
    }

}

/* EUSCI A2 UART ISR - Echoes data back to PC host */
void EUSCIA2_IRQHandler(void)
{
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A2_BASE);

    MAP_UART_clearInterruptFlag(EUSCI_A2_BASE, status);

    if (status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        //printf(EUSCI_A0_BASE, "Receive2\n\r");
    }

    if (status & EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG)
    {
        //printf(EUSCI_A0_BASE, "Transmit2 \n\r");
    }

}

/*
 * Port 1 interrupt handler. This handler is called whenever switches attached
 * to P1.1 (S1) and P1.4 (S2) are pressed.
 */
void PORT1_IRQHandler(void)
{
    //newTick = MAP_SysTick_getValue();
    uint32_t status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P1);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, status);

    /* Handles S1 button press */
    if (status & GPIO_PIN1)
    {
        //MAP_Interrupt_disableInterrupt(INT_PORT1);
        //printf(EUSCI_A0_BASE, "LED should light up \n\r");
        TIMER_A0->CCR[1] = PWM_PERIOD * (0 / 255);        // CCR1 PWM duty cycle
        TIMER_A0->CCR[2] = PWM_PERIOD * (GREEN / 255);    // CCR2 PWM duty cycle
        TIMER_A0->CCR[3] = PWM_PERIOD * (0 / 255);        // CCR3 PWM duty cycle
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
        //printf(EUSCI_A0_BASE, "LED lit?");
    }
    /* Handles S2 button press */
    if (status & GPIO_PIN4)
    {

        /* Initialize values to display */
        char *s = "//printf test";
        char c = '!';
        int i = -12345;
        unsigned u = 4321;
        long int l = -123456780;
        long unsigned n = 1098765432;
        unsigned x = 0xABCD;

        //printf(EUSCI_A0_BASE, "String         %s\r\n", s);
        //printf(EUSCI_A0_BASE, "Char           %c\r\n", c);
        //printf(EUSCI_A0_BASE, "Integer        %i\r\n", i);
        //printf(EUSCI_A0_BASE, "Unsigned       %u\r\n", u);
        //printf(EUSCI_A0_BASE, "Long           %l\r\n", l);
        //printf(EUSCI_A0_BASE, "uNsigned loNg  %n\r\n", n);
        //printf(EUSCI_A0_BASE, "heX            %x\r\n", x);
    }

}

/*
 * SysTick interrupt handler. This handler toggles RGB LED on/off.
 */
void SysTick_Handler(void)
{
    if (!(TIMER_A0->CCTL[1] & TIMER_A_CCTLN_OUTMOD_0))
    {
        TIMER_A0->CCTL[1] = TIMER_A_CCTLN_OUTMOD_7;
        TIMER_A0->CCTL[2] = TIMER_A_CCTLN_OUTMOD_7;
        TIMER_A0->CCTL[3] = TIMER_A_CCTLN_OUTMOD_7;
    }

}
