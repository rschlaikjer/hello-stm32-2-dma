#include <errno.h>
#include <stdio.h>
#include <unistd.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/cm3/systick.h>

extern "C" {
    void sys_tick_handler(void);
    int _write(int file, const char *ptr, ssize_t len);
}

static void clock_setup() {
    // First, let's ensure that our clock is running off the high-speed internal
    // oscillator (HSI) at 48MHz.
    rcc_clock_setup_in_hsi_out_48mhz();

    // Since our LED is on GPIO bank A, we need to enable
    // the peripheral clock to this GPIO bank in order to use it.
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);

    // In order to use our UART, we must enable the clock to it as well.
    rcc_periph_clock_enable(RCC_USART1);
}

static void usart_setup() {
    // For the peripheral to work, we need to enable it's clock
    rcc_periph_clock_enable(RCC_USART1);
    // From the datasheet for the STM32F0 series of chips (Page 30, Table 11)
    // we know that the USART1 peripheral has it's TX line connected as
    // alternate function 1 on port A pin 9.
    // In order to use this pin for the alternate function, we need to set the
    // mode to GPIO_MODE_AF (alternate function). We also do not need a pullup
    // or pulldown resistor on this pin, since the peripheral will handle
    // keeping the line high when nothing is being transmitted.
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6);
    // Now that we have put the pin into alternate function mode, we need to
    // select which alternate function to use. PA9 can be used for several
    // alternate functions - Timer 15, USART1 TX, Timer 1, and on some devices
    // I2C. Here, we want alternate function 1 (USART1_TX)
    gpio_set_af(GPIOB, GPIO_AF0, GPIO6);
    // Now that the pins are configured, we can configure the USART itself.
    // First, let's set the baud rate at 115200
    usart_set_baudrate(USART1, 115200);
    // Each datum is 8 bits
    usart_set_databits(USART1, 8);
    // No parity bit
    usart_set_parity(USART1, USART_PARITY_NONE);
    // One stop bit
    usart_set_stopbits(USART1, USART_CR2_STOPBITS_1);
    // For a debug console, we only need unidirectional transmit
    usart_set_mode(USART1, USART_MODE_TX);
    // No flow control
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    // Enable the peripheral
    usart_enable(USART1);

    // Optional extra - disable buffering on stdout.
    // Buffering doesn't save us any syscall overhead on embedded, and
    // can be the source of what seem like bugs.
    setbuf(stdout, NULL);
}

void spi_setup() {
    // Enable clock for SPI2 peripheral
    rcc_periph_clock_enable(RCC_SPI2);

    // Configure GPIOB, AF0: SCK = PB13, MISO = PB14, MOSI = PB15
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO13 | GPIO14 | GPIO15);
    gpio_set_af(GPIOB, GPIO_AF0, GPIO13 | GPIO14 | GPIO15);

    // We will be manually controlling the SS pin here, so set it as a normal output
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12);

    // SS is active low, so pull it high for now
    gpio_set(GPIOB, GPIO12);

    // Reset our peripheral
    spi_reset(SPI2);

    // Set main SPI settings:
    // - The datasheet for the 74HC595 specifies a max frequency at 4.5V of
    //   25MHz, but since we're running at 3.3V we'll instead use a 12MHz
    //   clock, or 1/4 of our main clock speed.
    // - Set the clock polarity to be zero at idle
    // - Set the clock phase to trigger on the rising edge, as per datasheet
    // - Send the most significant bit (MSB) first
    spi_init_master(
        SPI2,
        SPI_CR1_BAUDRATE_FPCLK_DIV_4,
        SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
        SPI_CR1_CPHA_CLK_TRANSITION_1,
        SPI_CR1_MSBFIRST
    );

    // Since we are manually managing the SS line, we need to move it to
    // software control here.
    spi_enable_software_slave_management(SPI2);

    // We also need to set the value of NSS high, so that our SPI peripheral
    // doesn't think it is itself in slave mode.
    spi_set_nss_high(SPI2);

    // The terminology around directionality can be a little confusing here -
    // unidirectional mode means that this is the only chip initiating
    // transfers, not that it will ignore any incoming data on the MISO pin.
    // Enabling duplex is required to read data back however.
    spi_set_unidirectional_mode(SPI2);

    // We're using 8 bit, not 16 bit, transfers
    spi_set_data_size(SPI2, SPI_CR2_DS_8BIT);

    // Enable the peripheral
    spi_enable(SPI2);
}
static void systick_setup() {
    // Set the systick clock source to our main clock
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    // Clear the Current Value Register so that we start at 0
    STK_CVR = 0;
    // In order to trigger an interrupt every millisecond, we can set the reload
    // value to be the speed of the processor / 1000 -1
    systick_set_reload(rcc_ahb_frequency / 1000 - 1);
    // Enable interrupts from the system tick clock
    systick_interrupt_enable();
    // Enable the system tick counter
    systick_counter_enable();
}

// Storage for our monotonic system clock.
// Note that it needs to be volatile since we're modifying it from an interrupt.
static volatile uint64_t _millis = 0;

uint64_t millis() {
    return _millis;
}

// This is our interrupt handler for the systick reload interrupt.
// The full list of interrupt services routines that can be implemented is
// listed in libopencm3/include/libopencm3/stm32/f0/nvic.h
void sys_tick_handler(void) {
    // Increment our monotonic clock
    _millis++;
}

/**
 * Delay for a real number of milliseconds
 */
void delay(uint64_t duration) {
    const uint64_t until = millis() + duration;
    while (millis() < until);
}

static void gpio_setup() {
    // Our test LED is connected to Port A pin 11, so let's set it as output
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO11);
}

int _write(int file, const char *ptr, ssize_t len) {
    // If the target file isn't stdout/stderr, then return an error
    // since we don't _actually_ support file handles
    if (file != STDOUT_FILENO && file != STDERR_FILENO) {
        // Set the errno code (requires errno.h)
        errno = EIO;
        return -1;
    }

    // Keep i defined outside the loop so we can return it
    ssize_t i;
    for (i = 0; i < len; i++) {
        // If we get a newline character, also be sure to send the carriage
        // return character first, otherwise the serial console may not
        // actually return to the left.
        if (ptr[i] == '\n') {
            usart_send_blocking(USART1, '\r');
        }

        // Write the character to send to the USART1 transmit buffer, and block
        // until it has been sent.
        usart_send_blocking(USART1, ptr[i]);
    }

    // Return the number of bytes we sent
    return i;
}

void spi_transfer(uint8_t tx_count, uint8_t *tx_data) {
    // Pull CS low to select target
    gpio_clear(GPIOB, GPIO12);

    for (uint8_t i = 0; i < tx_count; i++) {
        // Wait for the peripheral to become ready to transmit
        while (!(SPI_SR(SPI2) & SPI_SR_TXE));

        // Place the next data in the data register for transmission
        SPI_DR8(SPI2) = tx_data[i];
    }

    // Block return until the SPI periperhal has finished sending
    while (SPI_SR(SPI2) & SPI_SR_BSY);

    // Bring the SS pin high again
    gpio_set(GPIOB, GPIO12);
}

int main() {
    clock_setup();
    usart_setup();
    systick_setup();
    spi_setup();
    gpio_setup();

    // Toggle the LED on and off forever
    while (1) {
        printf("[%lld] LED on\n", millis());
        gpio_set(GPIOA, GPIO11);
        delay(1000);
        printf("[%lld] LED off\n", millis());
        gpio_clear(GPIOA, GPIO11);
        delay(1000);
    }

    return 0;
}
