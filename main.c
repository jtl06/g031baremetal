//basic bare metal implementation of blinking an LED.

#include <inttypes.h>
#include <stdbool.h>

#define BIT(x) (1UL << (x)) //bit mask
#define PIN(bank, num) ((((bank) - 'A') << 8) | (num )) //pack GPIO port and pin into one value
#define PINNO(pin) (pin & 255) //extract pin from packed value
#define PINBANK(pin) (pin >> 8) //extract bank index from packed value

//GPIO register block
struct gpio {
    volatile uint32_t MODER;     // 0x00
    volatile uint32_t OTYPER;    // 0x04
    volatile uint32_t OSPEEDR;   // 0x08
    volatile uint32_t PUPDR;     // 0x0C
    volatile uint32_t IDR;       // 0x10
    volatile uint32_t ODR;       // 0x14
    volatile uint32_t BSRR;      // 0x18
    volatile uint32_t LCKR;      // 0x1C
    volatile uint32_t AFR[2];    // 0x20, 0x24 (ARFL, ARFH)
    volatile uint32_t BRR;       // 0x28
};
//GPIO base, g031 starts at 0x50000000, with 0x400 offset per bank
#define GPIO(bank_numeric_index) ((volatile struct gpio *)(uintptr_t)(0x50000000U + 0x400U * (uint32_t)(bank_numeric_index)))

//RCC block for G0
struct rcc {
    volatile uint32_t CR;            /* 0x00 */
    volatile uint32_t ICSCR;         /* 0x04 */
    volatile uint32_t CFGR;          /* 0x08 */
    volatile uint32_t PLLSYSCFGR;    /* 0x0C */
    volatile uint32_t RESERVED0;     /* 0x10 */
    volatile uint32_t CRRCR;         /* 0x14 */
    volatile uint32_t CIER;          /* 0x18 */
    volatile uint32_t CIFR;          /* 0x1C */
    volatile uint32_t CICR;          /* 0x20 */
    volatile uint32_t IOPRSTR;       /* 0x24 */
    volatile uint32_t AHBRSTR;       /* 0x28 */
    volatile uint32_t APBRSTR1;      /* 0x2C */
    volatile uint32_t APBRSTR2;      /* 0x30 */
    volatile uint32_t IOPENR;        /* 0x34 */
};
#define RCC ((struct rcc *)0x40021000)

//GPIO port mode register
enum { GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_AF, GPIO_MODE_ANALOG };

//sets GPIO mode: sets pin to mode
static inline void gpio_set_mode(uint16_t pin, uint8_t mode) 
{
    volatile struct gpio *gpio = GPIO(PINBANK(pin));  // GPIO bank
    uint32_t n = PINNO(pin);                      // Pin number
    gpio->MODER &= ~(3U << (n * 2));         // Clear existing setting
    gpio->MODER |= (mode & 3) << (n * 2);    // Set new mode
}

//writes to GPIO pin
static void gpio_write(uint16_t pin, bool v)
{
    volatile struct gpio *g = GPIO(PINBANK(pin));
    g->BSRR = BIT(PINNO(pin)) << (v ? 0 : 16);
}

//delay loop
static void delay(void) 
{
    for (volatile uint32_t i = 0; i < 800000; i++)
        __asm__ volatile ("nop");
}

//blinky
int main(void)
{
    uint16_t led = PIN('C', 6); //PC6 is LED pin

    RCC->IOPENR |= BIT(2); //enable GPIOC clock
    gpio_set_mode(led, GPIO_MODE_OUTPUT); //set PC6 to output

    for(;;) //run forever 
    {
        gpio_write(led, true); //toggle on
        delay(); //delay
        gpio_write(led, false); //toggle off
        delay(); //delay
    }
}

// Startup code
__attribute__((naked, noreturn)) void _reset(void) {
    extern long _sbss, _ebss, _sdata, _edata, _sidata;
    for (long *dst = &_sbss; dst < &_ebss; dst++) *dst = 0; 
    for (long *dst = &_sdata, *src = &_sidata; dst < &_edata;) *dst++ = *src++; 

    main(); //call main();
    for(;;) (void) 0; //loop in case main returns
}
  
extern char _estack;  // Defined in link.ld

void Default_Handler(void) { for(;;); } //dummy handler
  
// 16 standard and 31 STM32-specific handlers
__attribute__((section(".vectors")))
void (*const tab[16 + 31])(void) = {
    (void (*)(void)) &_estack,  // initial SP
    _reset,                     // reset
    Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    Default_Handler,            /* 10 */
    Default_Handler,            /* 11 SVC */
    Default_Handler, Default_Handler,
    Default_Handler,            /* 14 PendSV */
    Default_Handler,            /* 15 SysTick */
    [16 ... 46] = Default_Handler      /* 31 peripheral IRQ stubs */
};