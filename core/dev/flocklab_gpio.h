#ifndef __FLOCKLAB_GPIO_H__
#define __FLOCKLAB_GPIO_H__

#define PIN0    0
#define PIN1    1
#define PIN2    2
#define PIN3    3
#define PIN4    4
#define PIN5    5
#define PIN6    6
#define PIN7    7

#define PORT1   1
#define PORT2   2
#define PORT3   3
#define PORT4   4
#define PORT5   5
#define PORT6   6

#define PIN_SET_I(port, pin)            (P##port##OUT |= BIT##pin)
#define PIN_CLR_I(port, pin)            (P##port##OUT &= ~BIT##pin)
#define PIN_XOR_I(port, pin)            (P##port##OUT ^= BIT##pin)
#define PIN_SEL_I(port, pin)            (P##port##SEL |= BIT##pin)
#define PIN_UNSEL_I(port, pin)          (P##port##SEL &= ~BIT##pin)
#define PIN_CFG_OUT_I(port, pin)        (P##port##DIR |= BIT##pin)
#define PIN_CFG_IN_I(port, pin)         (P##port##DIR &= ~BIT##pin)
#define PIN_GET_I(port, pin)            ((P##port##IN & BIT##pin) > 0)

/**
 * @brief get a (port, pin) input
 */
#define PIN_GET(p)                      PIN_GET_I(p)
/**
 * @brief set a (port, pin) output, i.e. push it's output signal high
 */
#define PIN_SET(p)                      PIN_SET_I(p)
/**
 * @brief toggle output state of a pin
 */
#define PIN_XOR(p)                      PIN_XOR_I(p)
/**
 * @brief clear a (port, pin) output, i.e. pull it's output signal low
 */
#define PIN_CLR(p)                      PIN_CLR_I(p)
/**
 * @brief select a (port, pin), i.e. configure it in module function mode
 */
#define PIN_SEL(p)                      PIN_SEL_I(p)
/**
 * @brief unselect a (port, pin), i.e. configure it in port mode (GPIO)
 */
#define PIN_UNSEL(p)                    PIN_UNSEL_I(p)
/**
 * @brief set a (port, pin) as output
 * @note don't forget to unselect a pin if you want to use it in GPIO mode
 */
#define PIN_CFG_OUT(p)                  PIN_CFG_OUT_I(p)
/**
 * @brief set a (port, pin) as input
 */
#define PIN_CFG_IN(p)                   PIN_CFG_IN_I(p)

/* the following pins assignments are given by FlockLAB, do not change */
#define FLOCKLAB_LED1               PORT6, PIN7  /* for GPIO tracing */
#define FLOCKLAB_LED2               PORT6, PIN6  /* for GPIO tracing */
#define FLOCKLAB_LED3               PORT6, PIN2  /* for GPIO tracing */
#define FLOCKLAB_SIG1               PORT2, PIN7  /* target actuation */
#define FLOCKLAB_SIG2               PORT2, PIN3  /* target actuation */
#define FLOCKLAB_INT1               PORT6, PIN0  /* for GPIO tracing */
#define FLOCKLAB_INT2               PORT6, PIN1  /* for GPIO tracing */

#endif // __FLOCKLAB_GPIO_H__