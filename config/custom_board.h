#ifndef __CUSTOM_BOARD_H__
#define __CUSTOM_BOARD_H__

#define BUTTON         24
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

#define LED_R          13
#define LED_G          12
#define LED_B          11

#define NMOS_1          6
#define NMOS_2          7
#define NMOS_3          8
#define NMOS_4          9

// Low frequency clock source to be used by the SoftDevice
#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_XTAL,            \
                                 .rc_ctiv       = 0,                                \
                                 .rc_temp_ctiv  = 0,                                \
                                 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}

#endif
