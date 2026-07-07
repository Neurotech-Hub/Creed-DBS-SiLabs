#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

// $[CMU]
// [CMU]$

// $[LFXO]
// [LFXO]$

// $[PRS.ASYNCH0]
// [PRS.ASYNCH0]$

// $[PRS.ASYNCH1]
// [PRS.ASYNCH1]$

// $[PRS.ASYNCH2]
// [PRS.ASYNCH2]$

// $[PRS.ASYNCH3]
// [PRS.ASYNCH3]$

// $[PRS.ASYNCH4]
// [PRS.ASYNCH4]$

// $[PRS.ASYNCH5]
// [PRS.ASYNCH5]$

// $[PRS.ASYNCH6]
// [PRS.ASYNCH6]$

// $[PRS.ASYNCH7]
// [PRS.ASYNCH7]$

// $[PRS.ASYNCH8]
// [PRS.ASYNCH8]$

// $[PRS.ASYNCH9]
// [PRS.ASYNCH9]$

// $[PRS.ASYNCH10]
// [PRS.ASYNCH10]$

// $[PRS.ASYNCH11]
// [PRS.ASYNCH11]$

// $[PRS.SYNCH0]
// [PRS.SYNCH0]$

// $[PRS.SYNCH1]
// [PRS.SYNCH1]$

// $[PRS.SYNCH2]
// [PRS.SYNCH2]$

// $[PRS.SYNCH3]
// [PRS.SYNCH3]$

// $[GPIO]
// [GPIO]$

// $[TIMER0]
// [TIMER0]$

// $[TIMER1]
// [TIMER1]$

// $[TIMER2]
// [TIMER2]$

// $[TIMER3]
// [TIMER3]$

// $[TIMER4]
// [TIMER4]$

// $[USART0]
// USART0 CLK on PA08
#ifndef USART0_CLK_PORT                         
#define USART0_CLK_PORT                          gpioPortA
#endif
#ifndef USART0_CLK_PIN                          
#define USART0_CLK_PIN                           8
#endif

// USART0 CS on PA05
#ifndef USART0_CS_PORT                          
#define USART0_CS_PORT                           gpioPortA
#endif
#ifndef USART0_CS_PIN                           
#define USART0_CS_PIN                            5
#endif

// USART0 RX on PA06
#ifndef USART0_RX_PORT                          
#define USART0_RX_PORT                           gpioPortA
#endif
#ifndef USART0_RX_PIN                           
#define USART0_RX_PIN                            6
#endif

// USART0 TX on PA04
#ifndef USART0_TX_PORT                          
#define USART0_TX_PORT                           gpioPortA
#endif
#ifndef USART0_TX_PIN                           
#define USART0_TX_PIN                            4
#endif

// [USART0]$

// $[USART1]
// [USART1]$

// $[I2C1]
// [I2C1]$

// $[PDM]
// [PDM]$

// $[ETAMPDET]
// [ETAMPDET]$

// $[LETIMER0]
// [LETIMER0]$

// $[IADC0]
// IADC0 POS on PC02
#ifndef IADC0_POS_PORT                          
#define IADC0_POS_PORT                           gpioPortC
#endif
#ifndef IADC0_POS_PIN                           
#define IADC0_POS_PIN                            2
#endif

// [IADC0]$

// $[ACMP0]
// [ACMP0]$

// $[I2C0]
// [I2C0]$

// $[EUSART0]
// [EUSART0]$

// $[PTI]
// [PTI]$

// $[MODEM]
// [MODEM]$

// $[CUSTOM_PIN_NAME]
#ifndef _PORT                                   
#define _PORT                                    gpioPortA
#endif
#ifndef _PIN                                    
#define _PIN                                     0
#endif

#ifndef MAG_PORT                                
#define MAG_PORT                                 gpioPortA
#endif
#ifndef MAG_PIN                                 
#define MAG_PIN                                  7
#endif

#ifndef SHDN_PORT                               
#define SHDN_PORT                                gpioPortB
#endif
#ifndef SHDN_PIN                                
#define SHDN_PIN                                 0
#endif

#ifndef LED0_PORT                               
#define LED0_PORT                                gpioPortB
#endif
#ifndef LED0_PIN                                
#define LED0_PIN                                 1
#endif

#ifndef ELEC0_OUT_PORT                          
#define ELEC0_OUT_PORT                           gpioPortB
#endif
#ifndef ELEC0_OUT_PIN                           
#define ELEC0_OUT_PIN                            2
#endif

#ifndef ELEC1_OUT_PORT                          
#define ELEC1_OUT_PORT                           gpioPortB
#endif
#ifndef ELEC1_OUT_PIN                           
#define ELEC1_OUT_PIN                            4
#endif

#ifndef VBATT_PORT                              
#define VBATT_PORT                               gpioPortC
#endif
#ifndef VBATT_PIN                               
#define VBATT_PIN                                2
#endif

// [CUSTOM_PIN_NAME]$

#endif // PIN_CONFIG_H

// $[EUART0]
// [EUART0]$

