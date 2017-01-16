#ifndef __18F47J53_H__
#define __18F47J53_H__

//////////// Standard Header file for the PIC18F47J53 device ////////////////
///////////////////////////////////////////////////////////////////////////
////        (C) Copyright 1996, 2013 Custom Computer Services          ////
//// This source code may only be used by licensed users of the CCS C  ////
//// compiler.  This source code may only be distributed to other      ////
//// licensed users of the CCS C compiler.  No other use, reproduction ////
//// or distribution is permitted without written permission.          ////
//// Derivative programs created using this software in object code    ////
//// form are not restricted in any way.                               ////
///////////////////////////////////////////////////////////////////////////
#device PIC18F47J53

#nolist
//////// Program memory: 65532x16  Data RAM: 3760  Stack: 31
//////// I/O: 33   Analog Pins: 13
//////// C Scratch area: 00   ID Location: 0000
//////// Fuses: NOWDT,WDT,PLL12,PLL10,PLL6,PLL5,PLL4,PLL3,PLL2,PLL1,PLLEN
//////// Fuses: NOPLLEN,NOSTVREN,STVREN,NOXINST,XINST,DEBUG,NODEBUG,CPUDIV6
//////// Fuses: CPUDIV3,CPUDIV2,NOCPUDIV,PROTECT,NOPROTECT,INTRC_IO,INTRC
//////// Fuses: INTRC_PLL_IO,HS,HSPLL,EC,ECPLL,SOSC_LOW,SOSC_DIG,SOSC_HIGH
//////// Fuses: NOCLOCKOUT,CLOCKOUT,NOFCMEN,FCMEN,NOIESO,IESO,WDT1,WDT2,WDT4
//////// Fuses: WDT8,WDT16,WDT32,WDT64,WDT128,WDT256,WDT512,WDT1024,WDT2048
//////// Fuses: WDT4096,WDT8192,WDT16384,WDT32768,DSWDTOSC_T1,DSWDTOSC_INT
//////// Fuses: RTCOSC_INT,RTCOSC_T1,NODSBOR,DSBOR,NODSWDT,DSWDT,DSWDT2
//////// Fuses: DSWDT8,DSWDT32,DSWDT128,DSWDT512,DSWDT2048,DSWDT8192
//////// Fuses: DSWDT32768,DSWDT131072,DSWDT524288,DSWDT2097152,DSWDT8388608
//////// Fuses: DSWDT33554432,DSWDT134217728,DSWDT536870912,DSWDT2147483648
//////// Fuses: NOIOL1WAY,IOL1WAY,ADC12,ADC10,MSSPMSK5,MSSPMSK7,NOWPFP,WPFP
//////// Fuses: WPCFG,NOWPCFG,NOWPDIS,WPDIS,WPBEG,WPEND,LS24MHZ,LS48MHZ
////////
#if (!defined(__PCH__)||defined(__ISNT_CCS__))
#define _bif

#ifndef int8
#define int8 char
#endif

#ifndef int16
#define int16 long
#endif

#ifndef int32
#define int32 long long
#endif

#ifndef float32
#define float32 float
#endif

#ifndef int1
#define int1 char
#endif

#endif
////////////////////////////////////////////////////////////////// I/O
// Discrete I/O Functions: SET_TRIS_x(), OUTPUT_x(), INPUT_x(),
//                         PORT_x_PULLUPS(), INPUT(),
//                         OUTPUT_LOW(), OUTPUT_HIGH(),
//                         OUTPUT_FLOAT(), OUTPUT_BIT()
// Discrete I/O Prototypes:
_bif void set_tris_a(int8 value);
_bif void set_tris_b(int8 value);
_bif void set_tris_c(int8 value);
_bif void set_tris_d(int8 value);
_bif void set_tris_e(int8 value);
_bif int8 get_tris_a(void);
_bif int8 get_tris_b(void);
_bif int8 get_tris_c(void);
_bif int8 get_tris_d(void);
_bif int8 get_tris_e(void);
_bif void output_a(int8 value);
_bif void output_b(int8 value);
_bif void output_c(int8 value);
_bif void output_d(int8 value);
_bif void output_e(int8 value);
_bif int8 input_a(void);
_bif int8 input_b(void);
_bif int8 input_c(void);
_bif int8 input_d(void);
_bif int8 input_e(void);
_bif int1 input_change_a(void);
_bif int1 input_change_b(void);
_bif int1 input_change_c(void);
_bif int1 input_change_d(void);
_bif int1 input_change_e(void);
_bif int1 input(int16 pin);
_bif int1 input_state(int16 pin);
_bif void output_low(int16 pin);
_bif void output_high(int16 pin);
_bif void output_toggle(int16 pin);
_bif void output_bit(int16 pin, int1 level);
_bif void output_float(int16 pin);
_bif void output_drive(int16 pin);

// Constants used to identify pins in the above are:

#define PIN_A0  31744
#define PIN_A1  31745
#define PIN_A2  31746
#define PIN_A3  31747
#define PIN_A5  31749
#define PIN_A6  31750
#define PIN_A7  31751

#define PIN_B0  31752
#define PIN_B1  31753
#define PIN_B2  31754
#define PIN_B3  31755
#define PIN_B4  31756
#define PIN_B5  31757
#define PIN_B6  31758
#define PIN_B7  31759

#define PIN_C0  31760
#define PIN_C1  31761
#define PIN_C2  31762
#define PIN_C4  31764
#define PIN_C5  31765
#define PIN_C6  31766
#define PIN_C7  31767

#define PIN_D0  31768
#define PIN_D1  31769
#define PIN_D2  31770
#define PIN_D3  31771
#define PIN_D4  31772
#define PIN_D5  31773
#define PIN_D6  31774
#define PIN_D7  31775

#define PIN_E0  31776
#define PIN_E1  31777
#define PIN_E2  31778

////////////////////////////////////////////////////////////////// Useful defines
#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef BYTE
#define BYTE int8
#endif

#ifndef BOOLEAN
#define BOOLEAN int1
#endif

#define getc getch
#define fgetc getch
#define getchar getch
#define putc putchar
#define fputc putchar
#define fgets gets
#define fputs puts

////////////////////////////////////////////////////////////////// Control
// Control Functions:  RESET_CPU(), SLEEP(), RESTART_CAUSE()
// Prototypes:
_bif int8 restart_cause(void);
_bif void reset_cpu(void);
_bif void sleep(void);
_bif void sleep(int8 type);
// Constants returned from RESTART_CAUSE() are:
#define WDT_TIMEOUT       7
#define MCLR_FROM_SLEEP  11
#define MCLR_FROM_RUN    15
#define NORMAL_POWER_UP  12
#define BROWNOUT_RESTART 14
#define WDT_FROM_SLEEP    3
#define RESET_INSTRUCTION 0

#define POR_FROM_DS      0x0100
#define MCLR_FROM_DS     0x0400
#define RTC_FROM_DS      0x0800
#define DSWDT_FROM_DS    0x1000
#define ULPWU_FROM_DS    0x2000
#define FAULT_FROM_DS    0x8000
#define EXT_FROM_DS      0x0200

//Constants for calls to sleep() to enable deep sleep mode are:
#define DEEP_SLEEP       3
// Any of the following may be OR'ed in with the above using |
#define WAKE_FROM_RTCC   4  // Wake-up from RTCC is enabled
#define WAKE_FROM_ULP    8  // ULPWU module is enabled in Deep Sleep

#byte DSGPR0 = getenv("SFR:DSGPR0")
#byte DSGPR1 = getenv("SFR:DSGPR1")

// Constants for calls to sleep() are:
#define REG_LOW_POWER    2
#define REG_NORMAL       0  // default

// Constants for calls to sleep() are:
#define SLEEP_FULL       0  // Default
#define SLEEP_IDLE       1  // Clock and peripherals don't stop

////////////////////////////////////////////////////////////////// Timer 0
// Timer 0 (AKA RTCC)Functions: SETUP_COUNTERS() or SETUP_TIMER_0(),
//                              SET_TIMER0() or SET_RTCC(),
//                              GET_TIMER0() or GET_RTCC()
// Timer 0 Prototypes:
_bif void setup_timer_0(int8 mode);
_bif void set_timer0(int16 value);
_bif int16 get_timer0(void);
_bif void setup_counters(int8 mode, int8 prescaler);
_bif void set_rtcc(int16 value);
_bif int16 get_rtcc(void);
// Constants used for SETUP_TIMER_0() are:
#define T0_INTERNAL   0
#define T0_EXT_L_TO_H 32
#define T0_EXT_H_TO_L 48

#define T0_DIV_1      8
#define T0_DIV_2      0
#define T0_DIV_4      1
#define T0_DIV_8      2
#define T0_DIV_16     3
#define T0_DIV_32     4
#define T0_DIV_64     5
#define T0_DIV_128    6
#define T0_DIV_256    7

#define T0_OFF        0x80

#define T0_8_BIT      0x40

#define RTCC_INTERNAL   0      // The following are provided for compatibility
#define RTCC_EXT_L_TO_H 32     // with older compiler versions
#define RTCC_EXT_H_TO_L 48
#define RTCC_DIV_1      8
#define RTCC_DIV_2      0
#define RTCC_DIV_4      1
#define RTCC_DIV_8      2
#define RTCC_DIV_16     3
#define RTCC_DIV_32     4
#define RTCC_DIV_64     5
#define RTCC_DIV_128    6
#define RTCC_DIV_256    7
#define RTCC_OFF        0x80
#define RTCC_8_BIT      0x40

// Constants used for SETUP_COUNTERS() are the above
// constants for the 1st param and the following for
// the 2nd param:

////////////////////////////////////////////////////////////////// WDT
// Watch Dog Timer Functions: SETUP_WDT() or SETUP_COUNTERS() (see above)
//                            RESTART_WDT()
// WDT base is 4ms
// WDT Prototypes:
_bif void setup_wdt(int16 mode);
_bif void restart_wdt(void);
// Constants used for SETUP_WDT() are:
#define WDT_ON        0x100
#define WDT_OFF       0

#define WDT_4MS    0x300
#define WDT_8MS    0x500
#define WDT_16MS   0x700
#define WDT_32MS   0x900
#define WDT_64MS   0xB00
#define WDT_128MS  0xD00
#define WDT_256MS  0xF00
#define WDT_512MS  0x1100
#define WDT_1S     0x1300
#define WDT_2S     0x1500
#define WDT_4S     0x1700
#define WDT_8S     0x1900
#define WDT_16     0x1B00
#define WDT_32S    0x1D00
#define WDT_64S    0x1F00
#define WDT_128S   0x2100


////////////////////////////////////////////////////////////////// Timer 1
// Timer 1 Functions: SETUP_TIMER_1, GET_TIMER1, SET_TIMER1
// Timer 1 Prototypes:
_bif void setup_timer_1(int16 mode);
_bif int16 get_timer1(void);
_bif void set_timer1(int16 value);
// Constants used for SETUP_TIMER_1() are:
//      (or (via |) together constants from each group)
#define T1_DISABLED         0
#define T1_INTERNAL         0x07
#define T1_EXTERNAL         0x87
#define T1_EXTERNAL_SYNC    0x83
#define T1_FOSC             0x47

#define T1_ENABLE_SOSC    0x08

#define T1_DIV_BY_1         0
#define T1_DIV_BY_2         0x10
#define T1_DIV_BY_4         0x20
#define T1_DIV_BY_8         0x30

#define T1_GATE           0x8000
#define T1_GATE_INVERTED  0xC000
#define T1_GATE_TOGGLE    0xA000
#define T1_GATE_SINGLE    0x9000
#define T1_GATE_TIMER2    0x8100
#define T1_GATE_COMP1     0x8200
#define T1_GATE_COMP2     0x8300

////////////////////////////////////////////////////////////////// Timer 2
// Timer 2 Functions: SETUP_TIMER_2, GET_TIMER2, SET_TIMER2
// Timer 2 Prototypes:
_bif void setup_timer_2(int8 mode, int8 period, int8 postscale);
_bif int8 get_timer2(void);
_bif void set_timer2(int8 value);
// Constants used for SETUP_TIMER_2() are:
#define T2_DISABLED         0
#define T2_DIV_BY_1         4
#define T2_DIV_BY_4         5
#define T2_DIV_BY_16        6

////////////////////////////////////////////////////////////////// Timer 3
// Timer 3 Functions: SETUP_TIMER_3, GET_TIMER3, SET_TIMER3
// Timer 3 Prototypes:
_bif void setup_timer_3(int16 mode);
_bif int16 get_timer3(void);
_bif void set_timer3(int16 value);
// Constants used for SETUP_TIMER_3() are:
//      (or (via |) together constants from each group)
#define T3_DISABLED         0
#define T3_INTERNAL         0x07
#define T3_EXTERNAL         0x87
#define T3_EXTERNAL_SYNC    0x83
#define T3_FOSC             0x47

#define T3_ENABLE_SOSC    0x08

#define T3_DIV_BY_1         0
#define T3_DIV_BY_2         0x10
#define T3_DIV_BY_4         0x20
#define T3_DIV_BY_8         0x30

#define T3_GATE           0x8000
#define T3_GATE_INVERTED  0xC000
#define T3_GATE_TOGGLE    0xA000
#define T3_GATE_SINGLE    0x9000
#define T3_GATE_TIMER4    0x8100
#define T3_GATE_COMP1     0x8200
#define T3_GATE_COMP2     0x8300

////////////////////////////////////////////////////////////////// Timer 4
// Timer 4 Functions: SETUP_TIMER_4, GET_TIMER4, SET_TIMER4
// Timer 4 Prototypes:
_bif void setup_timer_4(int16 mode,int8 period, int8 postscaler);
_bif int8 get_timer4(void);
_bif void set_timer4(int8 value);
// Constants used for SETUP_TIMER_4() are:
#define T4_DISABLED         0
#define T4_DIV_BY_1         4
#define T4_DIV_BY_4         5
#define T4_DIV_BY_16        6

////////////////////////////////////////////////////////////////// Timer 5
// Timer 5 Functions: SETUP_TIMER_5, GET_TIMER5, SET_TIMER5
// Timer 5 Prototypes:
_bif void setup_timer_5(int16 mode);
_bif int16 get_timer5(void);
_bif void set_timer5(int16 value);
// Constants used for SETUP_TIMER_5() are:
//      (or (via |) together constants from each group)
#define T5_DISABLED         0
#define T5_INTERNAL         0x07
#define T5_EXTERNAL         0x87
#define T5_EXTERNAL_SYNC    0x83
#define T5_FOSC             0x47

#define T5_ENABLE_SOSC    0x08

#define T5_DIV_BY_1         0
#define T5_DIV_BY_2         0x10
#define T5_DIV_BY_4         0x20
#define T5_DIV_BY_8         0x30

#define T5_GATE           0x8000
#define T5_GATE_INVERTED  0xC000
#define T5_GATE_TOGGLE    0xA000
#define T5_GATE_SINGLE    0x9000
#define T5_GATE_TIMER6    0x8100
#define T5_GATE_COMP1     0x8200
#define T5_GATE_COMP2     0x8300

////////////////////////////////////////////////////////////////// Timer 6
// Timer 6 Functions: SETUP_TIMER_6, GET_TIMER5, SET_TIMER5
// Timer 6 Prototypes:
_bif void setup_timer_6(int8 mode, int8 period, int8 postscaler);
_bif int8 get_timer6(void);
_bif void set_timer6(int8 value);
// Constants used for SETUP_TIMER_6() are:
//      (or (via |) together constants from each group)
#define T6_DISABLED         0
#define T6_DIV_BY_1         4
#define T6_DIV_BY_4         5
#define T6_DIV_BY_16        6

////////////////////////////////////////////////////////////////// Timer 8
// Timer 8 Functions: SETUP_TIMER_8, GET_TIMER8, SET_TIMER8
// Timer 8 Prototypes:
_bif void setup_timer_8(int8 mode, int8 period, int8 postscaler);
_bif int8 get_timer8(void);
_bif void set_timer8(int8 value);
// Constants used for SETUP_TIMER_8() are:
//      (or (via |) together constants from each group)
#define T8_DISABLED         0
#define T8_DIV_BY_1         4
#define T8_DIV_BY_4         5
#define T8_DIV_BY_16        6

////////////////////////////////////////////////////////////////// RTC
// RTC Functions: setup_rtc(), setup_rtc_alarm(), rtc_read(),
//                rtc_write(), rtc_alarm_read(), rtc_alarm_write()
// Structure used in read and write functions (pass pointer):
typedef struct {
   int8 tm_year;
   int8 tm_temp;  // Not used by built in functions, place holder do not use
   int8 tm_mday;
   int8 tm_mon;
   int8 tm_hour;
   int8 tm_wday;
   int8 tm_sec;
   int8 tm_min;
   int8 tm_isdst; // Not used by built in functions
} rtc_time_t;
// RTC Prototypes:
_bif void setup_rtc(int32 mode);
_bif void setup_rtc(int32 mode, int8 calibration);
_bif void setup_rtc_alarm(int8 mode, int8 mask, int8 repeat);
_bif void rtc_read(rtc_time_t *time);
_bif void rtc_write(rtc_time_t *time);
_bif void rtc_read_alarm(rtc_time_t *time);
_bif void rtc_write_alarm(rtc_time_t *time);
// Constants used in setup_rtc() are:  // Second param is calibration
#define RTC_ENABLE           0x8000
#define RTC_DISABLE          0
#define RTC_OUTPUT_SOURCE    0x20400
#define RTC_OUTPUT_SECONDS   0x10400
#define RTC_OUTPUT_ALARM     0x00400
//
// Constants used in setup_rtc_alarm() first param are:
#define RTC_ALARM_ENABLE     0x80
#define RTC_ALARM_DISABLE    0
#define RTC_CHIME_ENABLE     0x40
#define RTC_CHIME_DISABLE    0
// Constants used in setup_rtc_alarm() second param are:  // Third param is repeat#
#define RTC_ALARM_HALFSECOND 0x00
#define RTC_ALARM_SECOND     0x04
#define RTC_ALARM_10_SECONDS 0x08
#define RTC_ALARM_MINUTE     0x0C
#define RTC_ALARM_10_MINUTES 0x10
#define RTC_ALARM_HOUR       0x14
#define RTC_ALARM_DAY        0x18
#define RTC_ALARM_WEEK       0x1C
#define RTC_ALARM_MONTH      0x20
#define RTC_ALARM_YEAR       0x24

////////////////////////////////////////////////////////////////// CCP
// CCP Functions: SETUP_CCPx, SET_PWMx_DUTY
// CCP Variables: CCP_x, CCP_x_LOW, CCP_x_HIGH
// CCP1 Prototypes:
_bif void setup_ccp1(int32 mode);
_bif void setup_ccp1(int32 mode, int8 pwm);
_bif void set_pwm1_duty(int16 value);
// Constants used for SETUP_CCPx() are:
#define CCP_OFF                         0
#define CCP_CAPTURE_FE                  4
#define CCP_CAPTURE_RE                  5
#define CCP_CAPTURE_DIV_4               6
#define CCP_CAPTURE_DIV_16              7
#define CCP_COMPARE_SET_ON_MATCH        8
#define CCP_COMPARE_CLR_ON_MATCH        9
#define CCP_COMPARE_INT                 0xA
#define CCP_COMPARE_INT_AND_TOGGLE      0x2
#define CCP_COMPARE_RESET_TIMER         0xB
#define CCP_PWM                         0xC
#define CCP_PWM_PLUS_1                  0x1c
#define CCP_PWM_PLUS_2                  0x2c
#define CCP_PWM_PLUS_3                  0x3c
//#define CCP_USE_TIMER3                0x100  OBSOLETE, SEE TIMER-3
#word   CCP_1       =                   getenv("SFR:CCPR1L")
#byte   CCP_1_LOW   =                   getenv("SFR:CCPR1L")
#byte   CCP_1_HIGH  =                   getenv("SFR:CCPR1H")
// The following are used to select the Timer source for the CCP/ECCP
// The first timer is the timer used when in CAPTURE or COMPARE mode
// The second timer is the timer used when in PWM mode
#define CCP_USE_TIMER1_AND_TIMER2       0x0000 //used for CCP 1 2 3 4 5 6 7 8 9 10
#define CCP_USE_TIMER3_AND_TIMER4       0x0100 //used for CCP 1 2 3 4
#define CCP_USE_TIMER3_AND_TIMER6       0x0200 //used for CCP 1 2 3 4
#define CCP_USE_TIMER3_AND_TIMER8       0x0300 //used for CCP 1 2 3
#define CCP_USE_TIMER5_AND_TIMER4       0x0100 //used for CCP 5 7
#define CCP_USE_TIMER5_AND_TIMER2       0x0100 //used for CCP 6
#define CCP_USE_TIMER5_AND_TIMER6       0x0200 //used for CCP 7
#define CCP_USE_TIMER5_AND_TIMER8       0x0300 //used for CCP 7
#define CCP_USE_TIMER1_AND_TIMER6       0x0200 //used for CCP 8
#define CCP_USE_TIMER1_AND_TIMER4       0x0100 //used for CCP 8 9

// The following should be used with the ECCP unit only (or these in)
#define CCP_PWM_H_H                     0x0c
#define CCP_PWM_H_L                     0x0d
#define CCP_PWM_L_H                     0x0e
#define CCP_PWM_L_L                     0x0f

#define CCP_PWM_FULL_BRIDGE             0x40
#define CCP_PWM_FULL_BRIDGE_REV         0xC0
#define CCP_PWM_HALF_BRIDGE             0x80

#define CCP_SHUTDOWN_ON_COMP1           0x100000
#define CCP_SHUTDOWN_ON_COMP2           0x200000
#define CCP_SHUTDOWN_ON_COMP            0x300000
#define CCP_SHUTDOWN_ON_INT0            0x400000
#define CCP_SHUTDOWN_ON_COMP1_INT0      0x500000
#define CCP_SHUTDOWN_ON_COMP2_INT0      0x600000
#define CCP_SHUTDOWN_ON_COMP_INT0       0x700000

#define CCP_SHUTDOWN_AC_L               0x000000
#define CCP_SHUTDOWN_AC_H               0x040000
#define CCP_SHUTDOWN_AC_F               0x080000

#define CCP_SHUTDOWN_BD_L               0x000000
#define CCP_SHUTDOWN_BD_H               0x010000
#define CCP_SHUTDOWN_BD_F               0x020000

#define CCP_SHUTDOWN_RESTART            0x80000000

#define CCP_PULSE_STEERING_A            0x01000000
#define CCP_PULSE_STEERING_B            0x02000000
#define CCP_PULSE_STEERING_C            0x04000000
#define CCP_PULSE_STEERING_D            0x08000000
#define CCP_PULSE_STEERING_SYNC         0x10000000
#define CCP_COMPLEMENTARY_AB            0x43000000


// CCP2 Prototypes:
_bif void setup_ccp2(int32 mode);
_bif void setup_ccp2(int32 mode, int8 pwm);
_bif void set_pwm2_duty(int16 value);
#word   CCP_2       =                   getenv("SFR:CCPR2L")
#byte   CCP_2_LOW   =                   getenv("SFR:CCPR2L")
#byte   CCP_2_HIGH  =                   getenv("SFR:CCPR2H")

// CCP3 Prototypes:
_bif void setup_ccp3(int32 mode);
_bif void setup_ccp3(int32 mode, int8 pwm);
_bif void set_pwm3_duty(int16 value);
#word   CCP_3       =                   getenv("SFR:CCPR3L")
#byte   CCP_3_LOW   =                   getenv("SFR:CCPR3L")
#byte   CCP_3_HIGH  =                   getenv("SFR:CCPR3H")

// CCP4 Prototypes:
_bif void setup_ccp4(int32 mode);
_bif void setup_ccp4(int32 mode, int8 pwm);
_bif void set_pwm4_duty(int16 value);
#word   CCP_4       =                   getenv("SFR:CCPR4L")
#byte   CCP_4_LOW   =                   getenv("SFR:CCPR4L")
#byte   CCP_4_HIGH  =                   getenv("SFR:CCPR4H")

// CCP5 Prototypes:
_bif void setup_ccp5(int32 mode);
_bif void setup_ccp5(int32 mode, int8 pwm);
_bif void set_pwm5_duty(int16 value);
#word   CCP_5       =                   getenv("SFR:CCPR5L")
#byte   CCP_5_LOW   =                   getenv("SFR:CCPR5L")
#byte   CCP_5_HIGH  =                   getenv("SFR:CCPR5H")

// CCP6 Prototypes:
_bif void setup_ccp6(int32 mode);
_bif void setup_ccp6(int32 mode, int8 pwm);
_bif void set_pwm6_duty(int16 value);
#word   CCP_6       =                   getenv("SFR:CCPR6L")
#byte   CCP_6_LOW   =                   getenv("SFR:CCPR6L")
#byte   CCP_6_HIGH  =                   getenv("SFR:CCPR6H")

// CCP7 Prototypes:
_bif void setup_ccp7(int32 mode);
_bif void setup_ccp7(int32 mode, int8 pwm);
_bif void set_pwm7_duty(int16 value);
#word   CCP_7       =                   getenv("SFR:CCPR7L")
#byte   CCP_7_LOW   =                   getenv("SFR:CCPR7L")
#byte   CCP_7_HIGH  =                   getenv("SFR:CCPR7H")

// CCP8 Prototypes:
_bif void setup_ccp8(int32 mode);
_bif void setup_ccp8(int32 mode, int8 pwm);
_bif void set_pwm8_duty(int16 value);
#word   CCP_8       =                   getenv("SFR:CCPR8L")
#byte   CCP_8_LOW   =                   getenv("SFR:CCPR8L")
#byte   CCP_8_HIGH  =                   getenv("SFR:CCPR8H")

// CCP9 Prototypes:
_bif void setup_ccp9(int32 mode);
_bif void setup_ccp9(int32 mode, int8 pwm);
_bif void set_pwm9_duty(int16 value);
#word   CCP_9       =                   getenv("SFR:CCPR9L")
#byte   CCP_9_LOW   =                   getenv("SFR:CCPR9L")
#byte   CCP_9_HIGH  =                   getenv("SFR:CCPR9H")

// CCP10 Prototypes:
_bif void setup_ccp10(int32 mode);
_bif void setup_ccp10(int32 mode, int8 pwm);
_bif void set_pwm10_duty(int16 value);
#word   CCP_10      =                   getenv("SFR:CCPR10L")
#byte   CCP_10_LOW  =                   getenv("SFR:CCPR10L")
#byte   CCP_10_HIGH =                   getenv("SFR:CCPR10H")

////////////////////////////////////////////////////////////////// PMP
// PP Functions: SETUP_PSP(),
// PSP_INPUT_FULL(), PSP_OUTPUT_FULL(),
//                PSP_OVERFLOW(), INPUT_D(), OUTPUT_D()
// PP Variables: PSP_DATA
// PP Prototypes:
_bif void setup_psp(int32 mode);
_bif int1 psp_output_full(void);
_bif int1 psp_input_full(void);
_bif int1 psp_overflow(void);
// Constants used in SETUP_PSP() are:
#define PMP_DISABLED     0
#define PMP_ENABLED      0x02008000
#define PMP_COMBINE_RW   0x01000000
#define PSP_ADDRESS_MODE 0x01000000
#define PSP_NORMAL_MODE  0x18000000
#define PMP_16_BIT       0x04000000
#define PMP_INC_ADDR     0x08000000
#define PMP_DEC_ADDR     0x10000000
#define PMP_INT          0x20000000
#define PMP_STALL        0x40000000
#define PMP_INT_3        0x60000000

#define PMP_RD_ACT_HIGH  0x00000101
#define PMP_WR_ACT_HIGH  0x00000202
#define PMP_RD_ACT_LOW   0x00000100
#define PMP_WR_ACT_LOW   0x00000200
#define PMP_BE_ACT_HIGH  0x00000404
#define PMP_BE_ACT_LOW   0x00000400
#define PMP_CS1_ACT_HIGH 0x00000008
#define PMP_CS2_ACT_HIGH 0x00000010
#define PMP_AL_ACT_HIGH  0x00000020
#define PMP_ADDR_MUX_A_D 0x00000800
#define PMP_ADDR_MUX_D   0x00001000
#define PMP_IDLE_OFF     0x00002000

#define PMP_WAITE        0x00010000  // Data hold after strobe, add *2 for 3Tcy or *3 for 4Tcy
#define PMP_WAITM        0x00040000  // Data hold read to strobe, add *n for n*Tcy (n=1..15)
#define PMP_WAITB        0x00400000  // Data hold before R/W, add *2 for 3Tcy or *3 for 4Tcy

#define PP_ACTIVE_IDLE   0x00002000

////////////////////////////////////////////////////////////////// SPI
// SPI Functions: SETUP_SPI, SPI_WRITE, SPI_READ, SPI_DATA_IN
// SPI Prototypes:
_bif void setup_spi(int32 mode);
_bif void spi_write(int8 value);
_bif int8 spi_read(void);
_bif int8 spi_read(int8 value);
_bif int1 spi_data_in(void);
_bif void setup_spi2(int32 mode);
_bif void spi_write2(int8 value);
_bif int8 spi_read2(void);
_bif int8 spi_read2(int8 value);
_bif int1 spi_data_in2(void);
// Constants used in SETUP_SPI() are:
#define SPI_DISABLED             0x00
#define SPI_MASTER               0x20
#define SPI_SLAVE                0x24
#define SPI_SCK_IDLE_HIGH        0x10
#define SPI_SCK_IDLE_LOW         0x00
#define SPI_CLK_DIV_4            0x00
#define SPI_CLK_DIV_16           0x01
#define SPI_CLK_DIV_64           0x02
#define SPI_CLK_T2               0x03
#define SPI_SS_DISABLED          0x01

#define SPI_XMIT_L_TO_H          0x4000
#define SPI_XMIT_H_TO_L          0x0000

#define SPI_SAMPLE_AT_MIDDLE     0x0000
#define SPI_SAMPLE_AT_END        0x8000

//The following are provided for compatibility
#define SPI_L_TO_H       SPI_SCK_IDLE_LOW
#define SPI_H_TO_L       SPI_SCK_IDLE_HIGH

////////////////////////////////////////////////////////////////// UART
// UART Prototypes:
_bif void setup_uart(int32 baud);
_bif void setup_uart(int32 baud, int8 stream);
_bif void setup_uart(int32 baud, int8 stream, int32 clock);
_bif void set_uart_speed(int32 baud);
_bif void set_uart_speed(int32 baud, int8 stream);
_bif void set_uart_speed(int32 baud, int8 stream, int32 clock);
// Constants used in setup_uart() are:
// FALSE - Turn UART off
// TRUE  - Turn UART on
#define UART_ADDRESS           2
#define UART_DATA              4
#define UART_AUTODETECT        8
#define UART_AUTODETECT_NOWAIT 9
#define UART_WAKEUP_ON_RDA     10
#define UART_SEND_BREAK        13
////////////////////////////////////////////////////////////////// COMP
// Comparator Variables: C1OUT, C2OUT, C3OUT
// Comparator Prototypes:
_bif void setup_comparator(int32 mode);
// Constants used in setup_comparator() are:
//
#define NC_NC_NC_NC   0x040404
#define NC_NC         0x040404

//Pick one constant for COMP1
#define CP1_A3_A0    0x000080
#define CP1_A5_A0    0x000081
#define CP1_A2_A0    0x000082
#define CP1_FVR_A0    0x000083
#define CP1_A3_VREF    0x000084
#define CP1_A5_VREF    0x000085
#define CP1_A2_VREF    0x000086
#define CP1_FVR_VREF  0x000087
//Optionally OR with any of the following
#define CP1_OUT_ON_RP  0x000040
#define CP1_INVERT    0x000020
#define CP1_INT_L2H    0x000008
#define CP1_INT_H2L    0x000010

//Pick one constant for COMP2
#define CP2_A2_A1    0x008000
#define CP2_B2_A1    0x008100
#define CP2_C2_A1    0x008200
#define CP2_FVR_A1    0x008300
#define CP2_A2_VREF    0x008400
#define CP2_B2_VREF    0x008500
#define CP2_C2_VREF    0x008600
#define CP2_FVR_VREF  0x008700
//Optionally OR with any of the following
#define CP2_OUT_ON_RP  0x004000
#define CP2_INVERT    0x002000
#define CP2_INT_L2H    0x000800
#define CP2_INT_H2L    0x001000

//Pick one constant for COMP3
#define CP3_A2_B3    0x800000
#define CP3_B1_B3    0x810000
#define CP3_B0_B3    0x820000
#define CP3_FVR_B3    0x830000
#define CP3_A2_VREF    0x840000
#define CP3_B1_VREF    0x850000
#define CP3_B0_VREF    0x860000
#define CP3_FVR_VREF  0x870000
//Optionally OR with any of the following
#define CP3_OUT_ON_RP  0x400000
#define CP3_INVERT    0x200000
#define CP3_INT_L2H    0x080000
#define CP3_INT_H2L    0x100000

#bit C1OUT = 0xF70.0
#bit C2OUT = 0xF70.1
#bit C3OUT = 0xF70.2

////////////////////////////////////////////////////////////////// VREF
// VREF Prototypes:
_bif void setup_vref(int8 mode);
// Constants used in setup_vref() are:
#define VREF_OFF     0
#define VREF_LOW     0xA0
#define VREF_HIGH    0x80
#define VREF_EXT     0x10  // Range for ref gen VREF- to VREF+
#define VREF_OUT_A2  0x40
// Or (with |) the above with a number 0-15

////////////////////////////////////////////////////////////////// VREF
// LVD Prototypes:
_bif void setup_low_volt_detect(int8 mode);
// CONSTANTS USED IN SETUP_LOW_VOLT_DETECT() ARE
#define LVD_LVDIN   0x1F
#define LVD_34      0x1E
#define LVD_31      0x1D
#define LVD_30      0x1C
#define LVD_29      0x1B
#define LVD_28      0x1A
#define LVD_26      0x19
#define LVD_24      0x18

#define LVD_TRIGGER_BELOW 0
#define LVD_TRIGGER_ABOVE 0x80


////////////////////////////////////////////////////////////////// INTERNAL RC
// Oscillator Prototypes:
_bif int8 setup_oscillator(int8 mode);
_bif int8 setup_oscillator(int8 mode, signed int8 tune);
// Constants used in setup_oscillator() are:
#define OSC_31250   0x8000
#define OSC_PLL_ON  0x4000
#define OSC_PLL_OFF 0
#define OSC_31KHZ   0
#define OSC_125KHZ  0x10
#define OSC_250KHZ  0x20
#define OSC_500KHZ  0x30
#define OSC_1MHZ    0x40
#define OSC_2MHZ    0x50
#define OSC_4MHZ    0x60
#define OSC_8MHZ    0x70
// The following may be OR'ed in with the above using |
#define OSC_TIMER1  1
#define OSC_INTRC   3
#define OSC_NORMAL  0
// The following may be OR'ed in with the above using |
#define OSC_IDLE_MODE  0x80
// A second optional parameter may be used with this part to fine
// tune the speed (signed int,-32 to 31)
// Result may be (ignore all other bits)
#define OSC_STATE_STABLE 4
#define OSC_STATE_EXT_RUNNING 8


////////////////////////////////////////////////////////////////// ADC
// ADC Functions: SETUP_ADC(), SETUP_ADC_PORTS() (aka SETUP_PORT_A),
//                SET_ADC_CHANNEL(), READ_ADC()
// ADC Prototypes:
_bif void setup_adc(int16 mode);
_bif int8 read_adc(void);
_bif int8 read_adc(int8 mode);
_bif int16 read_adc(void);
_bif int16 read_adc(int8 mode);
_bif int1 adc_done(void);
// Constants used for SETUP_ADC() are:
#define ADC_OFF               0           // ADC Off
#define ADC_CLOCK_DIV_2   0x100
#define ADC_CLOCK_DIV_4    0x04
#define ADC_CLOCK_DIV_8    0x01
#define ADC_CLOCK_DIV_16   0x05
#define ADC_CLOCK_DIV_32   0x02
#define ADC_CLOCK_DIV_64   0x06
#define ADC_CLOCK_INTERNAL 0x03           // Internal 2-6us
// The following may be OR'ed in with the above using |
#define ADC_TAD_MUL_0      0x00
#define ADC_TAD_MUL_2      0x08
#define ADC_TAD_MUL_4      0x10
#define ADC_TAD_MUL_6      0x18
#define ADC_TAD_MUL_8      0x20
#define ADC_TAD_MUL_12     0x28
#define ADC_TAD_MUL_16     0x30
#define ADC_TAD_MUL_20     0x38

//ADC Prototypes:
_bif void setup_adc_ports(int32 pins);
_bif void setup_adc_ports(int32 pins, int32 reference);
_bif void set_adc_channel(int8 channel);
// Constants used in SETUP_ADC_PORTS() are:
// First argument:
// OR together desired pins
#define sAN12            0x10     //| B0
#define sAN11            0x8      //| C2
#define sAN10            0x4      //| B1
#define sAN9             0x2      //| B3
#define sAN8             0x1      //| B2
#define sAN7            0x800000  //| E2
#define sAN6            0x400000  //| E1
#define sAN5            0x200000  //| E0
#define sAN4            0x100000  //| A5
#define sAN3             0x80000  //| A3
#define sAN2             0x40000  //| A2
#define sAN1             0x20000  //| A1
#define sAN0             0x10000  //| A0
#define NO_ANALOGS           0x0  // None
#define ALL_ANALOG      0xff00ff  // A0 A1 A2 A3 A5 E0 E1 E2 B2 B3 B1 C2 B0

// Optional Second argument:
#define VSS_VDD              0x0000        //| Range 0-Vdd
#define VSS_VREF             0x4000        //| Range 0-Vref
#define VREF_VREF            0xc000        //| Range Vref-Vref
#define VREF_VDD             0x8000        //| Range Vref-Vdd

// Constants used in READ_ADC() are:
#define ADC_START_AND_READ     7   // This is the default if nothing is specified
#define ADC_START_ONLY         1
#define ADC_READ_ONLY          6

////////////////////////////////////////////////////////////////// BIF
// Built In Functions Prototypes
//
// Math Prototypes:
_bif signed int8 abs(signed int8 x);
_bif signed int16 abs(signed int16 x);
_bif signed int32 abs(signed int32 x);
_bif float32 abs(float32 x);
_bif unsigned int16 _mul(unsigned int8, unsigned int8);
_bif signed int16 _mul(signed int8, signed int8);
_bif unsigned int32 _mul(unsigned int16, unsigned int16);
_bif signed int32 _mul(signed int16, signed int16);

// Memory Manipulation Prototypes:
_bif int8 read_bank(int8 bank, int8 offset);
_bif void write_bank(int8 bank, int8 offset, int8 value);
_bif void strcpy(char* dest, char* src);
_bif void strcopy(char* dest, char* src);
_bif void memset(unsigned int8* destination, unsigned int8 value, unsigned int16 num);
_bif void memcpy(unsigned int8* destination, unsigned int8* source, unsigned int16 num);

// String Prototypes:
_bif char toupper(char cvalue);
_bif char tolower(char cvalue);
_bif void sprintf(char* string, char* cstring, ...);

// Data Manipulators Prototypes:
_bif int1 shift_left(unsigned int8* address, unsigned int8 bytes, int1 value);
_bif int1 shift_right(unsigned int8* address, unsigned int8 bytes, int1 value);
_bif void rotate_left(unsigned int8* address, unsigned int8 bytes);
_bif void rotate_right(unsigned int8* address, unsigned int8 bytes);
_bif void swap(unsigned int8 value);
_bif unsigned int8 make8(unsigned int16 var, unsigned int8 offset);
_bif unsigned int8 make8(unsigned int32 var, unsigned int8 offset);
_bif unsigned int16 make16(unsigned int8 varhigh, unsigned int8 varlow);
_bif unsigned int32 make32(unsigned int16 var1);
_bif unsigned int32 make32(unsigned int16 var1, unsigned int16 var2);
_bif unsigned int32 make32(unsigned int16 var1, unsigned int8 var2);
_bif unsigned int32 make32(unsigned int16 var1, unsigned int8 var2, unsigned int8 var3);
_bif unsigned int32 make32(unsigned int8 var1);
_bif unsigned int32 make32(unsigned int8 var1, unsigned int8 var2);
_bif unsigned int32 make32(unsigned int8 var1, unsigned int8 var2, unsigned int8 var3);
_bif unsigned int32 make32(unsigned int8 var1, unsigned int8 var2, unsigned int8 var3, unsigned int8 var4);
_bif unsigned int32 make32(unsigned int8 var1, unsigned int16 var2);
_bif unsigned int32 make32(unsigned int8 var1, unsigned int16 var2, unsigned int8 var3);
_bif unsigned int32 make32(unsigned int8 var1, unsigned int8 var2, unsigned int16 var3);
_bif void bit_set(unsigned int8 var, unsigned int8 bit);
_bif void bit_set(unsigned int16 var, unsigned int8 bit);
_bif void bit_set(unsigned int32 var, unsigned int8 bit);
_bif void bit_clear(unsigned int8 var, unsigned int8 bit);
_bif void bit_clear(unsigned int16 var, unsigned int8 bit);
_bif void bit_clear(unsigned int32 var, unsigned int8 bit);
_bif int1 bit_test(unsigned int8 var, unsigned int8 bit);
_bif int1 bit_test(unsigned int16 var, unsigned int8 bit);
_bif int1 bit_test(unsigned int32 var, unsigned int8 bit);

// #use delay() Prototypes:
_bif void delay_cycles(unsigned int8 count);
_bif void delay_ms(unsigned int16 time);
_bif void delay_us(unsigned int16 time);

// #use rs232() Prototypes:
_bif void putchar(char cdata);
_bif void putchar(char cdata, unsigned int8 stream);
_bif void puts(char* string);
_bif void puts(char* string, unsigned int8 stream);
_bif char getch(void);
_bif char getch(unsigned int8 stream);
_bif void gets(char* string);
_bif void gets(char* string, unsigned int8 stream);
_bif int1 kbhit(void);
_bif int1 kbhit(unsigned int8 stream);
_bif void printf(char* string, ...);
_bif void fprintf(unsigned int8 stream, char* string, ...);
_bif void putc_send(void);
_bif void fputc_send(unsigned int8 stream);
_bif int1 rcv_buffer_full(void);
_bif int1 rcv_buffer_full(unsigned int8 stream);
_bif unsigned int16 rcv_buffer_bytes(void);
_bif unsigned int16 rcv_buffer_bytes(unsigned int8 stream);
_bif int1 tx_buffer_full(void);
_bif int1 tx_buffer_full(unsigned int8 stream);
_bif unsigned int16 tx_buffer_bytes(void);
_bif unsigned int16 tx_buffer_bytes(unsigned int8 stream);

// #use i2c() Prototypes:
_bif unsigned int8 i2c_read(void);
_bif unsigned int8 i2c_read(unsigned int8 stream);
_bif unsigned int8 i2c_read(unsigned int8 stream, int1 ack);
_bif int1 i2c_write(unsigned int8 data);
_bif int1 i2c_write(unsigned int8 stream, unsigned int8 data);
_bif void i2c_start(void);
_bif void i2c_start(unsigned int8 stream);
_bif void i2c_start(unsigned int8 stream, unsigned int8 restart);
_bif void i2c_stop(void);
_bif void i2c_stop(unsigned int8 stream);
_bif int8 i2c_isr_state(void);
_bif void i2c_slaveaddr(unsigned int8 addr);
_bif void i2c_slaveaddr(unsigned int8 stream, unsigned int8 addr);
_bif int1 i2c_poll(void);
_bif int1 i2c_poll(unsigned int8 stream);
_bif void i2c_init(unsigned int32 baud);
_bif void i2c_init(unsigned int8 stream, unsigned int32 baud);

// #use spi() Prototypes:
_bif unsigned int8 spi_xfer(void);
_bif unsigned int16 spi_xfer(void);
_bif unsigned int32 spi_xfer(void);
_bif unsigned int8 spi_xfer(unsigned int8 data);
_bif unsigned int16 spi_xfer(unsigned int16 data);
_bif unsigned int32 spi_xfer(unsigned int32 data);
_bif unsigned int8 spi_xfer(unsigned int8 stream, unsigned int8 data);
_bif unsigned int16 spi_xfer(unsigned int8 stream, unsigned int16 data);
_bif unsigned int32 spi_xfer(unsigned int8 stream, unsigned int32 data);
_bif unsigned int8 spi_xfer(unsigned int8 stream, unsigned int8 data, unsigned int8 bits);
_bif unsigned int16 spi_xfer(unsigned int8 stream, unsigned int16 data, unsigned int8 bits);
_bif unsigned int32 spi_xfer(unsigned int8 stream, unsigned int32 data, unsigned int8 bits);
_bif void spi_init(unsigned int32 baud);
_bif void spi_init(unsigned int8 stream, unsigned int32 baud);
_bif void spi_speed(unsigned int32 baud);
_bif void spi_speed(unsigned int8 stream, unsigned int32 baud);
_bif void spi_speed(unsigned int8 stream, unsigned int32 baud, unsigned int32 clock);
_bif void spi_prewrite(unsigned int8 data);
_bif void spi_prewrite(unsigned int16 data);
_bif void spi_prewrite(unsigned int32 data);
_bif void spi_prewrite(unsigned int8, unsigned int8 data);
_bif void spi_prewrite(unsigned int8, unsigned int16 data);
_bif void spi_prewrite(unsigned int8, unsigned int32 data);
_bif unsigned int8 spi_xfer_in(void);
_bif unsigned int16 spi_xfer_in(void);
_bif unsigned int32 spi_xfer_in(void);
_bif unsigned int8 spi_xfer_in(unsigned int8 bits);
_bif unsigned int16 spi_xfer_in(unsigned int8 bits);
_bif unsigned int32 spi_xfer_in(unsigned int8 bits);
_bif unsigned int8 spi_xfer_in(unsigned int8 stream, unsigned int8 bits);
_bif unsigned int16 spi_xfer_in(unsigned int8 stream, unsigned int8 bits);
_bif unsigned int32 spi_xfer_in(unsigned int8 stream, unsigned int8 bits);

// #use rtos() Prototypes:
_bif void rtos_run(void);
_bif void rtos_yield(void);
_bif void rtos_enable(unsigned int8 task);
_bif void rtos_disable(unsigned int8 task);
_bif void rtos_terminate(void);
_bif void rtos_await(int1 flag);
_bif void rtos_wait(unsigned int8 sem);
_bif void rtos_signal(unsigned int8 sem);
_bif void rtos_msg_send(unsigned int8 task, unsigned int8 msg);
_bif unsigned int8 rtos_msg_read(void);
_bif unsigned int8 rtos_msg_poll(void);
_bif int1 rtos_overrun(unsigned int8 task);
_bif void rtos_stats(unsigned int8 task, unsigned int8* stat);

// #use timer() Prototypes:
_bif unsigned int8 get_ticks(void);
_bif unsigned int16 get_ticks(void);
_bif unsigned int32 get_ticks(void);
_bif unsigned int8 get_ticks(unsigned int8 stream);
_bif unsigned int16 get_ticks(unsigned int8 stream);
_bif unsigned int32 get_ticks(unsigned int8 stream);
_bif void set_ticks(unsigned int8 value);
_bif void set_ticks(unsigned int16 value);
_bif void set_ticks(unsigned int32 value);
_bif void set_ticks(unsigned int8 stream, unsigned int8 value);
_bif void set_ticks(unsigned int8 stream, unsigned int16 value);
_bif void set_ticks(unsigned int8 stream, unsigned int32 value);

// #use pwm() Prototypes:
_bif void pwm_on(void);
_bif void pwm_on(unsigned int8 stream);
_bif void pwm_off(void);
_bif void pwm_off(unsigned int8 stream);
_bif void pwm_set_duty(unsigned int16 duty);
_bif void pwm_set_duty(unsigned int8 stream, unsigned int16 duty);
_bif void pwm_set_duty_percent(unsigned int16 percent);
_bif void pwm_set_duty_percent(unsigned int8 stream, unsigned int16 percent);
_bif void pwm_set_frequency(unsigned int32 frequency);
_bif void pwm_set_frequency(unsigned int8 stream, unsigned int32 frequency);

// #use capture() Prototypes:
_bif unsigned int16 get_capture_time(void);
_bif unsigned int16 get_capture_time(unsigned int8 stream);
_bif int1 get_capture_event(void);
_bif int1 get_capture_event(unsigned int8 stream);

// Enviroment Prototypes:
//_bif unsigned int8 getenv(char* cstring);

// Address Prototypes:
#ifndef __ADDRESS__
 #if defined(__PCM__) || define(__PCB__)
  typedef unsigned int16 __ADDRESS__;
 #else
  typedef unsigned int32 __ADDRESS__;
 #endif
#endif
_bif void goto_address(__ADDRESS__ address);
_bif __ADDRESS__ label_address(__ADDRESS__ label);

// Program Memory Prototypes:
_bif void read_program_memory(__ADDRESS__ address, unsigned int8* dataptr, unsigned int16 count);
_bif unsigned int16 read_program_eeprom(__ADDRESS__ address);
_bif void erase_program_eeprom(__ADDRESS__ address);
_bif void write_program_memory(__ADDRESS__ address, unsigned int8* dataptr, unsigned int16 count);
_bif void write_program_eeprom(__ADDRESS__ address, unsigned int16 data);

// #use touchpad() Prototypes:
_bif void touchpad_state(unsigned int8 state);
_bif char touchpad_getc(void);
_bif int1 touchpad_hit(void);
////////////////////////////////////////////////////////////////// INT
// Interrupt Functions: ENABLE_INTERRUPTS(), DISABLE_INTERRUPTS(),
//                      CLEAR_INTERRUPT(), INTERRUPT_ACTIVE(),
//                      EXT_INT_EDGE()
// INT Prototypes:
_bif void enable_interrupts(int32 interrupt);
_bif void disable_interrupts(int32 interrupt);
_bif void clear_interrupt(int32 interrupt);
_bif int1 interrupt_active(int32 interrupt);
_bif int1 interrupt_enabled(int32 interrupt);
_bif void ext_int_edge(int8 source, int8 edge);
_bif void jump_to_irs(int16 address);
// Constants used in EXT_INT_EDGE() are:
#define L_TO_H              0x40
#define H_TO_L                 0
// Constants used in ENABLE/DISABLE_INTERRUPTS() are:
#define GLOBAL                    0xF2C0
#define PERIPH                    0xF240
#define INT_RTCC                  0x00F220
#define INT_TIMER0                0x00F220
#define INT_TIMER1                0x009D01
#define INT_TIMER2                0x009D02
#define INT_TIMER3                0x00A002
#define INT_EXT_L2H               0x5000F210
#define INT_EXT_H2L               0x6000F210
#define INT_EXT                   0x00F210
#define INT_EXT1_L2H              0x5001F008
#define INT_EXT1_H2L              0x6001F008
#define INT_EXT1                  0x00F008
#define INT_EXT2_L2H              0x5002F010
#define INT_EXT2_H2L              0x6002F010
#define INT_EXT2                  0x00F010
#define INT_RB                    0x01FFF208
#define INT_PSP                   0x009D80
#define INT_AD                    0x009D40
#define INT_RDA                   0x009D20
#define INT_TBE                   0x009D10
#define INT_SSP                   0x009D08
#define INT_CCP1                  0x009D04
#define INT_CCP2                  0x00A001
#define INT_BUSCOL                0x00A008
#define INT_RDA2                  0x00A320
#define INT_TBE2                  0x00A310
#define INT_TIMER4                0x00A308
#define INT_OSCF                  0x00A080
#define INT_HLVD                  0x00A004
#define INT_BUSCOL2               0x00A340
#define INT_EXT3_L2H              0x5003F020
#define INT_EXT3_H2L              0x6003F020
#define INT_EXT3                  0x00F020
#define INT_USB                   0x00A010
#define INT_SSP2                  0x00A380
#define INT_RTC                   0x00A301
#define INT_COMP                  0x00A020
#define INT_COMP2                 0x00A040
#define INT_TMR3G                 0x00A302
#define INT_CTMU                  0x00A308
#define INT_CCP3                  0x008E01
#define INT_CCP4                  0x008E02
#define INT_CCP5                  0x008E04
#define INT_CCP6                  0x008E08
#define INT_CCP7                  0x008E10
#define INT_CCP8                  0x008E20
#define INT_CCP9                  0x008E40
#define INT_CCP10                 0x008E80
#define INT_TMR1G                 0x009101
#define INT_TMR5G                 0x009102
#define INT_TIMER5                0x009104
#define INT_TIMER6                0x009108
#define INT_TIMER8                0x009110
#define INT_COMP3                 0x009120
#define INT_RB4                   0x110F208
#define INT_RB5                   0x120F208
#define INT_RB6                   0x140F208
#define INT_RB7                   0x180F208

#list

#endif
