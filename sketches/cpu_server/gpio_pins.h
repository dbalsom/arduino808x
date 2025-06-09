/*
    Arduino8088 Copyright 2022-2025 Daniel Balsom
    https://github.com/dbalsom/arduino_8088

    Permission is hereby granted, free of charge, to any person obtaining a
    copy of this software and associated documentation files (the “Software”),
    to deal in the Software without restriction, including without limitation
    the rights to use, copy, modify, merge, publish, distribute, sublicense,
    and/or sell copies of the Software, and to permit persons to whom the
    Software is furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER   
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
    DEALINGS IN THE SOFTWARE.
*/

// This header contains defines that enable direct register access to GPIO lines
// for the Arduino MEGA, DUE and GIGA.

// GIGA support is untested. I wouldn't recommend copying this for your own
// GIGA project.

#ifndef _GPIO_PINS_H
#define _GPIO_PINS_H

#define BIT00  (1u << 0)
#define BIT01  (1u << 1)
#define BIT02  (1u << 2)
#define BIT03  (1u << 3)
#define BIT04  (1u << 4)
#define BIT05  (1u << 5)
#define BIT06  (1u << 6)
#define BIT07  (1u << 7)
#define BIT08  (1u << 8)
#define BIT09  (1u << 9)
#define BIT10  (1u << 10)
#define BIT11  (1u << 11)
#define BIT12  (1u << 12)
#define BIT13  (1u << 13)
#define BIT14  (1u << 14)
#define BIT15  (1u << 15)
#define BIT16  (1u << 16)
#define BIT17  (1u << 17)
#define BIT18  (1u << 18)
#define BIT19  (1u << 19)
#define BIT20  (1u << 20)
#define BIT21  (1u << 21)
#define BIT22  (1u << 22)
#define BIT23  (1u << 23)
#define BIT24  (1u << 24)
#define BIT25  (1u << 25)
#define BIT26  (1u << 26)
#define BIT27  (1u << 27)
#define BIT28  (1u << 28)
#define BIT29  (1u << 29)
#define BIT30  (1u << 30)
#define BIT31  (1u << 31)

#define SET_BIT00  (1u << 0)
#define SET_BIT01  (1u << 1)
#define SET_BIT02  (1u << 2)
#define SET_BIT03  (1u << 3)
#define SET_BIT04  (1u << 4)
#define SET_BIT05  (1u << 5)
#define SET_BIT06  (1u << 6)
#define SET_BIT07  (1u << 7)
#define SET_BIT08  (1u << 8)
#define SET_BIT09  (1u << 9)
#define SET_BIT10  (1u << 10)
#define SET_BIT11  (1u << 11)
#define SET_BIT12  (1u << 12)
#define SET_BIT13  (1u << 13)
#define SET_BIT14  (1u << 14)
#define SET_BIT15  (1u << 15)

#define CLR_BIT00  ((1u << 0) << 16)
#define CLR_BIT01  ((1u << 1) << 16)
#define CLR_BIT02  ((1u << 2) << 16)
#define CLR_BIT03  ((1u << 3) << 16)
#define CLR_BIT04  ((1u << 4) << 16)
#define CLR_BIT05  ((1u << 5) << 16)
#define CLR_BIT06  ((1u << 6) << 16)
#define CLR_BIT07  ((1u << 7) << 16)
#define CLR_BIT08  ((1u << 8) << 16)
#define CLR_BIT09  ((1u << 9) << 16)
#define CLR_BIT10  ((1u << 10) << 16)
#define CLR_BIT11  ((1u << 11) << 16)
#define CLR_BIT12  ((1u << 12) << 16)
#define CLR_BIT13  ((1u << 13) << 16)
#define CLR_BIT14  ((1u << 14) << 16)
#define CLR_BIT15  ((1u << 15) << 16)

#if defined (ARDUINO_GIGA)
    // If Arduino GIGA
    #define READ_PIN_D04            ((GPIOJ->IDR & BIT08) != 0)
    #define WRITE_PIN_D04(x)  ((x) ? (GPIOJ->ODR |= BIT08) : (GPIOJ->ODR &= ~BIT08))
    #define READ_PIN_D05            ((GPIOA->IDR & BIT07) != 0)
    #define WRITE_PIN_D05(x)  ((x) ? (GPIOA->ODR |= BIT07) : (GPIOA->ODR &= ~BIT07))
    #define READ_PIN_D06            ((GPIOD->IDR & BIT13) != 0)
    #define WRITE_PIN_D06(x)  ((x) ? (GPIOD->ODR |= BIT13) : (GPIOD->ODR &= ~BIT13))
    #define READ_PIN_D07            ((GPIOB->IDR & BIT04) != 0)
    #define WRITE_PIN_D07(x)  ((x) ? (GPIOB->ODR |= BIT04) : (GPIOB->ODR &= ~BIT04))
    #define READ_PIN_D08            ((GPIOB->IDR & BIT08) != 0)
    #define WRITE_PIN_D08(x)  ((x) ? (GPIOB->ODR |= BIT08) : (GPIOB->ODR &= ~BIT08))    
    #define READ_PIN_D09            ((GPIOB->IDR & BIT09) != 0)
    #define WRITE_PIN_D09(x)  ((x) ? (GPIOB->ODR |= BIT09) : (GPIOB->ODR &= ~BIT09))    
    #define READ_PIN_D10            ((GPIOK->IDR & BIT01) != 0)
    #define WRITE_PIN_D10(x)  ((x) ? (GPIOK->ODR |= BIT01) : (GPIOK->ODR &= ~BIT01))    
    #define READ_PIN_D11            ((GPIOJ->IDR & BIT10) != 0)
    #define WRITE_PIN_D11(x)  ((x) ? (GPIOJ->ODR |= BIT10) : (GPIOJ->ODR &= ~BIT10))    
    #define READ_PIN_D12            ((GPIOJ->IDR & BIT11) != 0)
    #define WRITE_PIN_D12(x)  ((x) ? (GPIOJ->ODR |= BIT11) : (GPIOJ->ODR &= ~BIT11))    
    #define READ_PIN_D13            ((GPIOH->IDR & BIT06) != 0)
    #define WRITE_PIN_D13(x)  ((x) ? (GPIOH->ODR |= BIT06) : (GPIOH->ODR &= ~BIT06))    
    #define READ_PIN_D14            ((GPIOG->IDR & BIT14) != 0)
    #define WRITE_PIN_D13(x)  ((x) ? (GPIOG->ODR |= BIT14) : (GPIOK->ODR &= ~BIT14))        
    #define READ_PIN_D15            ((GPIOC->IDR & BIT07) != 0)
    #define WRITE_PIN_D15(x)  ((x) ? (GPIOC->ODR |= BIT07) : (GPIOC->ODR &= ~BIT07))        
    #define READ_PIN_D16            ((GPIOH->IDR & BIT13) != 0)
    #define WRITE_PIN_D16(x)  ((x) ? (GPIOH->ODR |= BIT13) : (GPIOH->ODR &= ~BIT13))
    #define READ_PIN_D17            ((GPIOI->IDR & BIT09) != 0)
    #define WRITE_PIN_D17(x)  ((x) ? (GPIOI->ODR |= BIT09) : (GPIOI->ODR &= ~BIT09))
    #define READ_PIN_D18            ((GPIOD->IDR & BIT05) != 0)
    #define WRITE_PIN_D18(x)  ((x) ? (GPIOD->ODR |= BIT05) : (GPIOD->ODR &= ~BIT05))
    #define READ_PIN_D19            ((GPIOD->IDR & BIT06) != 0)
    #define WRITE_PIN_D19(x)  ((x) ? (GPIOD->ODR |= BIT06) : (GPIOD->ODR &= ~BIT06))
    
    #define READ_PIN_D20            ((GPIOB->IDR & BIT11) != 0)
    #define READ_PIN_D21            ((GPIOH->IDR & BIT04) != 0)
    #define READ_PIN_D22            ((GPIOJ->IDR & BIT12) != 0)
    #define READ_PIN_D23            ((GPIOG->IDR & BIT13) != 0)
    #define READ_PIN_D24            ((GPIOG->IDR & BIT12) != 0)
    #define READ_PIN_D25            ((GPIOJ->IDR & BIT00) != 0)
    #define READ_PIN_D26            ((GPIOJ->IDR & BIT14) != 0)
    #define READ_PIN_D27            ((GPIOJ->IDR & BIT01) != 0)
    #define READ_PIN_D28            ((GPIOJ->IDR & BIT15) != 0)
    #define READ_PIN_D29            ((GPIOJ->IDR & BIT02) != 0)
    #define READ_PIN_D30            ((GPIOK->IDR & BIT03) != 0)
    #define READ_PIN_D31            ((GPIOJ->IDR & BIT03) != 0)
    #define READ_PIN_D32            ((GPIOK->IDR & BIT04) != 0)
    #define READ_PIN_D33            ((GPIOJ->IDR & BIT04) != 0)
    #define READ_PIN_D34            ((GPIOK->IDR & BIT05) != 0)
    #define READ_PIN_D35            ((GPIOJ->IDR & BIT05) != 0)
    #define READ_PIN_D36            ((GPIOK->IDR & BIT06) != 0)
    #define READ_PIN_D37            ((GPIOJ->IDR & BIT06) != 0)
    #define READ_PIN_D38            ((GPIOJ->IDR & BIT07) != 0)
    #define WRITE_PIN_D38(x)  ((x) ? (GPIOJ->ODR |= BIT07) : (GPIOJ->ODR &= ~BIT07))
    #define READ_PIN_D39            ((GPIOI->IDR & BIT14) != 0)
    #define WRITE_PIN_D39(x)  ((x) ? (GPIOI->ODR |= BIT14) : (GPIOI->ODR &= ~BIT14))
    #define READ_PIN_D40            ((GPIOE->IDR & BIT06) != 0)
    #define WRITE_PIN_D40(x)  ((x) ? (GPIOE->ODR |= BIT06) : (GPIOE->ODR &= ~BIT06))
    #define READ_PIN_D41            ((GPIOK->IDR & BIT07) != 0)
    #define WRITE_PIN_D41(x)  ((x) ? (GPIOK->ODR |= BIT07) : (GPIOK->ODR &= ~BIT07))
    #define READ_PIN_D42            ((GPIOI->IDR & BIT15) != 0)
    #define WRITE_PIN_D42(x)  ((x) ? (GPIOI->ODR |= BIT15) : (GPIOI->ODR &= ~BIT15))
    #define READ_PIN_D43            ((GPIOI->IDR & BIT10) != 0)
    #define WRITE_PIN_D43(x)  ((x) ? (GPIOI->ODR |= BIT10) : (GPIOI->ODR &= ~BIT10))
    #define READ_PIN_D44            ((GPIOG->IDR & BIT10) != 0)
    #define WRITE_PIN_D44(x)  ((x) ? (GPIOG->ODR |= BIT10) : (GPIOG->ODR &= ~BIT10))
    #define READ_PIN_D45            ((GPIOI->IDR & BIT13) != 0)
    #define WRITE_PIN_D45(x)  ((x) ? (GPIOI->ODR |= BIT13) : (GPIOI->ODR &= ~BIT13))
    #define READ_PIN_D46            ((GPIOH->IDR & BIT15) != 0)
    #define WRITE_PIN_D46(x)  ((x) ? (GPIOH->ODR |= BIT15) : (GPIOH->ODR &= ~BIT15))
    #define READ_PIN_D47            ((GPIOB->IDR & BIT02) != 0)
    #define WRITE_PIN_D47(x)  ((x) ? (GPIOB->ODR |= BIT02) : (GPIOB->ODR &= ~BIT02))
    #define READ_PIN_D48            ((GPIOK->IDR & BIT00) != 0)
    #define WRITE_PIN_D48(x)  ((x) ? (GPIOK->ODR |= BIT00) : (GPIOK->ODR &= ~BIT00))
    #define READ_PIN_D49            ((GPIOE->IDR & BIT04) != 0)
    #define WRITE_PIN_D49(x)  ((x) ? (GPIOE->ODR |= BIT04) : (GPIOE->ODR &= ~BIT04))
    #define READ_PIN_D50            ((GPIOI->IDR & BIT11) != 0)
    #define WRITE_PIN_D50(x)  ((x) ? (GPIOI->ODR |= BIT11) : (GPIOI->ODR &= ~BIT11))
    #define READ_PIN_D51            ((GPIOE->IDR & BIT05) != 0)
    #define WRITE_PIN_D51(x)  ((x) ? (GPIOE->ODR |= BIT05) : (GPIOE->ODR &= ~BIT05))
    #define READ_PIN_D52            ((GPIOK->IDR & BIT02) != 0)
    #define WRITE_PIN_D52(x)  ((x) ? (GPIOK->ODR |= BIT02) : (GPIOK->ODR &= ~BIT02))
    #define READ_PIN_D53            ((GPIOG->IDR & BIT07) != 0)
    #define WRITE_PIN_D53(x)  ((x) ? (GPIOG->ODR |= BIT07) : (GPIOG->ODR &= ~BIT07))

    #define READ_PIN_A0             ((GPIOC->IDR & BIT04) != 0)
    #define WRITE_PIN_A0(x)   ((x) ? (GPIOC->ODR |= BIT04) : (GPIOC->ODR &= ~BIT04))
    #define READ_PIN_A1             ((GPIOC->IDR & BIT05) != 0)
    #define WRITE_PIN_A0(x)   ((x) ? (GPIOC->ODR |= BIT05) : (GPIOC->ODR &= ~BIT05))    
#elif defined(__SAM3X8E__) 
    // If Arduino DUE
    #define READ_PIN_D03      ((PIOC->PIO_PDSR & BIT28) != 0)
    #define READ_PIN_D06      ((PIOC->PIO_PDSR & BIT24) != 0)
    #define READ_PIN_D08      ((PIOC->PIO_PDSR & BIT22) != 0)
    #define READ_PIN_D09      ((PIOC->PIO_PDSR & BIT21) != 0)
    #define READ_PIN_D10      ((PIOC->PIO_PDSR & BIT29) != 0)
    #define READ_PIN_D11      ((PIOD->PIO_PDSR & BIT07) != 0)
    #define READ_PIN_D12      ((PIOD->PIO_PDSR & BIT08) != 0)
    #define READ_PIN_D13      ((PIOB->PIO_PDSR & BIT27) != 0)
    #define READ_PIN_D14      ((PIOD->PIO_PDSR & BIT04) != 0)
    #define READ_PIN_D15      ((PIOD->PIO_PDSR & BIT05) != 0)
    #define READ_PIN_D16      ((PIOA->PIO_PDSR & BIT13) != 0)
    #define READ_PIN_D17      ((PIOA->PIO_PDSR & BIT12) != 0)
    #define READ_PIN_D18      ((PIOA->PIO_PDSR & BIT11) != 0)
    #define READ_PIN_D19      ((PIOA->PIO_PDSR & BIT10) != 0)

    #define READ_PIN_D38      ((PIOC->PIO_PDSR & BIT06) != 0)
    #define READ_PIN_D39      ((PIOC->PIO_PDSR & BIT07) != 0)
    #define READ_PIN_D40      ((PIOC->PIO_PDSR & BIT08) != 0)

    #define READ_PIN_D43      ((PIOA->PIO_PDSR & BIT20) != 0) 
    #define READ_PIN_D44      ((PIOC->PIO_PDSR & BIT19) != 0)
    #define READ_PIN_D45      ((PIOC->PIO_PDSR & BIT18) != 0)
    #define READ_PIN_D46      ((PIOC->PIO_PDSR & BIT17) != 0)
    #define READ_PIN_D47      ((PIOC->PIO_PDSR & BIT16) != 0)
    #define READ_PIN_D48      ((PIOC->PIO_PDSR & BIT15) != 0)
    #define READ_PIN_D49      ((PIOC->PIO_PDSR & BIT14) != 0)
    #define READ_PIN_D50      ((PIOC->PIO_PDSR & BIT13) != 0)
    #define READ_PIN_D51      ((PIOC->PIO_PDSR & BIT12) != 0)
    #define READ_PIN_D52      ((PIOB->PIO_PDSR & BIT21) != 0)
    #define READ_PIN_D53      ((PIOB->PIO_PDSR & BIT14) != 0)

#elif defined(__AVR_ATmega2560__) 
    // If Arduino MEGA
    #define READ_PIN_D08  ((PINH & BIT05) != 0) // QS1 - Pin 8 (H5)
    #define READ_PIN_D09  ((PINH & BIT06) != 0) // QS0 - Pin 9 (H6)

    #define READ_PIN_D14  ((PINJ & BIT01) != 0) // S0  - Pin 14
    #define READ_PIN_D15  ((PINJ & BIT00) != 0) // S1  - Pin 15
    #define READ_PIN_D16  ((PINH & BIT01) != 0) // S2  - Pin 16 (H1)
    #define READ_PIN_D17  ((PINH & BIT00) != 0) // BHE - Pin 17 (H0)
    #define READ_PIN_D38  ((PIND & BIT07) != 0) // S3  - Pin 38 (D7)
    #define READ_PIN_D39  ((PING & BIT02) != 0) // S4  - Pin 39 (G2)
    #define READ_PIN_D40  ((PING & BIT01) != 0) // S5  - Pin 40 (G1)
    #define READ_PIN_D43  ((PINL & BIT06) != 0) // MCE/PDEN - Pin 43 (L6)
    #define READ_PIN_D44  ((PINL & BIT05) != 0) // DEN      - Pin 44 (L5)
    #define READ_PIN_A0   ((PINF & 0x01) != 0)
    #define READ_PIN_A1   ((PINF & 0x02) != 0)
    #define READ_PIN_D49  ((PINL & BIT00) != 0)
    #define READ_PIN_D50  ((PINB & 0x08) != 0)
    #define READ_PIN_D51  ((PINB & 0x04) != 0)
    #define READ_PIN_D52  ((PINB & 0x02) != 0)
    #define READ_PIN_D53  ((PINB & 0x01) != 0)
    #define READ_PIN_D46  ((PINL & 0x08) != 0)
    #define READ_PIN_D48  ((PINL & 0x02) != 0)
    #define READ_PIN_D47  ((PINL & 0x04) != 0)
    #define READ_PIN_D45  ((PINL & 0x10) != 0)

#endif

#endif // _GPIO_PINS_H