/**
  ******************************************************************************
  * @file    adc.h
  * @brief   4 cảm biến hồng ngoại đo tường: ADC1 IN4..IN7 (PA4..PA7),
  *          quét bằng DMA2 Stream0 vòng tròn, TIM4 CC4 kích ~5kHz
  *
  * LƯU Ý: không bật "Generate peripheral initialization as a pair of .c/.h files"
  * trong CubeMX - CubeMX sẽ sinh Core/Src/adc.c riêng và đè lên file này.
  ******************************************************************************
  */
#ifndef ADC_H
#define ADC_H

#include "main.h"

#define ADC_IR_CHANNELS       4           // = ADC1 NbrOfConversion trong CubeMX

// [0] = trước-trái (FL), [1] = trái (L), [2] = phải (R), [3] = trước-phải (FR)
// DMA ghi liên tục vào đây -> volatile
extern volatile uint16_t adc_value[ADC_IR_CHANNELS];

void ADC_IR_Start(void);                  // Bật DMA trước rồi mới bật TIM4 kích chuyển đổi
void ADC_IR_EmittersOn(void);             // Bật LED phát hồng ngoại

// ===== ĐỔI ADC -> KHOẢNG CÁCH TƯỜNG ===== (chưa có code)

// ===== PHÁT HIỆN TƯỜNG ===== (chưa có code)

void print_LED(void);                     // Debug: in 4 giá trị ADC

#endif /* ADC_H */
