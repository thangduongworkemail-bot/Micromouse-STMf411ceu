/**
  ******************************************************************************
  * @file    adc.c
  * @brief   4 cảm biến hồng ngoại đo tường
  *
  * Phần cứng (CubeMX): ADC1 quét IN4..IN7, mỗi lần TIM4 CC4 (~5kHz) kích 1 lượt 4 kênh,
  * DMA2 Stream0 vòng tròn chép kết quả vào adc_value[] -> CPU không phải làm gì,
  * đọc adc_value[] lúc nào cũng được giá trị mới nhất.
  ******************************************************************************
  */
#include "adc.h"
#include <stdio.h>          // sprintf

extern ADC_HandleTypeDef hadc1;           // main.c (CubeMX)
extern TIM_HandleTypeDef htim4;

// Thêm từ khóa volatile để báo cho CPU biết mảng này bị thay đổi ngầm bởi DMA
volatile uint16_t adc_value[ADC_IR_CHANNELS];

static char msg[200];                     // Bộ đệm in USB của module này

// ===== KHỞI ĐỘNG =====
// Kích hoạt ADC ở chế độ DMA TRƯỚC để nó nằm chờ tín hiệu, rồi mới bật TIM4 phát xung kích.
// Ngược lại (TIM4 trước): xung kích có thể rơi vào lúc ADC đã bật mà DMA chưa bật
// -> ADC tràn (OVR), DMA ngừng nhận yêu cầu và adc_value[] đứng im mãi.
void ADC_IR_Start(void)
{
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_value, ADC_IR_CHANNELS);
    // Kích Hoạt Tim4 nguồn trig cho DMA ADC 5khz
    // TIM 4 CH4 no output generator PWWM
    HAL_TIM_Base_Start(&htim4);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
}
// Bật tất cả LED phát (Chỉ để test, nếu làm mạch thật cần băm xung theo Timer)
// (PA3 là IN1 động cơ phải nên không bật ở đây)
void ADC_IR_EmittersOn(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_SET);
}

// ===== ĐỔI ADC -> KHOẢNG CÁCH TƯỜNG ===== (chưa có code)

// ===== PHÁT HIỆN TƯỜNG ===== (chưa có code)

// ===== DEBUG =====
//Hàm test LED
void print_LED(){
	      // Sử dụng hàm CDC_Print của bạn để tránh mất gói tin khi USB Busy
	      sprintf(msg, "IRFL:%u | IRL:%u | IRR:%u | IRFR:%u\r\n",
	              adc_value[0], adc_value[1], adc_value[2], adc_value[3]);

	      CDC_Print(msg);
		  HAL_Delay(20);
}
