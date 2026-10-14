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

// ===== QUY ĐỔI 2 MẮT TRƯỚC VỀ CÙNG THANG (cùng khoảng cách -> cùng giá trị) =====
// Đo 10/10/2026 (622 mẫu, đưa xe lại gần / ra xa tường phẳng, 2 mắt cùng khoảng cách): mắt trái nhạy hơn
// ~2.4-2.8 lần (xa tường: IRFL ~480 / IRFR ~160) và BÃO HÒA ~3850-3950 khi IRFR > ~1350 (sát tường).
// IRFR (không bão hòa, thang rộng hơn) làm CHUẨN; IRFL quy đổi theo bảng nội suy (adc.c) về thang IRFR.
// Sau quy đổi: lệch trung vị 1.4%, 90% mẫu < 5.2% (trước: IRFL lớn hơn IRFR ~160%).
// Bảng phải đo khi xe VUÔNG GÓC THẬT với tường (lệch 1% giữa 2 mắt ~ căn sai vài độ, xem adc.c).
// Đo lại 20:17 (thang IRFR đổi ~23%): dày mỗi 50 IRFR trong 800..1200, r TB ~0, lệch chuẩn 0.006
// IRFL bão hòa (>= IR_FL_SAT) thì IR_FrontLeft() chỉ là giới hạn dưới -> không so 2 bên được.
// Lần đo 3 (20:17): thang IRFR cao hơn ~23% -> IRFL bão hòa khi IRFR ~1750-1800, IR_FrontLeft() kẹp ở ~1820.
// Lần đo 5 (14/10): IRFL nén mạnh từ IRFR ~1720 (thô 3775), bão hòa hẳn ~3830 từ IRFR ~1850 -> kẹp ở 1870.
// Căn tường cần IRFR <= ~1650; muốn căn ở 1800-2000 phải giảm độ nhạy kênh trái (điện trở LED / thu) rồi đo lại bảng.
#define IR_FL_SAT             3800        // IRFL từ mức này trở lên coi là bão hòa
uint16_t IR_FL_ToRef(uint16_t fl_raw);    // Quy đổi 1 giá trị IRFL thô về thang IRFR
uint16_t IR_FrontLeft(void);              // IRFL hiện tại, đã quy đổi về thang IRFR
uint16_t IR_FrontRight(void);             // IRFR hiện tại (thang chuẩn)
uint8_t  IR_FrontLeftSaturated(void);     // 1: IRFL đang bão hòa (quá gần tường)

// ===== QUY ĐỔI 2 MẮT BÊN VỀ CÙNG THANG =====
// Đo 10/10/2026: tường bên đặt ở 13 khoảng cách như nhau cho mắt trái (IRL.log) và mắt phải (IRR.log).
// 2 mắt bên gần nhau sẵn (IRR/IRL = 0.91..1.09, lớn nhất ~1270, không bão hòa). IRR làm CHUẨN,
// IRL quy đổi theo bảng 13 điểm về thang IRR (đúng tại 13 điểm đo, nội suy giữa các điểm)
uint16_t IR_L_ToRef(uint16_t l_raw);      // Quy đổi 1 giá trị IRL thô về thang IRR
uint16_t IR_Left(void);                   // IRL hiện tại, đã quy đổi về thang IRR
uint16_t IR_Right(void);                  // IRR hiện tại (thang chuẩn)

// ===== IR ĐÃ LỌC (cho điều khiển theo tường, pid.c) =====
// IR_Tick() gọi mỗi 2ms trong ngắt TIM9: trung vị 3 mẫu (bỏ gai, mẫu tụt về 0) + lọc thông thấp IR_FILT_K.
// Các hàm _Filt trả về theo thang chuẩn (trước: thang IRFR, bên: thang IRR) như IR_FrontLeft()...
#define IR_FILT_K             0.3f        // Hệ số lọc mỗi 2ms (~6ms)
void  IR_Tick(void);
float IR_FL_Filt(void);
float IR_FR_Filt(void);
float IR_L_Filt(void);
float IR_R_Filt(void);

// ===== ĐỔI ADC -> KHOẢNG CÁCH TƯỜNG ===== (chưa có code)

// ===== PHÁT HIỆN TƯỜNG ===== (chưa có code)

void print_LED(void);                     // Debug: in 4 giá trị ADC

#endif /* ADC_H */
