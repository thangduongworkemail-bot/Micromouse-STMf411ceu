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

// ===== QUY ĐỔI 2 MẮT TRƯỚC VỀ CÙNG THANG (xem adc.h) =====
// Bảng IRFL thô -> IRFR cùng khoảng cách. Đổi linh kiện / chỉnh độ nhạy LED thì phải đo lại bảng.
// BẢNG PHẢI ĐO KHI XE VUÔNG GÓC THẬT với tường: 2 mắt chỉ cần lệch nhau ~1% là căn tường sai vài độ.
// Lần đo 3 (data.log 10/10/2026 20:17): ở cùng IRFL, IRFR cao hơn lần 1-2 ~23% (thang IRFR đổi: phần cứng /
// hướng mắt phải-trước?) -> bảng cũ cho FLq thấp ~20% (r ~ -0.1). Log có 2 tư thế: dòng ~345..468 FR/FL cao hơn
// ~3.5% (IRR cũng cao hơn -> xe xoay trái hơn) -> BỎ; dùng phần còn lại (305 mẫu, đầu + giữa + cuối log).
//   IRFR 800..900: tỉ lệ ~0.463 như dải trên (lần 4: trong mỗi lượt đẩy tỉ lệ gần như không đổi từ ~750 tới 1200)
//   IRFR 950..1200: DÀY mỗi 50 (vùng căn tường), nút = IRFR / tỉ lệ IRFR/IRFL trung vị quanh nút
//     -> IRFR 800..1200: r TB +0.0003, lệch chuẩn 0.006 (1 mẫu, 100 mẫu); đoạn tư thế bị bỏ đọc r ~ -0.009
//   IRFR 1300, 1500, 3772 -> 1802: thưa (trên 1200 chỉ để lùi khỏi tường; IRFL bắt đầu nén gần bão hòa)
// Lần đo 4 (data.log 14/10/2026 15:27): 2 lần ĐẶT YÊN khớp bảng: IRFR 1001 (42 mẫu) r -0.001, IRFR 1441 (23 mẫu)
//   r +0.003 -> GIỮ dải 800..1802. Các lượt ĐẨY TỚI / LÙI lệch nghiêng (FL sáng hơn ~10% ở CẢ 2 chiều, mỗi lượt
//   khác nhau 0.385..0.435) -> không dùng cho dải 800..1200. Dải DƯỚI 800 sai thật (FLq cao hơn IRFR 15..40%, càng
//   xa càng nhiều: IRFL có mức nền ~150) -> dựng lại 130..650 từ các lượt đẩy, trừ lệch nghiêng ước lượng
//   r = 0.043 x sqrt(IRFR/1000) (so lượt đẩy với đặt yên ở 900..1500): lượt đẩy dải xa còn lệch +0.02 = đúng phần nghiêng
// Lần đo 5 (data.log 14/10/2026 16:03, dải 1300..2000): IRFL GẦN BÃO HÒA thì bị nén (FR tăng nhanh hơn FL) -> đoạn thẳng
//   cũ 3357 -> 3772 nằm trên đường cong thật: đặt yên IRFR 1595 / 1700 / 1738 đọc r +0.019 / +0.030 / +0.017. Thay bằng
//   3574 -> 1595, 3775 -> 1720 (gộp 1700 + 1738), 3795 -> 1840 (lượt đẩy: đoạn "gối" ~5 IRFR / 1 đếm IRFL) -> 3 lần đặt
//   r 0.000 / +0.008 / -0.007; đặt yên lần 3 ở 3503 -> 1563 cũng thành 0.000. Dưới 1500 GIỮ (khớp đặt yên lần 3-4 ở
//   1294 / 1345 / 1441); lần đặt IRFR 1354 (r +0.056) mâu thuẫn với chúng và với chính lượt đẩy lần 5 -> bỏ.
//   GIỚI HẠN: IRFR > ~1720 thì IRFL vào "gối" (3775..3800), > ~1850 bão hòa hẳn (~3830) -> căn tường ở IRFR <= ~1650.
static const uint16_t ir_fl_raw[] = {  426,  524,  672,  861, 1080, 1281, 1497, 1724, 1834, 1945,
                                      2054, 2175, 2311, 2417, 2511, 2636, 2857, 3357, 3574, 3775, 3795 };
static const uint16_t ir_fl_ref[] = {  130,  175,  250,  350,  450,  550,  650,  800,  850,  900,
                                       950, 1000, 1050, 1100, 1150, 1200, 1300, 1500, 1595, 1720, 1840 };
#define IR_FL_TAB_N  (sizeof(ir_fl_raw) / sizeof(ir_fl_raw[0]))

// Bảng IRL thô -> IRR cùng khoảng cách: tường bên đặt ở 13 khoảng cách như nhau cho từng mắt (IRL.log, IRR.log
// 10/10/2026), mỗi điểm = trung vị đoạn đứng yên cuối cùng ở khoảng cách đó (12-123 mẫu, lệch chuẩn 2-16).
// 2 mắt bên vốn gần nhau (IRR/IRL = 0.91..1.09). Không thấy bão hòa (lớn nhất ~1270).
static const uint16_t ir_l_raw[] = {  231,  273,  312,  376,  394,  443,  509,  621,  735,  864, 1009, 1166, 1268 };
static const uint16_t ir_l_ref[] = {  300,  360,  408,  432,  506,  582,  556,  642,  722,  820,  929, 1060, 1179 };
#define IR_L_TAB_N   (sizeof(ir_l_raw) / sizeof(ir_l_raw[0]))

// Nội suy tuyến tính từng đoạn trong bảng x -> y (x tăng dần, n >= 2).
// Dưới bảng (xa hơn điểm xa nhất đã đo): tỉ lệ y0/x0. Trên bảng: kéo dài đoạn cuối, tối đa 4095
static uint16_t IR_Interp(const uint16_t *x, const uint16_t *y, uint32_t n, uint16_t raw)
{
    if (raw <= x[0]) return (uint16_t)((uint32_t)raw * y[0] / x[0]);
    uint32_t i = 1;
    while (i < n - 1 && raw > x[i]) i++;              // Đoạn [i-1, i] chứa raw (cuối bảng thì kéo dài đoạn cuối)
    int32_t v = y[i - 1] + ((int32_t)y[i] - y[i - 1]) * ((int32_t)raw - x[i - 1]) / ((int32_t)x[i] - x[i - 1]);
    if (v > 4095) v = 4095;
    return (uint16_t)v;
}
uint16_t IR_FL_ToRef(uint16_t fl_raw)
{
    if (fl_raw > IR_FL_SAT) fl_raw = IR_FL_SAT;       // Bão hòa: không biết gần hơn bao nhiêu
    return IR_Interp(ir_fl_raw, ir_fl_ref, IR_FL_TAB_N, fl_raw);
}
uint16_t IR_L_ToRef(uint16_t l_raw)  { return IR_Interp(ir_l_raw, ir_l_ref, IR_L_TAB_N, l_raw); }

// ===== LỌC NHIỄU IR (cho điều khiển theo tường) =====
// Mỗi 2ms (ngắt TIM9, pid.c gọi IR_Tick): trung vị 3 mẫu gần nhất (bỏ gai / mẫu tụt về 0) rồi lọc thông thấp
// IR_FILT_K (~6ms). Trả về theo thang chuẩn: IRFL, IRL đã quy đổi theo bảng (trễ ~6-8ms, nhiễu còn ~1/3)
static uint16_t ir_hist[ADC_IR_CHANNELS][3];
static uint8_t  ir_hist_i = 0;
static float    ir_filt[ADC_IR_CHANNELS];
static uint8_t  ir_filt_ok = 0;
static uint16_t Median3(uint16_t a, uint16_t b, uint16_t c)
{
    if (a > b) { uint16_t t = a; a = b; b = t; }
    if (b > c) b = c;
    return (a > b) ? a : b;
}
void IR_Tick(void)
{
    for (int c = 0; c < ADC_IR_CHANNELS; c++) {
        uint16_t v = adc_value[c];
        if (!ir_filt_ok) {                            // Lần đầu: điền đầy lịch sử
            ir_hist[c][0] = ir_hist[c][1] = ir_hist[c][2] = v;
            ir_filt[c] = v;
        }
        ir_hist[c][ir_hist_i] = v;
        float m = Median3(ir_hist[c][0], ir_hist[c][1], ir_hist[c][2]);
        ir_filt[c] += IR_FILT_K * (m - ir_filt[c]);
    }
    ir_hist_i = (uint8_t)((ir_hist_i + 1) % 3);
    ir_filt_ok = 1;
}
float IR_FL_Filt(void) { return IR_FL_ToRef((uint16_t)ir_filt[0]); }
float IR_FR_Filt(void) { return ir_filt[3]; }
float IR_L_Filt(void)  { return IR_L_ToRef((uint16_t)ir_filt[1]); }
float IR_R_Filt(void)  { return ir_filt[2]; }

// ===== ĐỔI ADC -> KHOẢNG CÁCH TƯỜNG ===== (chưa có code)

// ===== PHÁT HIỆN TƯỜNG ===== (chưa có code)

// ===== DEBUG =====
//Hàm test LED
void print_LED(){
	      // Sử dụng hàm CDC_Print của bạn để tránh mất gói tin khi USB Busy
	      // FLq: IRFL đã quy đổi về thang IRFR (cùng khoảng cách thì FLq ~ IRFR), "BH" = IRFL bão hòa
	      // Lq : IRL đã quy đổi về thang IRR (cùng khoảng cách thì Lq ~ IRR)
	      // r  : lệch 2 mắt trước (FL - FR) / (FL + FR) đã lọc - đặt xe vuông góc THẬT với tường ở ~WALL_TARGET_IR,
	      //      giá trị này chính là WALL_R_OFFSET (pid.h)
	      uint16_t fl = adc_value[0], l = adc_value[1], r = adc_value[2], fr = adc_value[3];
	      float ffl = IR_FL_Filt(), ffr = IR_FR_Filt();
	      sprintf(msg, "IRFL:%u | IRL:%u | IRR:%u | IRFR:%u | FLq:%u%s | Lq:%u | r:%+.3f\r\n",
	              fl, l, r, fr, IR_FL_ToRef(fl), (fl >= IR_FL_SAT) ? " BH" : "", IR_L_ToRef(l),
	              (ffl + ffr > 1.0f) ? (ffl - ffr) / (ffl + ffr) : 0.0f);
	        UART_Print(msg);
		  HAL_Delay(20);
}
