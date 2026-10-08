/**
  ******************************************************************************
  * @file    pid.h
  * @brief   Động cơ (DRV8833 kiểu đẩy/phanh) + encoder + PID vận tốc 2 bánh + bài test PID
  ******************************************************************************
  */
#ifndef PID_H
#define PID_H

#include "main.h"

// ===== PID VẬN TỐC ĐỘNG CƠ =====  (CONTROL_DT = 2ms nằm trong main.h)
// Đường kính HIỆU DỤNG từng bánh (mm): bánh cao su bị nén dưới sức nặng xe nên lăn nhỏ hơn 30mm danh định.
// Đo ngày 09/10/2026 (6 lần Straight_Log_UART 1m): xe báo 998.8mm mà thực tế ~970mm (thiếu 3cm)
//   -> trung bình 30 x 970/998.8 = 29.13; gyro giữ thẳng mà encoder báo quay trái ~5.4 độ/m
//   -> bánh phải nhỏ hơn bánh trái ~0.72%
// Chỉnh tiếp: đi thiếu/thừa -> nhân CẢ 2 với (quãng đường thật / quãng đường xe báo);
//   lệch 2 bánh -> lấy số đề xuất ở cuối báo cáo Straight_Log_UART (trung bình vài lần chạy)
#define WHEEL_DIAMETER_L      29.24f
#define WHEEL_DIAMETER_R      29.03f
#define WHEEL_DIAMETER        (0.5f * (WHEEL_DIAMETER_L + WHEEL_DIAMETER_R))   // Trung bình (để in / tham khảo)
#define ENCODER_CPR           700.0f          // Số xung encoder / 1 vòng bánh
#define MM_PER_COUNT_L        (3.14159265f * WHEEL_DIAMETER_L / ENCODER_CPR)
#define MM_PER_COUNT_R        (3.14159265f * WHEEL_DIAMETER_R / ENCODER_CPR)
#define MM_PER_COUNT          (3.14159265f * WHEEL_DIAMETER / ENCODER_CPR)    // Trung bình
#define SPEED_LPF_ALPHA       0.3f            // Hệ số lọc thông thấp vận tốc đo (0..1, nhỏ = lọc mạnh)
#define PWM_MAX               2999            // = ARR của TIM1 và TIM5
#define MAX_SPEED_MMS         350.0f          // Giới hạn vận tốc đặt (mm/s). Đo được: 300 mm/s đã cần ~72% PWM
                                              // -> tối đa ~400 mm/s, chừa lại phần PWM cho PID tăng tốc/bù tải
// ===== TEST PID (PID_Test) =====
#define PID_TEST_STAGES       5               // Số bước thử, vận tốc từng bước ở mảng pid_test_speeds[]
#define PID_TEST_STAGE_LEN    200             // Số mẫu mỗi bước (200 x 2ms = 0.4s)
#define PID_LOG_SIZE          (PID_TEST_STAGES * PID_TEST_STAGE_LEN)  // 1000 mẫu x 12 byte = 12KB RAM
#define PID_SETTLE_BAND       20.0f           // Ngưỡng coi là đã ổn định (mm/s), lớn hơn nhiễu encoder ~±12 mm/s
#define MAX_ACCEL_MMS2        2000.0f         // Gia tốc tối đa khi đổi vận tốc đặt (mm/s^2)
// Driver kiểu IN1/IN2: GPIO nối IN1, chân PWM nối IN2
// Cả IN1 và IN2 đều là PWM trên cùng 1 timer -> mỗi chu kỳ là đẩy/phanh (slow decay) ở CẢ 2 chiều
// (CubeMX: PB0 = TIM1_CH2N, PA3 = TIM5_CH4)
#define PWM_PERIOD            (PWM_MAX + 1)   // CCR = PWM_PERIOD: chân ở mức cao 100%
#define MOTOR_L_IN1_CCR       (TIM1->CCR2)    // Trái: IN1 = PB0 (TIM1 CH2N)
#define MOTOR_L_IN2_CCR       (TIM1->CCR3)    //       IN2 = PB1 (TIM1 CH3N)
#define MOTOR_R_IN1_CCR       (TIM5->CCR4)    // Phải: IN1 = PA3 (TIM5 CH4)
#define MOTOR_R_IN2_CCR       (TIM5->CCR3)    //       IN2 = PA2 (TIM5 CH3)
// Mạch này: IN1 = 0, IN2 = PWM là chạy tới -> duty dương = chạy tới
// Đổi thành -1 nếu bánh quay lùi khi duty dương
#define MOTOR_L_SIGN          (1)
#define MOTOR_R_SIGN          (1)
// Đổi thành -1 nếu xe chạy tới mà vận tốc đo ra âm
#define ENC_L_SIGN            (1)
#define ENC_R_SIGN            (1)

// ===== VÒNG NGOÀI: ĐI THẲNG (ENCODER) + GIỮ HƯỚNG / QUAY (GYRO) =====
#define TRACK_WIDTH_MM        76.5f           // Khoảng cách giữa 2 điểm tiếp đất của 2 bánh (mm), đã đo trên xe
                                              // (chỉ ảnh hưởng tốc độ quay/sửa hướng, góc cuối cùng do gyro quyết định)
// Hướng xe = YAW_SCALE * yaw_angle, dương = quay trái. Chiều của yaw chỉnh bằng YAW_SIGN trong mpu6500.h
// (sai dấu: giữ hướng bị ngược, xe lệch hướng > MOTION_ABORT_DEG là tự dừng và báo lỗi)
// Hiệu chỉnh độ nhạy gyro (MPU6500 sai số tới +-3%: 4 lần quay 90 có thể lệch thật tới ~10 độ dù xe tưởng đúng).
// Cách chỉnh: chạy Rectangle_Test, đo góc THẬT xe đã quay tổng cộng (4 lần quay phải = 360 nếu đúng)
//   YAW_SCALE mới = YAW_SCALE cũ x (góc thật / 360). Ví dụ xe về chỗ cũ mà mũi còn lệch trái 7 độ
//   (mới quay thật 353) -> YAW_SCALE = 1.0 x 353/360 = 0.981
#define YAW_SCALE             1.0f
#define MOVE_SPEED_MMS        250.0f          // Vận tốc tối đa khi đi thẳng
#define MOVE_DECEL_MMS2       1000.0f         // Gia tốc hãm dùng để tính điểm giảm tốc (< MAX_ACCEL_MMS2 để bánh kịp bám)
#define MOVE_TOL_MM           2.0f            // Sai số quãng đường chấp nhận
// PID giữ hướng khi đi thẳng (gyro MPU6500). Ra tốc độ sửa hướng w (độ/s) -> 2 bánh +-w*TRACK_WIDTH/2
//   w = KP*lệch + KI*tích phân lệch - KD*tốc độ quay gyro
//   KI: xóa lệch hướng còn lại khi 2 bánh không đều (bánh mòn khác nhau, sàn trơn 1 bên...)
//   KD: lấy thẳng tốc độ quay từ gyro (không đạo hàm số -> không khuếch đại nhiễu), giảm dao động
// Lần 2 (09/10/2026): 6/8/0.1 -> 10/15/0.2. Xe lắc là do bánh (cú giật yaw mỗi vòng bánh ~360ms @250mm/s,
// encoder không thấy) - mô phỏng: tăng gain chỉ giảm lắc ~10%, phải sửa bánh mới hết
// Lần 3 (09/10/2026, Heading_PID_Test mô phỏng bậc +-5 độ): Ki 15 vọt lố 13-15%, ổn định ~0.8s
// -> 12/6/0.4: vọt lố <=7%, ổn định ~0.25s, vẫn bù được 2 bánh lệch 1%. (15/8/0.4 nhanh hơn, w_max ~75/90)
#define HEADING_KP            12.0f           // (độ/s) / độ lệch
#define HEADING_KI            6.0f            // (độ/s) / (độ.s)
#define HEADING_KD            0.4f            // (độ/s) / (độ/s)
// Các số trên là giá trị lúc khởi động; khi chạy dùng biến heading_kp/ki/kd (Heading_PID_Test đổi qua Bluetooth)
#define HEADING_I_MAX_DPS     30.0f           // Giới hạn phần tích phân (chống tích lũy quá mức)
#define HEADING_MAX_DPS       90.0f           // Giới hạn tốc độ sửa hướng
#define LOG_UART_PERIOD_MS    20              // Straight_Log_UART: in 1 dòng mỗi chừng này ms (~70 byte = 6ms @115200)
// Straight_IMU_Test: ghi mỗi 2ms vào RAM (18 byte/mẫu -> 3000 mẫu = 54KB, đủ 1m + 1s đứng yên trước khi chạy)
#define IMU_REC_MAX           3000
#define IMU_REC_STILL         250             // Số mẫu mỗi giai đoạn đứng yên (250 x 2ms = 0.5s)
// Heading_PID_Test: đang chạy thẳng thì đổi hướng đích theo bậc +HP_STEP, 0, -HP_STEP, 0 rồi đo đáp ứng
#define HP_STEP_DEG           5.0f            // Độ lớn bậc đổi hướng (độ). 5 độ x 1s ~ lệch ngang 2cm
#define HP_T0_MS              500             // Bậc đầu tiên sau khi xuất phát (đã chạy đều)
#define HP_STEP_MS            1000            // Mỗi bậc giữ chừng này ms -> cần chạy >= 4.5s (~1.2m @250mm/s)
#define HP_SETTLE_DEG         0.5f            // |lệch| dưới mức này coi là đã bám đích
#define HP_RAW_DECIM          5               // In dữ liệu thô: 1 dòng / 5 mẫu (10ms)
#define HEAD_REC_MAX          5400            // 10 byte/mẫu, dùng chung bộ nhớ với Straight_IMU_Test (10.8s)
#define TURN_SPEED_DPS        300.0f          // Tốc độ quay tối đa khi quay tại chỗ (độ/s)
#define TURN_DECEL_DPS2       1500.0f         // Gia tốc góc hãm dùng để tính điểm giảm tốc khi quay
#define TURN_TOL_DEG          1.0f            // Sai số góc quay chấp nhận
#define MOTION_STOP_MMS       10.0f           // 2 bánh chậm hơn mức này ...
#define MOTION_SETTLE_MS      60              // ... liên tục chừng này ms (và đã tới đích) thì coi là xong 1 đoạn
#define MOTION_ABORT_DEG      30.0f           // Lệch hướng vượt quá mức này (sai YAW_SIGN, bị va/kẹt) -> dừng
#define MOTION_TIMEOUT_MS     5000            // Mỗi đoạn tối đa: chừng này ms + 2 lần thời gian chạy hết đoạn ở tốc độ tối đa

// Bộ PID (dùng cho vòng vận tốc động cơ)
typedef struct {
    float Ks;            // Feedforward bù ma sát: PWM tối thiểu để bánh bắt đầu quay (cộng theo chiều)
    float Kf;            // Feedforward: PWM ước lượng cho 1 mm/s vận tốc đặt
    float Ka;            // Feedforward gia tốc: PWM cho 1 mm/s^2 (bù quán tính, = Kf * hằng số thời gian động cơ)
    float Kp;
    float Ki;
    float Kd;
    float integral;      // Tổng tích phân đã nhân Ki (đơn vị PWM)
    float prev_measure;  // Giá trị đo lần trước, dùng cho khâu D
    float prev_setpoint; // Vận tốc đặt lần trước, dùng tính gia tốc cho Ka
    float out_min;
    float out_max;
} PID_TypeDef;
// 1 mẫu ghi đáp ứng bước (mm/s và PWM, làm tròn thành số nguyên)
typedef struct {
    int16_t ref_l, meas_l, pwm_l;
    int16_t ref_r, meas_r, pwm_r;
} PIDLog_TypeDef;

extern PID_TypeDef pid_speed_left, pid_speed_right;
extern uint8_t motor_hw_ok;
extern volatile float target_speed_left, target_speed_right;
extern float ref_speed_left, ref_speed_right;
extern volatile uint8_t motor_pid_enable;
extern float speed_left_mms, speed_right_mms;
extern int32_t pwm_left, pwm_right;

// ===== KHỞI ĐỘNG (luồng chính) =====
void Motor_Init(void);                    // Bật PWM 4 kênh + encoder, kiểm tra chân, phanh 2 động cơ

// ===== ĐIỀU KHIỂN (luồng chính) =====
void Motor_SetSpeed(float left_mms, float right_mms);
void Motor_Stop(void);
void Motor_SetPWM(int32_t left, int32_t right);   // Duty trực tiếp (test tay khi PID tắt)

// ===== GỌI TRONG NGẮT =====
void Motor_Speed_Tick(void);              // Ngắt TIM9 mỗi 2ms: đọc encoder + chạy PID vận tốc

// ===== PID =====
float PID_Compute(PID_TypeDef *pid, float setpoint, float measure, float dt);
void PID_Reset(PID_TypeDef *pid, float measure);
float Ramp(float current, float target, float max_step);

// ===== VÒNG NGOÀI: ĐI ĐÚNG QUÃNG ĐƯỜNG + GIỮ HƯỚNG / QUAY BẰNG GYRO =====
// Các hàm dưới CHỜ tới khi xe tới đích và dừng hẳn (gọi ở luồng chính, không gọi trong ngắt).
// Trả về 1: xong, 0: bị hủy (lệch hướng quá MOTION_ABORT_DEG / quá MOTION_TIMEOUT_MS / IMU, động cơ chưa sẵn sàng)
extern float dist_left_mm, dist_right_mm; // Quãng đường cộng dồn từ vận tốc encoder (mm)
extern float odo_x_mm, odo_y_mm;          // Vị trí ước lượng (encoder + gyro), gốc = lúc gọi Motion_Begin()
extern float enc_heading_deg;             // Góc quay tính CHỈ từ encoder = (quãng phải - quãng trái) / TRACK_WIDTH (độ)
extern float heading_kp, heading_ki, heading_kd;   // PID giữ hướng đang dùng (khởi đầu = HEADING_KP/KI/KD)
void    Motion_Begin(void);               // Lấy vị trí + hướng hiện tại làm gốc (xe đứng yên)
uint8_t Move_Straight(float dist_mm);     // Đi thẳng dist_mm (âm = lùi), giữ hướng bằng gyro
uint8_t Turn_Angle(float deg);            // Quay tại chỗ: dương = trái, âm = phải
uint8_t Turn_Left(float deg);
uint8_t Turn_Right(float deg);
float   Robot_Heading(void);              // Hướng xe (độ), dương = trái, không quấn về +-180

// ===== TEST / DEBUG (luồng chính) =====
void PID_Test(uint8_t print_raw);
void print_SpeedPID(void);
// Đi hình chữ nhật: thẳng length -> quay phải 90 -> thẳng width -> quay phải ... (4 cạnh) về chỗ cũ, rồi in báo cáo
void Rectangle_Test(float length_mm, float width_mm);
// Đi thẳng dist_mm (âm = lùi), giữ hướng bằng gyro, rồi in báo cáo (quãng đường từng bánh, lệch hướng, lệch ngang)
void Straight_Test(float dist_mm);
// Đi thẳng dist_mm, giữ hướng bằng PID gyro, VỪA CHẠY VỪA IN qua USART1 (HC-05) mỗi LOG_UART_PERIOD_MS:
// góc yaw gyro so với góc tính từ encoder, tốc độ quay gyro so với encoder. Chờ nhận 1 ký tự qua UART mới chạy.
void Straight_Log_UART(float dist_mm);
// Đi thẳng dist_mm, ghi TOÀN BỘ dữ liệu MPU (gyro + accel thô) + encoder + PID mỗi 2ms vào RAM, KHÔNG gửi UART
// lúc xe chạy. Trước khi chạy: 0.5s đứng yên im lặng + 0.5s đứng yên gửi UART liên tục (kiểm tra UART/Bluetooth
// có làm nhiễu encoder / IMU không). Đo chu kỳ + thời gian xử lý ngắt TIM9. Xong in qua UART: nhiễu từng giai đoạn,
// tần số rung nổi bật; dump_raw = 1 in thêm từng mẫu (9600 baud mất ~2 phút, 115200 ~10s)
void Straight_IMU_Test(float dist_mm, uint8_t dump_raw);
// TEST PID GIỮ HƯỚNG (gyro MPU6500): qua Bluetooth gửi "Kp Ki Kd" (vd: 10 15 0.2) hoặc 1 ký tự (giữ hệ số
// hiện tại) -> xe chỉnh lại offset gyro, chạy thẳng dist_mm, giữa chừng đổi hướng đích +5, 0, -5, 0 độ.
// In thời gian lên, vọt lố, thời gian ổn định, sai số xác lập từng bậc + gyro so với encoder. Cần dist >= 1200.
// print_raw = 1: in thêm dữ liệu mỗi 10ms để vẽ đồ thị
void Heading_PID_Test(float dist_mm, uint8_t print_raw);

#endif /* PID_H */
