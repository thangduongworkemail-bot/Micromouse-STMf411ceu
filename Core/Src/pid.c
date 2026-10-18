/**
  ******************************************************************************
  * @file    pid.c
  * @brief   Động cơ + encoder + PID vận tốc 2 bánh + bài test PID
  *
  * Phần cứng (CubeMX):
  *   Trái : PWM TIM1 CH2N (PB0 = IN1) + CH3N (PB1 = IN2), encoder TIM3 (PB4/PB5)
  *   Phải : PWM TIM5 CH4  (PA3 = IN1) + CH3  (PA2 = IN2), encoder TIM2 (PA0/PA1)
  * Luồng chạy:
  *   main: Motor_Init() -> Motor_SetSpeed(...) bất cứ lúc nào
  *   ngắt TIM9 (2ms): Motor_Speed_Tick() -> đọc encoder -> quãng đường -> vòng ngoài (Motion_Update)
  *                    -> ramp -> PID -> Motor_SetPWM
  ******************************************************************************
  */
#include "pid.h"
#include "mpu6500.h"        // yaw_angle, imu_ready cho giữ hướng / quay
#include "adc.h"            // adc_value[]: IR trước làm mốc cho Gyro_Spin_Calibrate
#include <math.h>           // sqrtf, fabsf, sinf, cosf
#include <stdio.h>          // snprintf

#define DEG2RAD               (3.14159265f / 180.0f)

extern TIM_HandleTypeDef htim1;           // main.c (CubeMX)
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;

// proces Data CNT encoder
static uint16_t prev_cnt_left = 0;
static uint16_t prev_cnt_right = 0;
//
// PID VẬN TỐC
// Feedforward PWM = Ks (theo chiều) + Kf * vận tốc đặt + Ka * gia tốc đặt
// Lấy từ PID_Test kiểu đẩy/phanh (6 lần chạy, PWM khi đã ổn định):
//   trái: ~832 @155, ~1058 @200, ~1523 @297 mm/s -> Ks ~ 80, Kf ~ 4.85
//   phải: ~852 @155, ~1070 @200, ~1555 @297 mm/s -> Ks ~ 85, Kf ~ 4.95  (chiều lùi đối xứng)
//   hằng số thời gian động cơ ~35 ms (mô phỏng khớp với báo cáo đo) -> Ka = Kf * 0.035 ~ 0.17
//   (bánh đứng yên cần ~450-500 PWM mới quay; Ka lúc bắt đầu tăng tốc đã bù phần này)
//   Khâu I bù phần sai lệch còn lại (pin yếu, tải nặng...). Kp, Ki: chỉnh tiếp bằng PID_Test()
// Bánh nhỏ hơn thì cùng PWM cho ra ít mm/s hơn -> Kf, Ka, Kp, Ki tỉ lệ nghịch với đường kính bánh
// Các số trên đo khi còn tính bánh 30mm; nay dùng đường kính hiệu dụng (pid.h) -> Kf, Ka nhân 30/D để
// PWM ra vẫn như cũ: trái 30/29.24, phải 30/29.03 (Kp, Ki lệch ~3% không đáng kể, giữ nguyên)
// Kp, Ki (10/10/2026, 8 lần PID_Test với 1.5/30): bánh phải bám tốt mọi lần (RMS ~6.5 mm/s = nhiễu lượng tử),
// bánh TRÁI mỗi vòng bánh (~92mm) có 1 chỗ KẸT CƠ KHÍ: tụt còn 25-45% vận tốc dù PWM thừa 500-1000 so với
// feedforward (RMS 25-44 mm/s). Kp 1.5 / Ki 30 quá chậm (băng thông ~6 rad/s) -> chỗ kẹt qua rồi mới bù xong.
// 5/150: Ki/Kp = 30 ~ 1/tau động cơ (35ms) -> vòng kín ~ bậc 1, tau ~ 35ms. Mô phỏng khớp data:
//   bánh trái kẹt: RMS 34 -> 21 mm/s, tụt thấp nhất 23% -> 63% vận tốc đặt; bánh tốt: không xấu đi
//   (vẫn ổn khi tau 25-50ms và trễ thêm 2 chu kỳ). Đổi lại PWM rung theo nhiễu encoder ~12 -> ~43 mỗi 2ms
//   (~1.4% duty, động cơ lọc mất). Chỗ kẹt phải sửa cơ khí, PID chỉ giảm được ảnh hưởng.
// Đã sửa cơ khí bánh trái, đo lại 24 lần với 5/150: cả 2 bánh RMS 6.1-6.6 mm/s (chỉ còn nhiễu lượng tử),
//   0 mẫu lệch > 35 mm/s, sai số xác lập |TB| < 2.6 mm/s, Bam_max 18-32, 2 bánh như nhau -> giữ 5/150.
PID_TypeDef pid_speed_left  = { .Ks = 80.0f, .Kf = 4.98f, .Ka = 0.174f, .Kp = 5.0f, .Ki = 150.0f, .Kd = 0.0f, .out_min = -PWM_MAX, .out_max = PWM_MAX };
PID_TypeDef pid_speed_right = { .Ks = 85.0f, .Kf = 5.12f, .Ka = 0.176f, .Kp = 5.0f, .Ki = 150.0f, .Kd = 0.0f, .out_min = -PWM_MAX, .out_max = PWM_MAX };
uint8_t motor_hw_ok = 0;                   // 1: PB0/PA3 đã là chân PWM (CubeMX), 0: khóa động cơ
volatile float target_speed_left  = 0.0f;  // Vận tốc mong muốn (mm/s), đặt bằng Motor_SetSpeed()
volatile float target_speed_right = 0.0f;
float ref_speed_left  = 0.0f;              // Vận tốc đặt sau bộ tăng/giảm tốc dần, đưa vào PID
float ref_speed_right = 0.0f;
volatile uint8_t motor_pid_enable = 0;     // 1: chạy vòng PID, 0: tắt PID và thả động cơ
float speed_left_mms  = 0.0f;              // Vận tốc đo đã lọc (mm/s)
float speed_right_mms = 0.0f;
int32_t pwm_left = 0, pwm_right = 0;       // Duty đang xuất (-PWM_MAX..PWM_MAX)
// Ghi đáp ứng cho PID_Test(): ngắt TIM9 ghi 1 mẫu / 2ms khi pid_log_count < PID_LOG_SIZE
static PIDLog_TypeDef pid_log[PID_LOG_SIZE];
static volatile uint16_t pid_log_count = PID_LOG_SIZE;  // = PID_LOG_SIZE: không ghi
// Vận tốc đặt của từng bước thử (mm/s): tăng tốc, tăng khi đang chạy, giảm, đảo chiều, dừng
static const float pid_test_speeds[PID_TEST_STAGES] = { 200.0f, 300.0f, 150.0f, -150.0f, 0.0f };

// Vòng ngoài (ngắt TIM9 ghi, luồng chính đọc)
float dist_left_mm = 0.0f, dist_right_mm = 0.0f;
float odo_x_mm = 0.0f, odo_y_mm = 0.0f;
// MOTION_HOLD: đứng giữ hướng + lùi khỏi tường quá gần + tự căn vuông góc với tường trước (Hold_Update)
typedef enum { MOTION_IDLE, MOTION_STRAIGHT, MOTION_TURN, MOTION_HOLD } MotionMode;
// MOTION_WALL: đi thẳng bị tường trước chặn (ngắt đặt) / đã căn xong (Wall_Align đặt)
typedef enum { MOTION_OK, MOTION_RUNNING, MOTION_ABORT, MOTION_TIMEOUT, MOTION_NOT_READY, MOTION_LIFTED,
               MOTION_WALL, MOTION_WALL_FAIL } MotionResult;
static volatile MotionMode   motion_mode   = MOTION_IDLE;
static volatile MotionResult motion_result = MOTION_OK;
static volatile float heading_target   = 0.0f;   // Hướng cần giữ / cần quay tới (độ, TUYỆT ĐỐI -> sai số không cộng dồn)
static volatile float move_start_mm    = 0.0f;   // Quãng đường tâm xe lúc bắt đầu đoạn đi thẳng
static volatile float move_target_mm   = 0.0f;
static volatile float move_speed_mms   = MOVE_SPEED_MMS;   // Tốc độ tối đa của đoạn đi thẳng đang chạy
static volatile uint8_t move_wall      = 0;      // 1: đoạn đi thẳng này dừng trước tường + tránh tường bên
static volatile float motion_abort_deg = MOTION_ABORT_DEG;
static volatile float motion_max_err   = 0.0f;   // Báo cáo: đi thẳng = |lệch hướng| lớn nhất, quay = góc vượt quá đích lớn nhất
static volatile float motion_turn_dir  = 1.0f;   // Chiều quay đang chạy: 1 trái, -1 phải
uint8_t wall_guard = 1;
uint8_t wall_center = 1;
static float wall_center_e = 0.0f;               // Lệch ngang tương đối đã lọc (dương = lệch trái, xem WALL_CENTER_)
static float wall_gain = 0.0f;                  // Độ nhạy căn tường g (r / độ) đã đo, 0 = chưa đo (xem pid.h)
static float wall_gain_ir = 0.0f;                // TB 2 mắt trước lúc đo g (g đổi theo khoảng cách)
static float heading_rate_f = 0.0f;              // Tốc độ quay (gyro) đã lọc cho khâu D (độ/s, dương = trái)
static volatile uint8_t hold_active = 0;         // Đứng giữ hướng: 1 = đang quay về hướng đích
static uint8_t heading_set = 0;                  // 0: chưa lấy hướng gốc
float yaw_scale = YAW_SCALE;                     // Độ nhạy gyro đang dùng (Gyro_Spin_Calibrate sửa lúc chạy)

// Ghi dữ liệu cho Gyro_Spin_Calibrate (ngắt TIM9 ghi 1 mẫu / SPIN_CAL_DECIM chu kỳ khi spin_rec_n < SPIN_CAL_MAX)
typedef struct {
    float    h;                           // Hướng theo gyro (độ, độ nhạy CŨ)
    float    dd;                          // Quãng đường bánh phải - bánh trái (mm)
    uint16_t ir;                          // IR trước-trái + trước-phải (ADC)
} SpinRec;
static SpinRec spin_rec[SPIN_CAL_MAX];
static volatile uint16_t spin_rec_n = SPIN_CAL_MAX;   // = SPIN_CAL_MAX: không ghi
static uint8_t spin_rec_div = 0;

static char msg[200];                      // Bộ đệm in USB của module này

// ===== ĐIỀU KHIỂN ĐỘNG CƠ =====
// Kiểm tra CubeMX đã đổi PB0 -> TIM1_CH2N, PA3 -> TIM5_CH4 chưa.
// Chưa đổi mà vẫn xuất PWM kiểu mới thì IN1 kẹt ở 0, IN2 = 100% -> xe chạy tới hết tốc!
uint8_t Motor_PinsConfigured(void)
{
    uint8_t pb0_ok = (((GPIOB->MODER  >> (0 * 2)) & 0x3) == 0x2)            // PB0 ở chế độ AF
                  && (((GPIOB->AFR[0] >> (0 * 4)) & 0xF) == GPIO_AF1_TIM1);
    uint8_t pa3_ok = (((GPIOA->MODER  >> (3 * 2)) & 0x3) == 0x2)            // PA3 ở chế độ AF
                  && (((GPIOA->AFR[0] >> (3 * 4)) & 0xF) == GPIO_AF2_TIM5);
    return pb0_ok && pa3_ok;
}
// Xuất duty có dấu (-PWM_MAX..PWM_MAX) cho 1 động cơ, driver kiểu IN1/IN2, cả 2 chân là PWM:
//   duty >= 0: IN2 = 1, IN1 = 0 trong |duty| -> chạy tới (0,1) / phanh (1,1)
//   duty <  0: IN1 = 1, IN2 = 0 trong |duty| -> chạy lùi (1,0) / phanh (1,1)
//   duty =  0: phanh (1,1)
// Ghi chân lên 100% trước để lúc đổi chiều, trạng thái trung gian luôn là phanh (1,1)
void Motor_Write(volatile uint32_t *ccr_in1, volatile uint32_t *ccr_in2, int32_t duty)
{
    if (!motor_hw_ok) {                     // Chưa đổi chân trong CubeMX: giữ IN2 = 0 -> thả trôi
        *ccr_in1 = 0;
        *ccr_in2 = 0;
        return;
    }
    if (duty > PWM_MAX)  duty = PWM_MAX;
    if (duty < -PWM_MAX) duty = -PWM_MAX;

    if (duty >= 0) {
        *ccr_in2 = PWM_PERIOD;
        *ccr_in1 = (uint32_t)(PWM_PERIOD - duty);
    } else {
        *ccr_in1 = PWM_PERIOD;
        *ccr_in2 = (uint32_t)(PWM_PERIOD + duty);  // = PWM_PERIOD - |duty|
    }
}
// Xuất duty cho cả 2 động cơ, dương = chạy tới, 0 = phanh. Xe đang bị nhấc: luôn phanh
void Motor_SetPWM(int32_t left, int32_t right)
{
    if (robot_lifted) left = right = 0;
    pwm_left  = left;
    pwm_right = right;
    Motor_Write(&MOTOR_L_IN1_CCR, &MOTOR_L_IN2_CCR, MOTOR_L_SIGN * left);
    Motor_Write(&MOTOR_R_IN1_CCR, &MOTOR_R_IN2_CCR, MOTOR_R_SIGN * right);
}
// Bật PWM 4 kênh (20kHz) + 2 encoder, kiểm tra chân, rồi phanh 2 động cơ.
// Gọi 1 lần trong main TRƯỚC khi bật ngắt TIM9.
void Motor_Init(void)
{
    // Timer 1 (Base đếm) 20kHz cho bên trái
    HAL_TIM_Base_Start(&htim1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);   // IN1 trái  (PB0)
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);   // IN2 trái  (PB1)
    // Timer 5 (Base đếm) 20kHz cho bên phải
    HAL_TIM_Base_Start(&htim5);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);      // IN1 phải (PA3)
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);      // IN2 phải (PA2)
    // IN1, IN2 cùng timer và để preload CCR (mặc định CubeMX) -> 2 chân đổi duty cùng lúc ở cuối chu kỳ PWM
    // Chỉ cho chạy động cơ khi CubeMX đã đổi PB0/PA3 sang chân PWM
    motor_hw_ok = Motor_PinsConfigured();
    // Kích Hoạt TIMER cho ENCODER: TIM2 bên phải, TIM3 bên trái
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    prev_cnt_left  = TIM3->CNT;
    prev_cnt_right = TIM2->CNT;
    //Tắt Động Cơ: IN1 = 1, IN2 = 1 -> phanh (chưa đổi chân trong CubeMX thì thả trôi)
    Motor_SetPWM(0, 0);
}

// ===== PID =====
float PID_Compute(PID_TypeDef *pid, float setpoint, float measure, float dt)
{
    float error = setpoint - measure;

    // Khâu D lấy theo giá trị đo thay vì sai số -> không bị giật khi đổi vận tốc đặt
    float derivative = -(measure - pid->prev_measure) / dt;
    pid->prev_measure = measure;

    // Feedforward lo phần lớn PWM ngay từ đầu, PID chỉ bù sai lệch -> bám vận tốc đặt nhanh hơn
    //   Ks: vượt ma sát theo chiều quay, Kf: tỉ lệ với vận tốc, Ka: tỉ lệ với gia tốc (động cơ có quán tính)
    float accel = (setpoint - pid->prev_setpoint) / dt;
    pid->prev_setpoint = setpoint;
    float feedforward = pid->Kf * setpoint + pid->Ka * accel;
    if (setpoint > 0.0f) feedforward += pid->Ks;
    if (setpoint < 0.0f) feedforward -= pid->Ks;
    float integral_new = pid->integral + pid->Ki * error * dt;
    float out = feedforward + pid->Kp * error + integral_new + pid->Kd * derivative;

    // Anti-windup: output bão hòa mà sai số còn đẩy tiếp cùng chiều thì không cộng dồn khâu I
    if (out > pid->out_max) {
        out = pid->out_max;
        if (error < 0.0f) pid->integral = integral_new;
    } else if (out < pid->out_min) {
        out = pid->out_min;
        if (error > 0.0f) pid->integral = integral_new;
    } else {
        pid->integral = integral_new;
    }
    return out;
}
void PID_Reset(PID_TypeDef *pid, float measure)
{
    pid->integral = 0.0f;
    pid->prev_measure = measure;
    pid->prev_setpoint = 0.0f;
}
// PID cho 1 bánh. Vận tốc đặt = 0 thì dừng hẳn và xả tích phân để xe không tự bò
int32_t Wheel_PID(PID_TypeDef *pid, float target, float measure)
{
    if (target == 0.0f) {
        PID_Reset(pid, measure);
        return 0;
    }
    return (int32_t)PID_Compute(pid, target, measure, CONTROL_DT);
}
// Đưa dần current về target, mỗi lần đổi tối đa max_step
float Ramp(float current, float target, float max_step)
{
    if (target > current + max_step) return current + max_step;
    if (target < current - max_step) return current - max_step;
    return target;
}
static void Motion_Update(void);
// Vòng PID vận tốc, gọi trong ngắt TIM9 mỗi 2ms với số xung encoder vừa đếm được
void Motor_Speed_Control(int16_t delta_left, int16_t delta_right)
{
    static uint8_t pid_running = 0;

    // 1. Đổi xung -> mm/s rồi lọc thông thấp
    //    (1 xung trong 2ms ~ 67 mm/s, chưa lọc thì PID bị nhiễu lượng tử rất lớn)
    float raw_left  = ENC_L_SIGN * delta_left  * MM_PER_COUNT_L / CONTROL_DT;
    float raw_right = ENC_R_SIGN * delta_right * MM_PER_COUNT_R / CONTROL_DT;
    speed_left_mms  += SPEED_LPF_ALPHA * (raw_left  - speed_left_mms);
    speed_right_mms += SPEED_LPF_ALPHA * (raw_right - speed_right_mms);

    // 1b. Quãng đường = cộng dồn vận tốc encoder x 2ms. Dùng vận tốc CHƯA lọc: tổng đúng bằng
    //     số xung x mm/xung, không bị trễ của bộ lọc (vận tốc đã lọc làm quãng đường hụt ~1.4mm khi chạy 300mm/s)
    dist_left_mm  += raw_left  * CONTROL_DT;
    dist_right_mm += raw_right * CONTROL_DT;
    float d_center = 0.5f * (raw_left + raw_right) * CONTROL_DT;
    if (d_center != 0.0f) {               // Vị trí ước lượng: tâm xe đi d_center theo hướng gyro
        float h = Robot_Heading() * DEG2RAD;
        odo_x_mm += d_center * cosf(h);
        odo_y_mm += d_center * sinf(h);
    }

    // 1c. Lọc nhiễu IR (vòng ngoài dùng để dừng trước tường / tránh tường bên / căn tường)
    IR_Tick();

    // 1d. Xe bị nhấc mà vẫn còn lệnh chạy (luồng chính vừa bật lại đúng lúc nhấc): tắt lại
    if (robot_lifted && (motor_pid_enable || motion_mode != MOTION_IDLE)) Motion_OnLifted();

    // 1e. Vòng ngoài đang đi thẳng / quay thì nó đặt vận tốc cho 2 bánh
    Motion_Update();

    // 1f. Gyro_Spin_Calibrate đang ghi: hướng gyro + chênh quãng đường 2 bánh + IR trước
    if (spin_rec_n < SPIN_CAL_MAX && ++spin_rec_div >= SPIN_CAL_DECIM) {
        spin_rec_div = 0;
        SpinRec *r = &spin_rec[spin_rec_n];
        r->h  = Robot_Heading();
        r->dd = dist_right_mm - dist_left_mm;
        r->ir = adc_value[0] + adc_value[3];
        spin_rec_n++;
    }

    // 2. PID tắt: thả động cơ đúng 1 lần (để main vẫn test PWM tay được), giữ PID sạch
    if (!motor_pid_enable) {
        if (pid_running) {
            Motor_SetPWM(0, 0);
            pid_running = 0;
        }
        ref_speed_left  = 0.0f;
        ref_speed_right = 0.0f;
        PID_Reset(&pid_speed_left,  speed_left_mms);
        PID_Reset(&pid_speed_right, speed_right_mms);
        return;
    }
    pid_running = 1;

    // 3. Vận tốc đặt tăng/giảm dần theo MAX_ACCEL_MMS2 (tránh trượt bánh và bão hòa PWM)
    ref_speed_left  = Ramp(ref_speed_left,  target_speed_left,  MAX_ACCEL_MMS2 * CONTROL_DT);
    ref_speed_right = Ramp(ref_speed_right, target_speed_right, MAX_ACCEL_MMS2 * CONTROL_DT);

    // 4. Tính PID và xuất PWM
    Motor_SetPWM(Wheel_PID(&pid_speed_left,  ref_speed_left,  speed_left_mms),
                 Wheel_PID(&pid_speed_right, ref_speed_right, speed_right_mms));

    // 5. Đang chạy PID_Test() thì ghi lại mẫu này
    if (pid_log_count < PID_LOG_SIZE) {
        PIDLog_TypeDef *s = &pid_log[pid_log_count];
        s->ref_l  = (int16_t)ref_speed_left;
        s->meas_l = (int16_t)speed_left_mms;
        s->pwm_l  = (int16_t)pwm_left;
        s->ref_r  = (int16_t)ref_speed_right;
        s->meas_r = (int16_t)speed_right_mms;
        s->pwm_r  = (int16_t)pwm_right;
        pid_log_count++;
    }
}
// Ngắt TIM9 mỗi 2ms: đếm xung encoder vừa chạy được rồi chạy vòng PID vận tốc
void Motor_Speed_Tick(void)
{
    uint16_t curr_cnt_left = TIM3->CNT;
    uint16_t curr_cnt_right = TIM2->CNT;

    int16_t delta_pulse_left = (int16_t)(curr_cnt_left - prev_cnt_left);
    int16_t delta_pulse_right = (int16_t)(curr_cnt_right - prev_cnt_right);

    prev_cnt_left = curr_cnt_left;
    prev_cnt_right = curr_cnt_right;

    Motor_Speed_Control(delta_pulse_left, delta_pulse_right);
}
// ĐẶT VẬN TỐC MONG MUỐN cho 2 bánh (mm/s, dương = chạy tới, âm = chạy lùi)
// Gọi lúc nào cũng được; xe tăng/giảm tốc dần tới giá trị mới. Đặt (0, 0) để giảm tốc rồi dừng.
// Hủy đoạn Move_Straight / Turn_Angle đang chạy (nếu có).
void Motor_SetSpeed(float left_mms, float right_mms)
{
    if (left_mms  >  MAX_SPEED_MMS) left_mms  =  MAX_SPEED_MMS;
    if (left_mms  < -MAX_SPEED_MMS) left_mms  = -MAX_SPEED_MMS;
    if (right_mms >  MAX_SPEED_MMS) right_mms =  MAX_SPEED_MMS;
    if (right_mms < -MAX_SPEED_MMS) right_mms = -MAX_SPEED_MMS;

    motion_mode        = MOTION_IDLE;          // Trước khi ghi vận tốc: ngắt không ghi đè lại nữa
    if (robot_lifted) {                        // Xe đang bị nhấc: không cho chạy
        Motor_Stop();
        return;
    }
    target_speed_left  = left_mms;
    target_speed_right = right_mms;
    motor_pid_enable   = 1;
}
// Dừng ngay: tắt PID và phanh 2 động cơ (không giảm tốc dần)
void Motor_Stop(void)
{
    motion_mode        = MOTION_IDLE;
    motor_pid_enable   = 0;
    target_speed_left  = 0.0f;
    target_speed_right = 0.0f;
    Motor_SetPWM(0, 0);
}

// ===== VÒNG NGOÀI: ĐI ĐÚNG QUÃNG ĐƯỜNG + GIỮ HƯỚNG / QUAY BẰNG GYRO =====
// Chạy trong ngắt TIM9 ngay trước PID vận tốc, ra vận tốc đặt cho 2 bánh:
//   đi thẳng : v theo quãng đường còn lại (giảm tốc vừa kịp dừng đúng chỗ) +- phần sửa hướng theo gyro
//   quay     : 2 bánh ngược chiều, tốc độ quay theo góc còn lại
// Hướng đích là TUYỆT ĐỐI (cộng dồn từ Motion_Begin): góc quay lệch 1 độ thì đoạn đi thẳng sau tự sửa,
// lần quay sau vẫn nhắm đúng góc gốc -> sai số không cộng dồn qua các góc của hình chữ nhật.
float Robot_Heading(void)
{
    return yaw_scale * yaw_angle;
}
static float Robot_Distance(void)         // Quãng đường tâm xe (mm)
{
    return 0.5f * (dist_left_mm + dist_right_mm);
}
static float Clamp(float x, float limit)
{
    if (x >  limit) return  limit;
    if (x < -limit) return -limit;
    return x;
}
// Vận tốc theo khoảng còn lại: tối đa v_max, gần đích giảm theo v = sqrt(2 * a * s) để vừa dừng đúng đích
// (còn lại âm = đã vượt qua -> chạy ngược lại)
static float Motion_Profile(float remaining, float v_max, float decel)
{
    float v = sqrtf(2.0f * decel * fabsf(remaining));
    if (v > v_max) v = v_max;
    return (remaining >= 0.0f) ? v : -v;
}
// Vận tốc tối đa khi thấy tường trước (IR front, thang IRFR): từ move_speed_mms ở WALL_SLOW_IR giảm tới
// WALL_APPROACH_MMS ở WALL_TARGET_IR. Nội suy v^2 theo 1/sqrt(IR) (~ khoảng cách nếu IR ~ 1/d^2) -> hãm gần đều
static float Wall_SpeedLimit(float front)
{
    float v_max = move_speed_mms, v_min = WALL_APPROACH_MMS;
    if (front <= WALL_SLOW_IR || v_max <= v_min) return v_max;
    float u_t = 1.0f / sqrtf(WALL_TARGET_IR), u_s = 1.0f / sqrtf(WALL_SLOW_IR);
    float frac = (1.0f / sqrtf(front) - u_t) / (u_s - u_t);   // 1: ở WALL_SLOW_IR, 0: ở WALL_TARGET_IR
    if (frac < 0.0f) frac = 0.0f;
    return sqrtf(v_min * v_min + (v_max * v_max - v_min * v_min) * frac);
}
// Cân giữa ô (đi thẳng tới, xem WALL_CENTER_ ở pid.h): góc lái thêm (độ, dương = sang trái) từ 2 mắt bên.
// v: vận tốc đặt hiện tại, front: mắt trước gần hơn. Không có tường bên / tường trước quá gần: 0 (chỉ giữ hướng gyro)
static float Wall_CenterOffset(float v, float front)
{
    float l = IR_L_Filt(), r = IR_R_Filt();
    float l0 = IR_L_ToRef((uint16_t)WALL_CENTER_L_IR), r0 = WALL_CENTER_R_IR;   // Cùng thang với IR_L_Filt / IR_R_Filt
    uint8_t has_l = l >= IR_L_ToRef((uint16_t)WALL_SIDE_PRESENT_IR);           // Ngưỡng theo IRL thô của print_LED
    uint8_t has_r = r >= WALL_SIDE_PRESENT_IR;
    if (front > WALL_CENTER_FRONT_IR || (!has_l && !has_r)) {
        wall_center_e = 0.0f;
        return 0.0f;
    }
    float el = has_l ? 1.0f - sqrtf(l0 / l) : 0.0f;    // Lệch trái theo tường trái
    float er = has_r ? sqrtf(r0 / r) - 1.0f : 0.0f;    // Lệch trái theo tường phải
    float e  = (has_l && has_r) ? 0.5f * (el + er) : el + er;
    wall_center_e += (CONTROL_DT / (WALL_CENTER_TAU_S + CONTROL_DT)) * (e - wall_center_e);
    float k_v = fminf(v / WALL_CENTER_FULL_MMS, 1.0f);   // Chậm thì lái ít (đứng yên: 0)
    return Clamp(-WALL_CENTER_KP * wall_center_e, WALL_CENTER_MAX_DEG) * k_v;
}
// ===== ĐỨNG GIỮ HƯỚNG + TƯỜNG TRƯỚC (MOTION_HOLD, chạy trong ngắt TIM9 mỗi 2ms) =====
// Đứng yên giữ hướng (xoay tay thì quay về). Có wall_guard:
//   - mắt trước > WALL_STOP_IR: lùi chậm (vẫn giữ hướng) tới dưới WALL_TARGET_IR -> luôn IR < WALL_STOP_IR
//   - CÓ TƯỜNG = cả 2 mắt trước >= WALL_PRESENT_IR (600). Xe đứng yên: còn xa thì tiến chậm tới ~WALL_TARGET_IR,
//     dừng, tự căn vuông góc, rồi đo lại offset gyro Z và ĐẶT LẠI HƯỚNG MPU: hướng vuông góc với tường = hướng đang
//     giữ (ws_href)
// Căn tường (mô hình gần vuông góc: lệch r = (FL - FR)/(FL + FR) = g x (hướng - hướng vuông góc)):
//   WS_IDLE  chờ đứng yên WALL_SETTLE_MS. Mắt gần hơn > đích + WALL_DIST_TOL_IR: lùi về đích; < đích - WALL_DIST_START_IR:
//            tiến tới đích (phanh sớm theo tốc độ tăng IR như khi đi thẳng). Đúng khoảng cách thì đo
//   WS_MEAS  đo r trung bình WALL_MEAS_N mẫu. g chưa biết: quay thử WALL_PROBE_DEG, đo lại -> g (cả chiều).
//            Còn lệch -r/g: đủ nhỏ -> WS_ZERO, chưa -> quay chỉnh (WS_TURN), đo lại, cập nhật g theo cát tuyến
//   WS_TURN  quay chỉnh nhỏ: ít nhất HOLD_MIN_DPS, phanh sớm bù đoạn trôi, vượt đích là dừng (không quay ngược)
//   WS_ZERO  đo lại offset gyro Z (như MPU6500_ZeroGyroZ, không chặn) rồi đặt hướng -> WS_DONE
//   WS_DONE  đã vuông góc. Bị xoay tay / tường lệch > WALL_REALIGN_DEG / mất tường -> WS_IDLE (căn lại khi có tường)
//   WS_FAIL  không căn được: giữ hướng cũ, MPU không đổi. Thử lại khi mất tường rồi có lại
typedef enum { WS_IDLE, WS_MEAS, WS_TURN, WS_ZERO, WS_DONE, WS_FAIL } WallState;
static volatile WallState ws = WS_IDLE;
static volatile uint8_t ws_force = 0;     // Wall_Align đang chờ: không thấy tường -> WS_FAIL (thay vì chờ mãi)
static uint8_t  ws_backing = 0;           // Đang lùi ra xa tường
static uint8_t  ws_approach = 0;          // Đang tiến lại gần tường (tới WALL_TARGET_IR)
static float    ws_appr_mm;               // Quãng đường lúc bắt đầu tiến (giới hạn WALL_APPROACH_MAX_MM)
static uint16_t ws_still = 0;             // Số chu kỳ 2 bánh đứng yên liên tục
static uint16_t ws_busy = 0;              // Số chu kỳ đang căn (có tường, chưa xong / hỏng), giới hạn WALL_ALIGN_MAX_MS
static uint16_t ws_n = 0;                 // Đếm mẫu / chu kỳ trong WS_MEAS, WS_TURN, WS_ZERO
static float    ws_fl, ws_fr;             // Tổng IR khi đo
static float    ws_r, ws_h;               // Lần đo trước: lệch r, hướng gyro
static float    ws_h0;                    // Hướng lúc bắt đầu căn (giới hạn WALL_ALIGN_MAX_DEG)
static float    ws_href;                  // Hướng đang giữ = hướng sẽ gán cho pháp tuyến tường
static float    ws_hperp;                 // Hướng gyro lúc vuông góc (hệ trước khi đặt lại)
static float    ws_prev_step, ws_dir, ws_rf;
static uint8_t  ws_it, ws_probe, ws_latched;
static uint32_t wz_last;                  // WS_ZERO: imu_sample_count lần trước
static float    wz_sum, wz_sq, wz_half;

static void Wall_Reset(void)              // Gọi khi bắt đầu MOTION_HOLD (ngắt chưa chạy chế độ này)
{
    ws          = WS_IDLE;
    ws_backing  = 0;
    ws_approach = 0;
    ws_still    = 0;
    ws_busy     = 0;
}
static void Wall_Fail(void)
{
    wall_gain      = 0.0f;                // Lần sau đo lại g
    heading_target = ws_href;             // Về hướng đang giữ trước khi căn
    ws = WS_FAIL;
}
static void Wall_TurnTo(float h)
{
    heading_target = h;
    ws_dir     = (h >= Robot_Heading()) ? 1.0f : -1.0f;
    ws_latched = 0;
    ws_n       = 0;
    ws         = WS_TURN;
}
static void Wall_StartMeas(void)
{
    ws_fl = ws_fr = 0.0f;
    ws_n  = 0;
    ws    = WS_MEAS;
}
// Hướng vuông góc với tường = ws_href. Xe còn lệch (Robot_Heading - ws_hperp) so với vuông góc (< WALL_ALIGN_ACCEPT_DEG)
// -> giữ phần lệch đó. yaw_angle chỉ bị ghi trong ngắt (cùng mức ưu tiên với ngắt I2C) -> không cần tắt ngắt
static void Wall_SetHeading(void)
{
    yaw_angle      = (ws_href + Robot_Heading() - ws_hperp) / yaw_scale;
    heading_target = ws_href;
    ws = WS_DONE;
}
// Đo xong WALL_MEAS_N mẫu: quay thử / quay chỉnh / xong
static void Wall_Evaluate(void)
{
    float fl = ws_fl / ws_n, fr = ws_fr / ws_n, h = Robot_Heading();
    if (fminf(fl, fr) < WALL_PRESENT_IR) {
        Wall_Fail();
        return;
    }
    float r = (fl - fr) / (fl + fr) - WALL_R_OFFSET;
    if (wall_gain == 0.0f) {
        if (!ws_probe) {                  // Chưa biết g: quay thử, gyro đo góc đã quay thật
            ws_r = r;
            ws_h = h;
            ws_probe = 1;
            Wall_TurnTo(h + WALL_PROBE_DEG);
            return;
        }
        ws_probe = 0;
        float g = (fabsf(h - ws_h) >= 0.5f * WALL_PROBE_DEG) ? (r - ws_r) / (h - ws_h) : 0.0f;
        if (fabsf(g) < WALL_MIN_GAIN) {   // Không quay được / IR gần như không đổi khi quay: không phải tường phẳng
            Wall_Fail();
            return;
        }
        wall_gain    = g;
        wall_gain_ir = 0.5f * (fl + fr);
    } else if (ws_it > 0 && fabsf(h - ws_h) > 1.0f) {
        // Bước quay đủ lớn: cập nhật g theo cát tuyến, chỉ nhận giá trị cùng chiều và không quá khác (nhiễu)
        float g = (r - ws_r) / (h - ws_h);
        if (g * wall_gain > 0.0f && fabsf(g) > 0.5f * fabsf(wall_gain) && fabsf(g) < 2.0f * fabsf(wall_gain)) {
            wall_gain    = g;
            wall_gain_ir = 0.5f * (fl + fr);
        }
    }
    float step = -r / wall_gain;          // Còn phải quay (độ)
    // Xong: lệch < WALL_ALIGN_TOL_DEG, hoặc lần quay vừa rồi đã vượt qua vuông góc (đổi chiều) mà còn lệch
    // < WALL_ALIGN_ACCEPT_DEG: bánh cần ~470 PWM mới nhích nên lần quay nhỏ nhất ~1.5 độ, quay tiếp chỉ vọt qua lại
    if (fabsf(step) < WALL_ALIGN_TOL_DEG || (fabsf(step) < WALL_ALIGN_ACCEPT_DEG && step * ws_prev_step < 0.0f)) {
        ws_hperp = h + step;
        ws_rf    = r;
#if WALL_REZERO_GYRO
        wz_sum = wz_sq = wz_half = 0.0f;
        wz_last = imu_sample_count;
        ws_n = 0;
        ws   = WS_ZERO;
#else
        Wall_SetHeading();
#endif
        return;
    }
    if (ws_it >= WALL_ALIGN_ITERS || fabsf(h + step - ws_h0) > WALL_ALIGN_MAX_DEG) {
        Wall_Fail();
        return;
    }
    ws_prev_step = step;
    ws_r = r;
    ws_h = h;
    ws_it++;
    Wall_TurnTo(h + step);
}
// Đo lại offset gyro Z không chặn: TB GYRO_REZERO_SAMPLES mẫu thô (kiểm tra nhiễu + nửa đầu/nửa sau như
// MPU6500_ZeroGyroZ; bị chạm / xoay chậm thì giữ offset cũ), rồi đặt hướng
static void Wall_ZeroTick(void)
{
    if (imu_sample_count == wz_last) return;   // Chờ mẫu IMU mới
    wz_last = imu_sample_count;
    float g = imu_raw[6];
    wz_sum += g;
    wz_sq  += g * g;
    if (++ws_n == GYRO_REZERO_SAMPLES / 2) wz_half = wz_sum;
    if (ws_n < GYRO_REZERO_SAMPLES) return;
    float mean  = wz_sum / ws_n, var = wz_sq / ws_n - mean * mean;
    float drift = (wz_half - (wz_sum - wz_half)) / (GYRO_REZERO_SAMPLES / 2) / 16.4f;
    if (!(var > 0.0f && sqrtf(var) / 16.4f > GYRO_CAL_MAX_STD_DPS) && fabsf(drift) <= GYRO_CAL_MAX_DRIFT_DPS)
        gyro_z_offset = mean;
    Wall_SetHeading();
}
// MOTION_HOLD mỗi 2ms: ra vận tốc tiến v + tốc độ quay w. head_err, w_pd: lệch hướng + PD giữ hướng,
// front_rate: tốc độ tăng IR mắt trước gần hơn (/s) (Motion_Update)
static void Hold_Update(float head_err, float w_pd, float fl, float fr, float front_rate, float *v, float *w)
{
    float front   = fmaxf(fl, fr);
    uint8_t wall  = wall_guard && fminf(fl, fr) >= WALL_PRESENT_IR;
    uint8_t still = fabsf(speed_left_mms) < MOTION_STOP_MMS && fabsf(speed_right_mms) < MOTION_STOP_MMS;
    if (!still) ws_still = 0;
    else if (ws_still < 0xFFFF) ws_still++;

    // 0. Giới hạn thời gian căn: có tường (hoặc Wall_Align đang chờ) mà quá WALL_ALIGN_MAX_MS chưa xong -> thôi căn.
    //    Chưa quay chỉnh (WS_IDLE: đang tiến / lùi / chờ đứng yên) thì hướng đích chưa bị đổi -> chỉ báo hỏng
    if ((wall || ws_force) && ws != WS_DONE && ws != WS_FAIL) {
        if (++ws_busy > WALL_ALIGN_MAX_MS / 2) {
            ws_approach = 0;
            ws_backing  = 0;                  // Quá gần (> WALL_STOP_IR) thì bước 1 vẫn bật lùi lại ở chu kỳ sau
            if (ws == WS_IDLE) ws = WS_FAIL;
            else Wall_Fail();
        }
    } else {
        ws_busy = 0;
    }

    // 1. Quá gần tường trước: lùi chậm, giữ hướng, tới dưới WALL_TARGET_IR thì thôi. Xong thì căn lại
    if (wall_guard && front > WALL_STOP_IR) ws_backing = 1;
    if (ws_backing) {
        if (!wall_guard || front < WALL_TARGET_IR) {
            ws_backing = 0;
        } else {
            *v = -WALL_CREEP_MMS;
            *w = Clamp(w_pd, HEADING_MAX_DPS);
            if (ws != WS_FAIL) ws = WS_IDLE;
            ws_approach = 0;
            return;
        }
    }
    // 1b. Tường còn xa: tiến chậm, giữ hướng, sắp tới WALL_TARGET_IR (tính cả đoạn trôi khi phanh) thì dừng.
    //     Mất tường thì thôi; đi quá WALL_APPROACH_MAX_MM mà chưa tới (tường lùi theo?) thì thôi căn
    if (ws_approach) {
        if (!wall || front + fmaxf(front_rate, 0.0f) * WALL_STOP_LEAD_S >= WALL_TARGET_IR) {
            ws_approach = 0;
        } else if (fabsf(Robot_Distance() - ws_appr_mm) > WALL_APPROACH_MAX_MM) {
            ws_approach = 0;
            Wall_Fail();
        } else {
            *v = WALL_CREEP_MMS;
            *w = Clamp(w_pd, HEADING_MAX_DPS);
            return;
        }
    }
    // 2. Đang quay chỉnh để căn tường: phanh SỚM một đoạn = tốc độ quay x WALL_TURN_COAST_S (đoạn trôi khi phanh),
    //    vào đó hoặc đã vượt đích là dừng hẳn. Đứng yên rồi thì lấy hướng thật làm đích (góc thật do gyro đo) và đo
    if (ws == WS_TURN) {
        if (++ws_n > WALL_TURN_TIMEOUT_MS / 2) {             // Kẹt bánh: thôi căn
            Wall_Fail();
            return;
        }
        if (fabsf(head_err) < WALL_TURN_TOL_DEG + fabsf(heading_rate_f) * WALL_TURN_COAST_S || head_err * ws_dir < 0.0f)
            ws_latched = 1;
        if (!ws_latched) {
            *w = Motion_Profile(head_err, TURN_SPEED_DPS, TURN_DECEL_DPS2);
            if (fabsf(*w) < HOLD_MIN_DPS) *w = (head_err > 0.0f) ? HOLD_MIN_DPS : -HOLD_MIN_DPS;
        } else if (ws_still >= WALL_SETTLE_MS / 2) {
            heading_target = Robot_Heading();
            Wall_StartMeas();
        }
        return;
    }
    // 3. Giữ hướng: lệch nhỏ thì phanh đứng yên, lệch > HOLD_START_DEG (xoay tay, va) thì quay về (ít nhất
    //    HOLD_MIN_DPS để thắng ma sát tĩnh), về trong HOLD_TOL_DEG thì phanh. Bị xoay thì căn tường lại sau đó
    float a = fabsf(head_err);
    if (a > HOLD_START_DEG) hold_active = 1;
    else if (a < HOLD_TOL_DEG) hold_active = 0;
    if (hold_active) {
        *w = w_pd;
        if (*w * head_err > 0.0f && fabsf(*w) < HOLD_MIN_DPS)     // Đang quay về: không chậm hơn mức tối thiểu
            *w = (head_err > 0.0f) ? HOLD_MIN_DPS : -HOLD_MIN_DPS;
        *w = Clamp(*w, HOLD_MAX_DPS);
        if (ws != WS_FAIL) ws = WS_IDLE;
        return;
    }
    // 4. Đứng yên: căn tường trước
    if (!wall_guard) return;
    float r = (fl + fr > 1.0f) ? (fl - fr) / (fl + fr) - WALL_R_OFFSET : 0.0f;
    switch (ws) {
    case WS_IDLE:
        if (ws_still < WALL_SETTLE_MS / 2) break;
        ws_href = heading_target;
        if (!wall) {
            if (ws_force) Wall_Fail();
            break;
        }
        if (front > WALL_TARGET_IR + WALL_DIST_TOL_IR) {    // Quá đích: lùi về đích trước (quay căn đưa 1 mắt lại gần ~4%)
            ws_backing = 1;
            break;
        }
        if (front < WALL_TARGET_IR - WALL_DIST_START_IR) {  // Còn xa: tiến tới đích trước
            ws_approach = 1;
            ws_appr_mm  = Robot_Distance();
            break;
        }
        // g đo ở khoảng cách khác nhiều (TB 2 mắt lệch > 30%) thì đo lại: g đổi theo khoảng cách, mắt gắn chụm vào
        // còn có thể đổi chiều
        if (wall_gain != 0.0f && fabsf(0.5f * (fl + fr) - wall_gain_ir) > 0.3f * wall_gain_ir) wall_gain = 0.0f;
        ws_h0 = Robot_Heading();
        ws_it = 0;
        ws_probe = 0;
        ws_prev_step = 0.0f;
        Wall_StartMeas();
        break;
    case WS_MEAS:
        if (!still) {
            ws = WS_IDLE;
            break;
        }
        ws_fl += fl;
        ws_fr += fr;
        if (++ws_n >= WALL_MEAS_N) Wall_Evaluate();
        break;
    case WS_ZERO:
        if (!still) ws = WS_IDLE;
        else Wall_ZeroTick();
        break;
    case WS_DONE:                         // Theo dõi: tường bị xoay đi / dời xa gần / mất tường thì căn lại
        if (!wall) {
            ws = WS_IDLE;
            break;
        }
        ws_rf += 0.02f * (r - ws_rf);     // ~0.1s
        if (wall_gain == 0.0f || fabsf(ws_rf / wall_gain) > WALL_REALIGN_DEG
            || fabsf(0.5f * (fl + fr) - wall_gain_ir) > 0.3f * wall_gain_ir) ws = WS_IDLE;
        break;
    case WS_FAIL:
        if (!wall) ws = WS_IDLE;
        break;
    default:
        break;
    }
}
static void Motion_Update(void)
{
    static uint16_t settle_ticks = 0;
    // Tốc độ quay đã lọc cho khâu D (lọc cả lúc không chạy -> bắt đầu đoạn nào cũng đã sẵn giá trị đúng).
    // Lọc để nhiễu rung gyro không làm vận tốc đặt 2 bánh nhảy mỗi 2ms (Ka của PID vận tốc nhân lên thành giật)
    heading_rate_f += (CONTROL_DT / (HEADING_D_TAU_S + CONTROL_DT)) * (yaw_scale * yaw_rate_dps - heading_rate_f);
    // Mắt trước gần tường hơn (thang IRFR) + tốc độ tăng đã lọc (/s), cho dừng trước tường
    static float front_prev = 0.0f, front_rate = 0.0f;
    float ir_fl = IR_FL_Filt(), ir_fr = IR_FR_Filt();
    float front = fmaxf(ir_fl, ir_fr);
    front_rate += (CONTROL_DT / (WALL_RATE_TAU_S + CONTROL_DT)) * ((front - front_prev) / CONTROL_DT - front_rate);
    front_prev  = front;
    if (motion_mode == MOTION_IDLE) {
        settle_ticks = 0;
        wall_center_e = 0.0f;
        return;
    }

    float head_err = heading_target - Robot_Heading();   // Dương: cần quay sang trái
    float v = 0.0f, w_dps = 0.0f;
    uint8_t arrived;

    if (motion_mode == MOTION_HOLD) {
        // Bị xoay quá nửa vòng: quay về theo đường ngắn nhất. Dời đích đúng bội 360 độ (vẫn cùng hướng),
        // giữ hướng đích liên tục cho các lệnh Turn_Angle / Move_Straight sau
        while (head_err >  180.0f) { heading_target -= 360.0f; head_err -= 360.0f; }
        while (head_err < -180.0f) { heading_target += 360.0f; head_err += 360.0f; }
    }
    float err_stat = (motion_mode == MOTION_TURN) ? -motion_turn_dir * head_err : fabsf(head_err);
    if (err_stat > motion_max_err) motion_max_err = err_stat;
    if (motion_mode != MOTION_HOLD && fabsf(head_err) > motion_abort_deg) {   // Sai YAW_SIGN / bị va, kẹt: dừng
        motion_mode   = MOTION_IDLE;
        motion_result = MOTION_ABORT;
        target_speed_left = target_speed_right = 0.0f;
        return;
    }
    // PD giữ hướng (xem pid.h)
    float w_pd = HEADING_KP * head_err - HEADING_KD * heading_rate_f;
    if (motion_mode == MOTION_STRAIGHT) {
        float remaining = move_target_mm - (Robot_Distance() - move_start_mm);
        arrived = fabsf(remaining) < MOVE_TOL_MM;
        if (!arrived) {
            v = Motion_Profile(remaining, move_speed_mms, MOVE_DECEL_MMS2);
            if (move_wall && v > 0.0f) {
                // Tường trước: sắp tới WALL_TARGET_IR (tính cả đoạn trôi khi phanh) thì dừng (PID giảm tốc theo
                // MAX_ACCEL rồi phanh), luồng chính căn tường
                if (front + fmaxf(front_rate, 0.0f) * WALL_STOP_LEAD_S >= WALL_TARGET_IR) {
                    motion_mode   = MOTION_IDLE;
                    motion_result = MOTION_WALL;
                    target_speed_left = target_speed_right = 0.0f;
                    return;
                }
                v = fminf(v, Wall_SpeedLimit(front));
                // Tường bên quá gần: cộng thêm lệch hướng ra xa (chỉ vào lệnh lái, không vào báo cáo / hủy)
                float side = WALL_SIDE_DEG_PER_IR * (fmaxf(IR_R_Filt() - WALL_SIDE_IR, 0.0f)
                                                   - fmaxf(IR_L_Filt() - WALL_SIDE_IR, 0.0f));
                w_pd += HEADING_KP * Clamp(side, WALL_SIDE_MAX_DEG);
            }
            // Cân giữa ô theo tường bên: cũng chỉ cộng vào lệnh lái, hướng đích gyro giữ nguyên
            if (wall_center && v > 0.0f) w_pd += HEADING_KP * Wall_CenterOffset(v, front);
            else wall_center_e = 0.0f;
            w_dps = Clamp(w_pd, HEADING_MAX_DPS);
        }
    } else if (motion_mode == MOTION_HOLD) {
        // Đứng giữ hướng + tường trước (Hold_Update). Không bao giờ xong: giữ tới lệnh chuyển động khác
        arrived = 0;
        Hold_Update(head_err, w_pd, ir_fl, ir_fr, front_rate, &v, &w_dps);
    } else {
        arrived = fabsf(head_err) < TURN_TOL_DEG;
        if (!arrived) w_dps = Motion_Profile(head_err, TURN_SPEED_DPS, TURN_DECEL_DPS2);
    }

    // Quay trái (w > 0): bánh phải nhanh hơn bánh trái. Tới đích thì đặt đúng 0 -> PID phanh hẳn
    float dv = w_dps * DEG2RAD * TRACK_WIDTH_MM * 0.5f;
    target_speed_left  = Clamp(v - dv, MAX_SPEED_MMS);
    target_speed_right = Clamp(v + dv, MAX_SPEED_MMS);

    // Xong: đã tới đích và 2 bánh đứng yên liên tục MOTION_SETTLE_MS (dừng mà trôi quá đích thì sửa tiếp)
    if (arrived && fabsf(speed_left_mms) < MOTION_STOP_MMS && fabsf(speed_right_mms) < MOTION_STOP_MMS) {
        if (++settle_ticks >= MOTION_SETTLE_MS / 2) {
            motion_mode   = MOTION_IDLE;
            motion_result = MOTION_OK;
        }
    } else {
        settle_ticks = 0;
    }
}
// Lấy vị trí + hướng hiện tại làm gốc. Gọi lúc xe đứng yên, trước một chuỗi Move_Straight / Turn_Angle.
// Chỉnh lại offset gyro Z trước (0.5s): offset sai (vd hiệu chỉnh lúc bật nguồn bị xoay nhẹ) thì PID giữ
// hướng sẽ giữ đúng cái hướng SAI đó -> xe quay đều suốt đoạn đường (đã gặp: lệch 2.68 độ/s -> quay ~12 độ/m)
void Motion_Begin(void)
{
    Motor_SetSpeed(0.0f, 0.0f);
    MPU6500_ZeroGyroZ();                  // Bánh quay / bị chạm thì giữ offset cũ
    Motion_SetOrigin();
}
// Như Motion_Begin nhưng KHÔNG đo lại gyro (không chặn). Mode Manager đo lại gyro trước bằng MPU6500_ZeroGyroZ_Start/Poll
void Motion_SetOrigin(void)
{
    Motor_SetSpeed(0.0f, 0.0f);
    heading_target = Robot_Heading();
    odo_x_mm = 0.0f;
    odo_y_mm = 0.0f;
    heading_set = 1;
}
// XE BỊ NHẤC (gọi trong ngắt IMU, mpu6500.c): hủy chuyển động đang chạy, tắt PID, phanh 2 bánh ngay.
// Hàm đang chờ (Move_Straight, Turn_Angle...) trả về 0, kết quả "xe bi nhac len"
void Motion_OnLifted(void)
{
    if (motion_mode != MOTION_IDLE) motion_result = MOTION_LIFTED;
    Motor_Stop();
}
// XE VỪA ĐẶT XUỐNG (gọi trong ngắt IMU, yaw đã về 0): hướng + vị trí lúc đặt xuống làm gốc
void Motion_OnPlaced(void)
{
    heading_target = Robot_Heading();
    heading_set    = 1;
    odo_x_mm       = 0.0f;
    odo_y_mm       = 0.0f;
}
// Bắt đầu 1 đoạn (không chờ). 0: động cơ bị khóa / IMU chưa sẵn sàng / xe đang bị nhấc
static uint8_t Motion_Start(MotionMode mode)
{
    if (!motor_hw_ok || !imu_ready) {
        motion_result = MOTION_NOT_READY;
        return 0;
    }
    if (robot_lifted) {
        motion_result = MOTION_LIFTED;
        return 0;
    }
    motion_max_err   = 0.0f;
    hold_active      = 0;
    if (mode == MOTION_HOLD) Wall_Reset();
    motion_result    = MOTION_RUNNING;
    motion_mode      = mode;              // Ghi cuối cùng: các thông số đã sẵn sàng khi ngắt đọc
    return 1;
}
// Chờ ngắt TIM9 báo xong (luồng chính)
static uint8_t Motion_Wait(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    while (motion_mode != MOTION_IDLE) {
        if (HAL_GetTick() - t0 > timeout_ms) {
            Motor_SetSpeed(0.0f, 0.0f);       // Hủy, giảm tốc rồi dừng
            motion_result = MOTION_TIMEOUT;
            break;
        }
    }
    if (motion_result == MOTION_ABORT) Motor_Stop();   // Lệch hướng bất thường: phanh ngay
    return motion_result == MOTION_OK;
}
// Chuẩn bị đoạn đi thẳng, trả về thời gian tối đa: MOTION_TIMEOUT_MS + 2 lần thời gian chạy hết đoạn
// ở tốc độ tối đa (đoạn dài không bị cắt)
static uint32_t Straight_Prepare(float dist_mm, float speed_mms)
{
    if (!heading_set) Motion_Begin();
    Motor_SetSpeed(0.0f, 0.0f);           // Bật PID vận tốc (vòng ngoài sẽ đặt vận tốc)
    move_start_mm    = Robot_Distance();
    move_target_mm   = dist_mm;
    move_speed_mms   = speed_mms;
    move_wall        = wall_guard && dist_mm > 0.0f;   // Chỉ có mắt trước: lùi thì không xét tường
    motion_abort_deg = MOTION_ABORT_DEG;
    return MOTION_TIMEOUT_MS + (uint32_t)(2000.0f * fabsf(dist_mm) / speed_mms);
}
uint8_t Move_Straight(float dist_mm, float speed_mms)
{
    speed_mms = fabsf(speed_mms);
    if (speed_mms > MAX_SPEED_MMS) speed_mms = MAX_SPEED_MMS;
    if (speed_mms < 1.0f || fabsf(dist_mm) < MOVE_TOL_MM) {
        // Tốc độ 0 hoặc quãng đường 0: ĐỨNG GIỮ HƯỚNG + tường trước (lùi khi IR > WALL_STOP_IR, tự căn vuông góc,
        // đặt lại MPU), chạy nền trong ngắt, không chờ. Đang giữ rồi thì để yên -> gọi lặp trong while(1) được
        if (motion_mode == MOTION_HOLD) return 1;
        if (!heading_set) Motion_Begin();
        Motor_SetSpeed(0.0f, 0.0f);
        return Motion_Start(MOTION_HOLD);
    }
    uint32_t timeout_ms = Straight_Prepare(dist_mm, speed_mms);
    if (!Motion_Start(MOTION_STRAIGHT)) return 0;
    uint8_t ok = Motion_Wait(timeout_ms);
    // Bị tường trước chặn giữa đường, hoặc đi đủ quãng đường mà trước mặt có tường (cả 2 mắt >= WALL_PRESENT_IR):
    // tiến / lùi tới WALL_TARGET_IR, căn vuông góc, đặt lại hướng MPU
    uint8_t wall_ahead = ok && move_wall && fminf(IR_FL_Filt(), IR_FR_Filt()) >= WALL_PRESENT_IR;
    if (motion_result != MOTION_WALL && !wall_ahead) return ok;
    // Giữ |lệch hướng| lớn nhất của đoạn đi thẳng cho báo cáo
    float max_err = motion_max_err;
    ok = Wall_Align();
    motion_max_err = max_err;
    return ok ? 2 : 0;
}
uint8_t Turn_Angle(float deg)
{
    if (!heading_set) Motion_Begin();
    if (!Turn_Angle_Begin(deg)) return 0;
    return Motion_Wait(MOTION_TIMEOUT_MS + (uint32_t)(2000.0f * fabsf(deg) / TURN_SPEED_DPS));
}
// Quay KHÔNG CHẶN: bắt đầu rồi trả về ngay, theo dõi bằng Motion_Status() (RUNNING -> DONE / FAIL), người gọi tự
// giới hạn thời gian. 0: không quay được (động cơ khóa / IMU chưa sẵn sàng / bị nhấc), hướng đích giữ nguyên
uint8_t Turn_Angle_Begin(float deg)
{
    if (!heading_set) Motion_SetOrigin();
    Motor_SetSpeed(0.0f, 0.0f);
    heading_target  += deg;
    motion_turn_dir  = (deg >= 0.0f) ? 1.0f : -1.0f;
    motion_abort_deg = fabsf(deg) + MOTION_ABORT_DEG;   // Quay ngược chiều quá MOTION_ABORT_DEG -> sai dấu gyro
    if (!Motion_Start(MOTION_TURN)) {
        heading_target -= deg;
        return 0;
    }
    return 1;
}
uint8_t Turn_Left(float deg)  { return Turn_Angle(deg); }
uint8_t Turn_Right(float deg) { return Turn_Angle(-deg); }

// ===== ĐI THẲNG KHÔNG CHỜ (solving_maze.c: chạy liên tục nhiều ô, kéo dài / rút ngắn đoạn đang chạy) =====
// Cùng chế độ MOTION_STRAIGHT với Move_Straight (PD giữ hướng gyro, hãm theo quãng còn lại, dừng trước tường, tránh
// tường bên) nhưng trả về ngay. Luồng chính đọc quãng đã đi, đổi đích bất cứ lúc nào: ngắt TIM9 tính lại vận tốc theo
// quãng còn lại mỗi 2ms -> kéo dài thì xe chạy tiếp không giảm tốc, rút ngắn thì xe hãm vừa kịp dừng ở đích mới.
// Không tự căn tường khi xong (Move_Straight có), không giới hạn thời gian (người gọi tự kiểm tra)
uint8_t Move_Straight_Begin(float dist_mm, float speed_mms)
{
    speed_mms = fabsf(speed_mms);
    if (speed_mms > MAX_SPEED_MMS) speed_mms = MAX_SPEED_MMS;
    if (speed_mms < 1.0f) return 0;
    (void)Straight_Prepare(dist_mm, speed_mms);
    return Motion_Start(MOTION_STRAIGHT);
}
// Đổi quãng đường đích (tính từ lúc Move_Straight_Begin) khi đang chạy. Ghi 1 float 32-bit: nguyên tử với ngắt
void Move_Straight_SetTarget(float dist_mm)
{
    move_target_mm = dist_mm;
}
// Quãng đường tâm xe đã đi từ lúc Move_Straight_Begin (mm). Tắt ngắt để 2 bánh cùng 1 chu kỳ
float Move_Straight_Traveled(void)
{
    __disable_irq();
    float d = Robot_Distance() - move_start_mm;
    __enable_irq();
    return d;
}
// Trạng thái đoạn chuyển động gần nhất (MOTION_STATUS_*, pid.h)
uint8_t Motion_Status(void)
{
    if (motion_mode != MOTION_IDLE) return MOTION_STATUS_RUNNING;
    switch (motion_result) {
    case MOTION_OK:   return MOTION_STATUS_DONE;
    case MOTION_WALL: return MOTION_STATUS_WALL;
    default:          return MOTION_STATUS_FAIL;
    }
}

// ===== TỰ CĂN THEO TƯỜNG TRƯỚC, CHỜ XONG (Wall_Align, xem pid.h) =====
// Dùng đúng chế độ đứng giữ hướng (MOTION_HOLD, căn tường chạy trong ngắt - Hold_Update), chờ tới khi căn xong
// (WS_DONE) / không căn được (WS_FAIL), rồi thôi giữ, phanh đứng yên như các lệnh chặn khác
uint8_t Wall_Align(void)
{
    uint8_t r;
    if (!heading_set) Motion_Begin();
    if (!Wall_Align_Begin()) return 0;
    while ((r = Wall_Align_Poll()) == WALL_ALIGN_BUSY) {
    }
    return r == WALL_ALIGN_OK;
}
// Bản KHÔNG CHẶN (giải mê cung): Begin rồi gọi Poll mỗi vòng lặp tới khi khác WALL_ALIGN_BUSY.
// Poll trả kết quả đúng 1 lần (lần đó thôi giữ, phanh đứng yên); gọi tiếp sau đó trả WALL_ALIGN_FAILED
static float    wa_href;                          // Hướng xe được lệnh đi = hướng sẽ gán cho pháp tuyến tường
static uint32_t wa_t0;
static uint8_t  wa_active = 0;
uint8_t Wall_Align_Begin(void)
{
    if (!heading_set) Motion_SetOrigin();
    wa_href = heading_target;
    Motor_SetSpeed(0.0f, 0.0f);
    ws_force = 1;                                 // Không thấy tường -> WS_FAIL ngay (không chờ mãi)
    if (!Motion_Start(MOTION_HOLD)) {
        ws_force = 0;
        return 0;
    }
    wa_t0     = HAL_GetTick();
    wa_active = 1;
    return 1;
}
uint8_t Wall_Align_Poll(void)
{
    if (!wa_active) return WALL_ALIGN_FAILED;
    if (motion_mode == MOTION_HOLD && ws != WS_DONE && ws != WS_FAIL && HAL_GetTick() - wa_t0 < WALL_ALIGN_TIMEOUT_MS)
        return WALL_ALIGN_BUSY;
    uint8_t ok = (motion_mode == MOTION_HOLD && ws == WS_DONE);
    wa_active = 0;
    ws_force  = 0;
    Motor_SetSpeed(0.0f, 0.0f);
    if (!ok) {
        if (motion_result != MOTION_LIFTED && motion_result != MOTION_NOT_READY) motion_result = MOTION_WALL_FAIL;
        heading_target = wa_href;                 // Lệnh sau vẫn tính theo hướng cũ
        return WALL_ALIGN_FAILED;
    }
    motion_result = MOTION_WALL;
    return WALL_ALIGN_OK;
}

// ===== TEST / DEBUG =====
// In dữ liệu PID để chỉnh hệ số (vẽ bằng Serial Plotter): vận tốc đặt (sau ramp), đo, PWM của trái rồi phải
void print_SpeedPID(){
	snprintf(msg, sizeof(msg), "PID,%.1f,%.1f,%ld,%.1f,%.1f,%ld\r\n",
	         ref_speed_left, speed_left_mms, pwm_left,
	         ref_speed_right, speed_right_mms, pwm_right);
	CDC_Print(msg);
	HAL_Delay(20);
}
// Phân tích 1 bước thử của 1 bánh (right = 0: trái, 1: phải) rồi in 1 dòng kết quả
void PID_Test_Report(const PIDLog_TypeDef *samples, float v_start, float v_end, uint8_t right)
{
    float delta     = v_end - v_start;
    float dir       = (delta >= 0.0f) ? 1.0f : -1.0f;
    float level_90  = v_start + 0.9f * delta;  // Mốc thời gian lên: đi được 90% bước nhảy
    int   t_rise    = -1;                      // ms, -1 = chưa đạt
    int   t_settle  = 0;                       // ms, lần cuối vận tốc còn nằm ngoài ngưỡng ổn định
    float overshoot = 0.0f;                    // mm/s vượt quá đích theo chiều đang đổi
    float track_max = 0.0f;                    // |đặt - đo| lớn nhất, đo độ bám theo ramp
    int   pwm_max   = 0;
    float err_sum   = 0.0f;                    // Sai số trung bình 1/4 cuối bước (đã ổn định)
    int   err_n     = 0;

    for (int i = 0; i < PID_TEST_STAGE_LEN; i++) {
        float ref  = right ? samples[i].ref_r  : samples[i].ref_l;
        float meas = right ? samples[i].meas_r : samples[i].meas_l;
        int   pwm  = right ? samples[i].pwm_r  : samples[i].pwm_l;
        float err  = meas - v_end;

        if (t_rise < 0 && dir * (meas - level_90) >= 0.0f) t_rise = i * 2;
        if (dir * err > overshoot) overshoot = dir * err;
        if (err > PID_SETTLE_BAND || err < -PID_SETTLE_BAND) t_settle = (i + 1) * 2;
        float track = (ref > meas) ? ref - meas : meas - ref;
        if (track > track_max) track_max = track;
        if (pwm < 0) pwm = -pwm;
        if (pwm > pwm_max) pwm_max = pwm;
        if (i >= PID_TEST_STAGE_LEN * 3 / 4) {
            err_sum += err;
            err_n++;
        }
    }

    // "--" = không đạt trong thời gian của bước
    char rise_str[12], settle_str[12];
    if (t_rise >= 0) snprintf(rise_str, sizeof(rise_str), "%d", t_rise);
    else             snprintf(rise_str, sizeof(rise_str), "--");
    if (t_settle < PID_TEST_STAGE_LEN * 2) snprintf(settle_str, sizeof(settle_str), "%d", t_settle);
    else                                   snprintf(settle_str, sizeof(settle_str), "--");
    float overshoot_pct = (delta != 0.0f) ? overshoot * 100.0f / (dir * delta) : 0.0f;

    snprintf(msg, sizeof(msg), "%5.0f->%-5.0f %-4s %9s %9.1f %8s %12.1f %13.1f %7d%s\r\n",
             v_start, v_end, right ? "Phai" : "Trai", rise_str, overshoot_pct, settle_str,
             err_sum / err_n, track_max, pwm_max, (pwm_max >= PWM_MAX) ? " BAO HOA" : "");
    CDC_Print(msg);
}
// CHẠY BÀI TEST PID: đứng yên -> lần lượt các vận tốc trong pid_test_speeds[] (mỗi bước 0.4s) -> dừng.
// Ngắt TIM9 ghi mẫu mỗi 2ms, chạy xong mới phân tích và in báo cáo qua USB
// (in trực tiếp chỉ được ~50 mẫu/s, không thấy được đáp ứng của vòng PID 2ms).
// print_raw = 1: in thêm toàn bộ dữ liệu thô (dòng STEP,...) để vẽ đồ thị.
// Lưu ý: xe chạy tới ~25cm rồi lùi lại một đoạn -> để trên sàn trống hoặc kê bánh lên.
// Muốn thử bước nhảy "thật" (không ramp) thì tạm tăng MAX_ACCEL_MMS2.
void PID_Test(uint8_t print_raw)
{
    if (!motor_hw_ok) {
        CDC_Print("\r\nLOI: chua doi PB0 -> TIM1_CH2N, PA3 -> TIM5_CH4 trong CubeMX. Dong co dang bi khoa.\r\n");
        return;
    }

    // 1. Dừng hẳn rồi chạy lần lượt các bước, ngắt TIM9 tự ghi mẫu
    Motor_SetSpeed(0.0f, 0.0f);
    HAL_Delay(500);
    pid_log_count = 0;
    for (int k = 0; k < PID_TEST_STAGES; k++) {
        Motor_SetSpeed(pid_test_speeds[k], pid_test_speeds[k]);
        while (pid_log_count < (k + 1) * PID_TEST_STAGE_LEN) {}
    }
    Motor_SetSpeed(0.0f, 0.0f);

    // 2. Chờ máy tính sẵn sàng rồi mới in (dữ liệu vẫn nằm trong RAM):
    //    chạy thử trên sàn không cắm dây -> xe chạy xong đứng phanh chờ, cắm USB + mở terminal là có báo cáo
    CDC_WaitHost();

    // 3. In báo cáo: thông số đang dùng + kết quả từng bước của từng bánh
    CDC_Print("\r\n================ TEST PID VAN TOC ================\r\n");
    snprintf(msg, sizeof(msg), "Trai: Ks=%.0f Kf=%.2f Ka=%.3f Kp=%.2f Ki=%.1f Kd=%.3f | Phai: Ks=%.0f Kf=%.2f Ka=%.3f Kp=%.2f Ki=%.1f Kd=%.3f\r\n",
             pid_speed_left.Ks, pid_speed_left.Kf, pid_speed_left.Ka, pid_speed_left.Kp, pid_speed_left.Ki, pid_speed_left.Kd,
             pid_speed_right.Ks, pid_speed_right.Kf, pid_speed_right.Ka, pid_speed_right.Kp, pid_speed_right.Ki, pid_speed_right.Kd);
    CDC_Print(msg);
    snprintf(msg, sizeof(msg), "Gia toc %.0f mm/s2 | Nguong on dinh +-%.0f mm/s | Moi buoc %d ms\r\n",
             MAX_ACCEL_MMS2, PID_SETTLE_BAND, PID_TEST_STAGE_LEN * 2);
    CDC_Print(msg);
    CDC_Print("Buoc(mm/s)   Banh t_len(ms) Vot_lo(%)  t_od(ms) Sai_so(mm/s) Bam_max(mm/s) PWM_max\r\n");
    float v_start = 0.0f;
    for (int k = 0; k < PID_TEST_STAGES; k++) {
        const PIDLog_TypeDef *stage = &pid_log[k * PID_TEST_STAGE_LEN];
        PID_Test_Report(stage, v_start, pid_test_speeds[k], 0);
        PID_Test_Report(stage, v_start, pid_test_speeds[k], 1);
        v_start = pid_test_speeds[k];
    }
    CDC_Print("t_len: thoi gian dat 90% buoc | Vot_lo: % vuot qua dich | t_od: thoi gian vao nguong on dinh\r\n");
    CDC_Print("Sai_so: sai so TB 0.1s cuoi | Bam_max: lech lon nhat giua dat (ramp) va do | --: khong dat\r\n");

    // 4. Dữ liệu thô
    if (print_raw) {
        CDC_Print("STEP,t_ms,dat_trai,do_trai,pwm_trai,dat_phai,do_phai,pwm_phai\r\n");
        for (int i = 0; i < PID_LOG_SIZE; i++) {
            PIDLog_TypeDef *s = &pid_log[i];
            snprintf(msg, sizeof(msg), "STEP,%d,%d,%d,%d,%d,%d,%d\r\n", i * 2,
                     s->ref_l, s->meas_l, s->pwm_l, s->ref_r, s->meas_r, s->pwm_r);
            CDC_Print(msg);
        }
    }
    CDC_Print("================ HET TEST ================\r\n");
}
// ĐI HÌNH CHỮ NHẬT: thẳng length -> quay phải 90 -> thẳng width -> quay phải 90 -> ... (4 cạnh, 4 lần quay)
// -> về đúng chỗ và hướng xuất phát. Quãng đường đo bằng encoder, hướng giữ / quay bằng gyro MPU6500.
// Đặt xe trên sàn, bấm reset rồi rút tay: xe đứng yên 1s (IMU chỉnh offset) mới chạy.
// Chạy xong đứng phanh, CHỜ máy tính mở cổng COM rồi in báo cáo (giống PID_Test).
typedef struct {
    uint8_t      turn;                    // 0: đi thẳng, 1: quay
    MotionResult result;
    float        target;                  // mm hoặc độ
    float        done;                    // Quãng đường đi được (mm) / góc đã quay (độ)
    float        head_err;                // Hướng đích - hướng thật lúc xong (độ)
    float        max_err;                 // |Lệch hướng| lớn nhất trong đoạn (độ)
    uint32_t     ms;
} MotionSeg;
// Chuỗi in ra cho từng MotionResult
static const char *const motion_result_str[] = { "OK", "dang chay", "HUY: lech huong (sai YAW_SIGN? bi va?)",
                                                 "HUY: qua thoi gian (ket banh?)", "HUY: chua san sang",
                                                 "HUY: xe bi nhac len",
                                                 "DUNG TRUOC TUONG: da can vuong goc, dat lai huong MPU",
                                                 "HUY: dung truoc tuong nhung khong can vuong goc duoc" };
void Rectangle_Test(float length_mm, float width_mm)
{
    MotionSeg seg[8];
    uint8_t n = 0;

    if (!motor_hw_ok || !imu_ready) {
        CDC_WaitHost();
        CDC_Print("\r\nLOI: dong co bi khoa (CubeMX) hoac IMU chua san sang (MPU6500_Setup) - khong chay duoc\r\n");
        return;
    }

    // 1. Đứng yên 1s rồi lấy vị trí + hướng hiện tại làm gốc
    Motor_SetSpeed(0.0f, 0.0f);
    HAL_Delay(1000);
    Motion_Begin();

    // 2. 4 cạnh, sau mỗi cạnh quay phải 90 độ. Đoạn nào hỏng thì dừng luôn
    for (int k = 0; k < 8; k++) {
        MotionSeg *s = &seg[n++];
        float h0 = Robot_Heading(), d0 = Robot_Distance();
        uint32_t t0 = HAL_GetTick();
        uint8_t ok;
        s->turn = k & 1;
        if (!s->turn) {
            s->target = (k % 4 == 0) ? length_mm : width_mm;
            ok = Move_Straight(s->target, MOVE_SPEED_MMS);
        } else {
            s->target = -90.0f;
            ok = Turn_Right(90.0f);
        }
        s->ms       = HAL_GetTick() - t0;
        s->result   = motion_result;
        s->done     = s->turn ? Robot_Heading() - h0 : Robot_Distance() - d0;
        s->head_err = heading_target - Robot_Heading();
        s->max_err  = motion_max_err;
        if (!ok) break;
    }
    float final_x = odo_x_mm, final_y = odo_y_mm, final_err = heading_target - Robot_Heading();

    // 3. Chờ máy tính rồi in báo cáo
    CDC_WaitHost();
    CDC_Print("\r\n================ TEST HINH CHU NHAT ================\r\n");
    snprintf(msg, sizeof(msg), "Canh %.0f x %.0f mm, quay phai | thang toi da %.0f mm/s, quay toi da %.0f do/s | TRACK_WIDTH %.0f mm\r\n",
             length_mm, width_mm, MOVE_SPEED_MMS, TURN_SPEED_DPS, TRACK_WIDTH_MM);
    CDC_Print(msg);
    CDC_Print("Doan Loai   Dich      Dat_duoc  LechHuongCuoi LechMax/VuotQua t(ms) Ket qua\r\n");
    for (int k = 0; k < n; k++) {
        MotionSeg *s = &seg[k];
        const char *unit = s->turn ? "do" : "mm";
        snprintf(msg, sizeof(msg), "%3d  %-5s %+7.1f%s %+7.1f%s  %+7.2f do   %s %5.2f do %6lu %s\r\n",
                 k + 1, s->turn ? "Quay" : "Thang", s->target, unit, s->done, unit,
                 s->head_err, s->turn ? "vuot" : "lech", s->max_err, (unsigned long)s->ms, motion_result_str[s->result]);
        CDC_Print(msg);
    }
    snprintf(msg, sizeof(msg), "Vi tri cuoi (uoc luong encoder + gyro): x=%+.1f y=%+.1f mm -> cach diem xuat phat %.1f mm\r\n",
             final_x, final_y, sqrtf(final_x * final_x + final_y * final_y));
    CDC_Print(msg);
    snprintf(msg, sizeof(msg), "Huong cuoi lech %+.2f do so voi huong xuat phat\r\n", final_err);
    CDC_Print(msg);
    CDC_Print("Day la uoc luong cua chinh xe (tin encoder + gyro). Do bang thuoc vi tri / goc THAT:\r\n");
    CDC_Print("- canh that ngan/dai deu hon dich -> WHEEL_DIAMETER_L va _R moi = cu x (canh that / canh dich)\r\n");
    CDC_Print("- 4 lan quay that khong du/qua 360 do -> YAW_SCALE moi = cu x (goc that / 360) (xem pid.h)\r\n");
    CDC_Print("================ HET TEST ================\r\n");
}
// ĐI THẲNG dist_mm (âm = lùi) rồi in báo cáo. Quãng đường đo bằng encoder, hướng giữ bằng gyro MPU6500.
// Đặt xe trên sàn, bấm reset rồi rút tay: xe đứng yên 1s (IMU chỉnh offset) mới chạy.
// Chạy xong đứng phanh, CHỜ máy tính mở cổng COM rồi in báo cáo (giống PID_Test).
void Straight_Test(float dist_mm, float speed_mms)
{
    if (!motor_hw_ok || !imu_ready) {
        CDC_WaitHost();
        CDC_Print("\r\nLOI: dong co bi khoa (CubeMX) hoac IMU chua san sang (MPU6500_Setup) - khong chay duoc\r\n");
        return;
    }

    // 1. Đứng yên 1s rồi lấy vị trí + hướng hiện tại làm gốc, chạy
    Motor_SetSpeed(0.0f, 0.0f);
    HAL_Delay(1000);
    Motion_Begin();
    float h0 = heading_target * DEG2RAD;
    float l0 = dist_left_mm, r0 = dist_right_mm;
    uint32_t t0 = HAL_GetTick();
    Move_Straight(dist_mm, speed_mms);
    uint32_t ms = HAL_GetTick() - t0;

    // 2. Kết quả (đọc ngay, trước khi chờ máy tính). Vị trí đổi về trục của xe lúc xuất phát
    MotionResult result = motion_result;
    float dl = dist_left_mm - l0, dr = dist_right_mm - r0;
    float head_err = heading_target - Robot_Heading();
    float max_err = motion_max_err;
    float along   =  odo_x_mm * cosf(h0) + odo_y_mm * sinf(h0);   // Dọc hướng xuất phát
    float lateral = -odo_x_mm * sinf(h0) + odo_y_mm * cosf(h0);   // Ngang, dương = lệch sang trái

    // 3. Chờ máy tính rồi in báo cáo
    CDC_WaitHost();
    CDC_Print("\r\n================ TEST DI THANG ================\r\n");
    snprintf(msg, sizeof(msg), "Dich %+.1f mm | toc do toi da %.0f mm/s, giam toc %.0f mm/s2 | sai so cho phep %.1f mm\r\n",
             dist_mm, speed_mms, MOVE_DECEL_MMS2, MOVE_TOL_MM);
    CDC_Print(msg);
    snprintf(msg, sizeof(msg), "Ket qua: %s | thoi gian %lu ms (TB %.0f mm/s)\r\n", motion_result_str[result],
             (unsigned long)ms, (ms > 0) ? fabsf(0.5f * (dl + dr)) * 1000.0f / ms : 0.0f);
    CDC_Print(msg);
    snprintf(msg, sizeof(msg), "Quang duong (encoder): tam xe %+.1f mm (lech dich %+.1f) | banh trai %+.1f | banh phai %+.1f mm\r\n",
             0.5f * (dl + dr), 0.5f * (dl + dr) - dist_mm, dl, dr);
    CDC_Print(msg);
    snprintf(msg, sizeof(msg), "Huong (gyro): lech luc dung %+.2f do | lech lon nhat khi chay %.2f do\r\n", head_err, max_err);
    CDC_Print(msg);
    snprintf(msg, sizeof(msg), "Vi tri cuoi (uoc luong encoder + gyro): doc %+.1f mm, ngang %+.1f mm (duong = lech trai)\r\n",
             along, lateral);
    CDC_Print(msg);
    CDC_Print("Day la uoc luong cua chinh xe. Do bang thuoc quang duong THAT roi chinh:\r\n");
    snprintf(msg, sizeof(msg), "  WHEEL_DIAMETER_L moi = %.2f x (quang duong that / %.1f), WHEEL_DIAMETER_R moi = %.2f x (cung ti le)\r\n",
             WHEEL_DIAMETER_L, fabsf(0.5f * (dl + dr)), WHEEL_DIAMETER_R);
    CDC_Print(msg);
    CDC_Print("  Xe cong deu 1 phia du gyro bao thang -> truc Z cua MPU bi nghieng / banh truot\r\n");
    CDC_Print("================ HET TEST ================\r\n");
}

// ===== TỰ HIỆU CHỈNH ĐỘ NHẠY GYRO BẰNG CÁCH QUAY (Gyro_Spin_Calibrate) =====
// Tín hiệu IR (làm mịn 5 mẫu ~7 độ), nhân pol để đỉnh tường luôn là cực ĐẠI (mạch có thể cho giá trị giảm khi gần)
static float Spin_Signal(uint32_t i, uint32_t n, float pol)
{
    uint32_t a = (i >= 2) ? i - 2 : 0, b = (i + 2 < n) ? i + 2 : n - 1;
    float s = 0.0f;
    for (uint32_t j = a; j <= b; j++) s += spin_rec[j].ir;
    return pol * s / (float)(b - a + 1);
}
// Tìm đỉnh tường trong cửa sổ hướng [lo, hi). Vị trí đỉnh = trọng tâm phần tín hiệu trên 70% (chính xác hơn
// lấy 1 mẫu lớn nhất). Trả 1 nếu có đỉnh rõ, nằm trọn trong cửa sổ; h, dd = hướng gyro, chênh quãng 2 bánh tại đỉnh
static uint8_t Spin_FindPeak(uint32_t n, float lo, float hi, float pol, float *h, float *dd, float *contrast)
{
    uint32_t i0 = n, i1 = 0, i_max = n;
    float s_max = -1e9f, s_min = 1e9f;
    for (uint32_t i = 0; i < n; i++) {
        if (spin_rec[i].h < lo || spin_rec[i].h >= hi) continue;
        if (i < i0) i0 = i;
        i1 = i + 1;
        float s = Spin_Signal(i, n, pol);
        if (s > s_max) { s_max = s; i_max = i; }
        if (s < s_min) s_min = s;
    }
    *contrast = (i_max < n) ? s_max - s_min : 0.0f;
    if (i_max >= n || *contrast < SPIN_CAL_MIN_CONTRAST) return 0;
    float thr = s_min + 0.7f * (s_max - s_min);
    uint32_t a = i_max, b = i_max;
    while (a > i0 && Spin_Signal(a - 1, n, pol) > thr) a--;
    while (b + 1 < i1 && Spin_Signal(b + 1, n, pol) > thr) b++;
    if (a == i0 || b + 1 == i1) return 0;          // Đỉnh bị cắt ở mép cửa sổ -> không tin
    float w_sum = 0.0f, h_sum = 0.0f, d_sum = 0.0f;
    for (uint32_t i = a; i <= b; i++) {
        float w = Spin_Signal(i, n, pol) - thr;
        w_sum += w;
        h_sum += w * spin_rec[i].h;
        d_sum += w * spin_rec[i].dd;
    }
    *h  = h_sum / w_sum;
    *dd = d_sum / w_sum;
    return 1;
}
void Gyro_Spin_Calibrate(uint8_t turns)
{
    float peak_h[11], peak_dd[11], contrast_min = 1e9f;
    if (turns < 2) turns = 2;
    if (turns > 10) turns = 10;
    if (!motor_hw_ok || !imu_ready) {
        UART_Print("\r\nLOI: dong co bi khoa (CubeMX) hoac IMU chua san sang (MPU6500_Setup) - khong chay duoc\r\n");
        HAL_Delay(1000);
        return;
    }
    snprintf(msg, sizeof(msg), "\r\nHieu chinh gyro: dat xe cach tuong 3-10cm, mui huong vao tuong. Xe se quay trai %u vong.\r\n"
             "Gui 1 ky tu de bat dau...\r\n", turns);
    UART_WaitStart(msg);

    // 1. Đứng yên 1s, đo lại offset gyro, lấy hướng ban đầu
    Motor_SetSpeed(0.0f, 0.0f);
    HAL_Delay(1000);
    Motion_Begin();
    float scale_old = yaw_scale, h0 = Robot_Heading();

    // 2. Quay trái đều SPIN_CAL_DPS, ghi dữ liệu, đủ (turns + 1) vòng + 180 độ thì dừng
    float total = (turns + 1) * 360.0f + 180.0f;
    float v = SPIN_CAL_DPS * DEG2RAD * TRACK_WIDTH_MM * 0.5f;
    spin_rec_div = 0;
    spin_rec_n = 0;
    Motor_SetSpeed(-v, v);
    uint32_t t0 = HAL_GetTick(), limit_ms = (uint32_t)(total / SPIN_CAL_DPS * 2000.0f) + 3000U;
    while (Robot_Heading() - h0 < total && HAL_GetTick() - t0 < limit_ms && spin_rec_n < SPIN_CAL_MAX) {}
    Motor_SetSpeed(0.0f, 0.0f);
    uint32_t n = spin_rec_n;
    spin_rec_n = SPIN_CAL_MAX;            // Ngừng ghi
    HAL_Delay(600);                       // Chờ dừng hẳn
    float h_end = Robot_Heading();

    // 3. Chiều tín hiệu: đỉnh tường là cực đại hay cực tiểu (xét 1 vòng, bỏ 90 độ đầu lúc mới quay)
    float mean = 0.0f, mx = 0.0f, mn = 1e9f;
    uint32_t cnt = 0;
    for (uint32_t i = 0; i < n; i++) {
        if (spin_rec[i].h < h0 + 90.0f || spin_rec[i].h >= h0 + 450.0f) continue;
        float s = spin_rec[i].ir;
        mean += s;
        cnt++;
        if (s > mx) mx = s;
        if (s < mn) mn = s;
    }
    float pol = (cnt > 0 && mx - mean / cnt >= mean / cnt - mn) ? 1.0f : -1.0f;

    // 4. Đỉnh vòng đầu (trong 1 vòng bất kỳ sau 90 độ đầu), rồi lần theo từng vòng: +360 độ +- cửa sổ
    const char *err = NULL;
    uint8_t found = Spin_FindPeak(n, h0 + 90.0f, h0 + 450.0f, pol, &peak_h[0], &peak_dd[0], &contrast_min);
    if (!found) err = "khong thay dinh tuong ro (dat gan tuong hon? IR co doc duoc khong?)";
    for (uint8_t k = 1; k <= turns && !err; k++) {
        float c;
        float expect = peak_h[k - 1] + 360.0f;
        if (!Spin_FindPeak(n, expect - SPIN_CAL_WINDOW_DEG, expect + SPIN_CAL_WINDOW_DEG, pol, &peak_h[k], &peak_dd[k], &c))
            err = "mat dinh tuong o 1 vong (xe bi xe dich? co vat la?)";
        if (c < contrast_min) contrast_min = c;
    }
    if (n >= SPIN_CAL_MAX && !err) err = "het bo nho ghi (giam so vong)";

    UART_Print("\r\n================ HIEU CHINH GYRO BANG CACH TU QUAY ================\r\n");
    snprintf(msg, sizeof(msg), "Quay trai %u vong @%.0f do/s | %lu mau | IR truoc %s khi gan tuong\r\n",
             turns, SPIN_CAL_DPS, (unsigned long)n, (pol > 0.0f) ? "TANG" : "GIAM");
    UART_Print(msg);

    float scale_k = 1.0f;                 // Hệ số sửa độ nhạy: góc thật / góc gyro
    if (!err) {
        float gyro_deg = peak_h[turns] - peak_h[0];
        scale_k = turns * 360.0f / gyro_deg;
        float d_min = 1e9f, d_max = -1e9f;
        for (uint8_t k = 1; k <= turns; k++) {
            float d = peak_h[k] - peak_h[k - 1];
            if (d < d_min) d_min = d;
            if (d > d_max) d_max = d;
            snprintf(msg, sizeof(msg), "  Vong %2u: dinh tuong o %+9.2f do gyro (cach vong truoc %.2f)\r\n", k, peak_h[k] - h0, d);
            UART_Print(msg);
        }
        float track = (peak_dd[turns] - peak_dd[0]) / (turns * 2.0f * 3.14159265f);
        if (fabsf(scale_k - 1.0f) > 0.08f) err = "sai so > 8% -> bat thuong, khong ap dung";
        snprintf(msg, sizeof(msg), "Gyro do %u vong THAT = %.2f do (dung ra %u) -> gyro %s %.3f%% | do tuong phan IR %.0f\r\n",
                 turns, gyro_deg, turns * 360, (gyro_deg > turns * 360.0f) ? "doc NHIEU" : "doc IT",
                 fabsf(gyro_deg / (turns * 360.0f) - 1.0f) * 100.0f, contrast_min);
        UART_Print(msg);
        snprintf(msg, sizeof(msg), "Do on dinh: moi vong lech nhau toi da %.2f do | TRACK_WIDTH hieu dung khi quay: %.1f mm (dang dung %.1f)\r\n",
                 d_max - d_min, track, TRACK_WIDTH_MM);
        UART_Print(msg);
    }
    // 5. Áp dụng độ nhạy mới rồi quay về hướng ban đầu (góc thật đã quay = góc gyro x hệ số mới)
    if (!err) {
        yaw_scale = scale_old * scale_k;
        snprintf(msg, sizeof(msg), "YAW_SCALE: %.5f -> %.5f  (DA AP DUNG. Chep vao pid.h: #define YAW_SCALE %.5ff)\r\n",
                 scale_old, yaw_scale, yaw_scale);
    } else {
        snprintf(msg, sizeof(msg), "THAT BAI: %s -> giu YAW_SCALE %.5f\r\n", err, scale_old);
    }
    UART_Print(msg);
    float turned = (h_end - h0) * scale_k;                       // Góc thật đã quay
    float back = -(turned - 360.0f * roundf(turned / 360.0f));   // Còn lệch so với hướng ban đầu
    Motion_Begin();                       // Đo lại offset, hướng gốc mới theo độ nhạy mới
    uint8_t ok = Turn_Angle(back);
    snprintf(msg, sizeof(msg), "Quay ve huong ban dau: %+.2f do -> %s%s\r\n", back, ok ? "OK" : "LOI",
             err ? " (theo gyro CHUA hieu chinh -> co the lech)" : "");
    UART_Print(msg);
    UART_Print("================ HET ================\r\n");
}
