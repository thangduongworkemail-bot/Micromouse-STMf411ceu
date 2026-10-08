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
#include <math.h>           // sqrtf, fabsf, sinf, cosf
#include <stdio.h>          // snprintf
#include <stdlib.h>         // abs
#include <string.h>         // memset

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
PID_TypeDef pid_speed_left  = { .Ks = 80.0f, .Kf = 4.98f, .Ka = 0.174f, .Kp = 1.5f, .Ki = 30.0f, .Kd = 0.0f, .out_min = -PWM_MAX, .out_max = PWM_MAX };
PID_TypeDef pid_speed_right = { .Ks = 85.0f, .Kf = 5.12f, .Ka = 0.176f, .Kp = 1.5f, .Ki = 30.0f, .Kd = 0.0f, .out_min = -PWM_MAX, .out_max = PWM_MAX };
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
float enc_heading_deg = 0.0f;
typedef enum { MOTION_IDLE, MOTION_STRAIGHT, MOTION_TURN } MotionMode;
typedef enum { MOTION_OK, MOTION_RUNNING, MOTION_ABORT, MOTION_TIMEOUT, MOTION_NOT_READY } MotionResult;
static volatile MotionMode   motion_mode   = MOTION_IDLE;
static volatile MotionResult motion_result = MOTION_OK;
static volatile float heading_target   = 0.0f;   // Hướng cần giữ / cần quay tới (độ, TUYỆT ĐỐI -> sai số không cộng dồn)
static volatile float move_start_mm    = 0.0f;   // Quãng đường tâm xe lúc bắt đầu đoạn đi thẳng
static volatile float move_target_mm   = 0.0f;
static volatile float motion_abort_deg = MOTION_ABORT_DEG;
static volatile float motion_max_err   = 0.0f;   // Báo cáo: đi thẳng = |lệch hướng| lớn nhất, quay = góc vượt quá đích lớn nhất
static volatile float motion_turn_dir  = 1.0f;   // Chiều quay đang chạy: 1 trái, -1 phải
static volatile float heading_integral = 0.0f;   // Tích phân lệch hướng của PID giữ hướng (độ.s)
static volatile float motion_w_corr    = 0.0f;   // Tốc độ sửa hướng PID vừa ra (độ/s), để in log
static uint8_t heading_set = 0;                  // 0: chưa lấy hướng gốc

// Đo ngắt TIM9 bằng bộ đếm chu kỳ CPU (DWT->CYCCNT, 60MHz): chu kỳ giữa 2 lần ngắt + thời gian xử lý.
// Chứng minh in UART (luồng chính) có làm trễ / chen vào vòng điều khiển hay không
typedef struct {
    uint32_t n;
    uint32_t period_min, period_max;      // Chu kỳ CPU giữa 2 lần vào Motor_Speed_Tick (chuẩn 120000 = 2ms)
    uint32_t exec_max;                    // Chu kỳ CPU xử lý Motor_Speed_Tick lâu nhất
} TickStat;
static volatile TickStat tick_stat;
static uint32_t tick_last_cyc = 0;
static void TickStat_Reset(void)
{
    tick_stat.n = 0;                      // Ngắt thấy n = 0 thì chỉ lấy mốc, chưa tính chu kỳ
    tick_stat.period_min = UINT32_MAX;
    tick_stat.period_max = 0;
    tick_stat.exec_max = 0;
}
// Ghi dữ liệu cho Straight_IMU_Test (ngắt TIM9 ghi 1 mẫu / 2ms khi imu_rec_n < IMU_REC_MAX)
typedef struct {
    int16_t g[3], a[3];                   // Gyro x y z, accel x y z (số thô MPU)
    int16_t w_pid;                        // Tốc độ sửa hướng PID x10 (độ/s)
    int8_t  enc_l, enc_r;                 // Xung encoder trong 2ms này
    uint8_t phase;                        // Giai đoạn test (REC_*)
    uint8_t fresh;                        // 1: IMU có mẫu mới kể từ lần ghi trước (0 = lặp mẫu cũ)
} ImuRec;
enum { REC_STILL, REC_UART, REC_MOVE, REC_AFTER, REC_PHASES };
// Ghi dữ liệu cho Heading_PID_Test (mỗi 2ms khi head_rec_n < HEAD_REC_MAX), đơn vị nguyên để tiết kiệm RAM
typedef struct {
    int16_t ref;                          // Bậc hướng đích so với hướng gốc (0.01 độ)
    int16_t y;                            // Hướng thật theo gyro so với hướng gốc (0.01 độ)
    int16_t enc;                          // Hướng theo encoder so với lúc xuất phát (0.01 độ)
    int16_t rate;                         // Tốc độ quay gyro (0.1 độ/s)
    int16_t w_pid;                        // Tốc độ sửa hướng PID ra (0.1 độ/s)
} HeadRec;
// 2 bài test không chạy cùng lúc -> dùng chung 1 vùng nhớ (54KB)
static union {
    ImuRec  imu[IMU_REC_MAX];
    HeadRec head[HEAD_REC_MAX];
} rec_buf;
static volatile uint16_t imu_rec_n = IMU_REC_MAX; // = IMU_REC_MAX: không ghi
static volatile uint8_t  imu_rec_phase = REC_STILL;
static uint32_t imu_rec_last_count = 0;
static volatile uint16_t head_rec_n = HEAD_REC_MAX;   // = HEAD_REC_MAX: không ghi
static float head_enc0 = 0.0f;                        // enc_heading_deg lúc bắt đầu ghi

// PID giữ hướng đang dùng: khởi đầu theo pid.h, Heading_PID_Test đổi được qua Bluetooth
float heading_kp = HEADING_KP, heading_ki = HEADING_KI, heading_kd = HEADING_KD;
static volatile float heading_step_deg = 0.0f;   // Heading_PID_Test: bậc cộng thêm vào hướng đích khi đi thẳng
static HAL_StatusTypeDef rezero_status = HAL_OK; // Kết quả chỉnh offset gyro lần gần nhất (Motion_Begin)

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
// Xuất duty cho cả 2 động cơ, dương = chạy tới, 0 = phanh
void Motor_SetPWM(int32_t left, int32_t right)
{
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
    // Bật bộ đếm chu kỳ CPU (DWT) để đo ngắt TIM9 - chạy được cả khi không cắm mạch nạp
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    TickStat_Reset();
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
static float Clamp(float x, float limit);
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
    // Góc quay chỉ theo encoder (so sánh với gyro): chênh quãng đường 2 bánh / khoảng cách 2 bánh
    enc_heading_deg += (raw_right - raw_left) * CONTROL_DT / TRACK_WIDTH_MM / DEG2RAD;
    float d_center = 0.5f * (raw_left + raw_right) * CONTROL_DT;
    if (d_center != 0.0f) {               // Vị trí ước lượng: tâm xe đi d_center theo hướng gyro
        float h = Robot_Heading() * DEG2RAD;
        odo_x_mm += d_center * cosf(h);
        odo_y_mm += d_center * sinf(h);
    }

    // 1c. Vòng ngoài đang đi thẳng / quay thì nó đặt vận tốc cho 2 bánh
    Motion_Update();

    // 1d. Straight_IMU_Test đang ghi: lưu mẫu IMU mới nhất + encoder + PID hướng
    if (imu_rec_n < IMU_REC_MAX) {
        ImuRec *r = &rec_buf.imu[imu_rec_n];
        uint32_t cnt = imu_sample_count;
        r->fresh = (cnt != imu_rec_last_count);
        imu_rec_last_count = cnt;
        for (int i = 0; i < 3; i++) {
            r->a[i] = imu_raw[i];
            r->g[i] = imu_raw[4 + i];
        }
        r->w_pid = (int16_t)(motion_w_corr * 10.0f);
        r->enc_l = (int8_t)((delta_left  > 127) ? 127 : (delta_left  < -127) ? -127 : delta_left);
        r->enc_r = (int8_t)((delta_right > 127) ? 127 : (delta_right < -127) ? -127 : delta_right);
        r->phase = imu_rec_phase;
        imu_rec_n++;
    }
    // 1e. Heading_PID_Test đang ghi
    if (head_rec_n < HEAD_REC_MAX) {
        HeadRec *h = &rec_buf.head[head_rec_n];
        h->ref   = (int16_t)(heading_step_deg * 100.0f);
        h->y     = (int16_t)Clamp((Robot_Heading() - heading_target) * 100.0f, 32000.0f);
        h->enc   = (int16_t)Clamp((enc_heading_deg - head_enc0) * 100.0f, 32000.0f);
        h->rate  = (int16_t)Clamp(yaw_rate_dps * 10.0f, 32000.0f);
        h->w_pid = (int16_t)(motion_w_corr * 10.0f);
        head_rec_n++;
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
    uint32_t c0 = DWT->CYCCNT;            // Đo chu kỳ ngắt (xem TickStat)
    if (tick_stat.n++ > 0) {
        uint32_t period = c0 - tick_last_cyc;
        if (period < tick_stat.period_min) tick_stat.period_min = period;
        if (period > tick_stat.period_max) tick_stat.period_max = period;
    }
    tick_last_cyc = c0;

    uint16_t curr_cnt_left = TIM3->CNT;
    uint16_t curr_cnt_right = TIM2->CNT;

    int16_t delta_pulse_left = (int16_t)(curr_cnt_left - prev_cnt_left);
    int16_t delta_pulse_right = (int16_t)(curr_cnt_right - prev_cnt_right);

    prev_cnt_left = curr_cnt_left;
    prev_cnt_right = curr_cnt_right;

    Motor_Speed_Control(delta_pulse_left, delta_pulse_right);

    uint32_t exec = DWT->CYCCNT - c0;
    if (exec > tick_stat.exec_max) tick_stat.exec_max = exec;
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
    return YAW_SCALE * yaw_angle;
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
static void Motion_Update(void)
{
    static uint16_t settle_ticks = 0;
    if (motion_mode == MOTION_IDLE) {
        settle_ticks = 0;
        return;
    }

    // Dương: cần quay sang trái. Đi thẳng thì cộng thêm bậc thử của Heading_PID_Test (bình thường = 0)
    float head_err = heading_target + ((motion_mode == MOTION_STRAIGHT) ? heading_step_deg : 0.0f) - Robot_Heading();
    float v = 0.0f, w_dps = 0.0f;
    uint8_t arrived;

    float err_stat = (motion_mode == MOTION_STRAIGHT) ? fabsf(head_err) : -motion_turn_dir * head_err;
    if (err_stat > motion_max_err) motion_max_err = err_stat;
    if (fabsf(head_err) > motion_abort_deg) {             // Sai YAW_SIGN / bị va, kẹt: dừng
        motion_mode   = MOTION_IDLE;
        motion_result = MOTION_ABORT;
        target_speed_left = target_speed_right = 0.0f;
        return;
    }
    if (motion_mode == MOTION_STRAIGHT) {
        float remaining = move_target_mm - (Robot_Distance() - move_start_mm);
        arrived = fabsf(remaining) < MOVE_TOL_MM;
        if (!arrived) {
            v = Motion_Profile(remaining, MOVE_SPEED_MMS, MOVE_DECEL_MMS2);
            // PID giữ hướng: P theo lệch, I xóa lệch còn lại, D lấy thẳng tốc độ quay gyro
            if (heading_ki > 0.001f)
                heading_integral = Clamp(heading_integral + head_err * CONTROL_DT, HEADING_I_MAX_DPS / heading_ki);
            else
                heading_integral = 0.0f;
            w_dps = Clamp(heading_kp * head_err + heading_ki * heading_integral - heading_kd * yaw_rate_dps,
                          HEADING_MAX_DPS);
        }
    } else {
        arrived = fabsf(head_err) < TURN_TOL_DEG;
        if (!arrived) w_dps = Motion_Profile(head_err, TURN_SPEED_DPS, TURN_DECEL_DPS2);
    }

    // Quay trái (w > 0): bánh phải nhanh hơn bánh trái. Tới đích thì đặt đúng 0 -> PID phanh hẳn
    motion_w_corr = w_dps;
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
    rezero_status = MPU6500_ZeroGyroZ();
    heading_target = Robot_Heading();
    odo_x_mm = 0.0f;
    odo_y_mm = 0.0f;
    heading_set = 1;
}
// Bắt đầu 1 đoạn (không chờ). 0: động cơ bị khóa / IMU chưa sẵn sàng
static uint8_t Motion_Start(MotionMode mode)
{
    if (!motor_hw_ok || !imu_ready) {
        motion_result = MOTION_NOT_READY;
        return 0;
    }
    motion_max_err   = 0.0f;
    heading_integral = 0.0f;
    motion_w_corr    = 0.0f;
    heading_step_deg = 0.0f;
    motion_result    = MOTION_RUNNING;
    motion_mode      = mode;              // Ghi cuối cùng: các thông số đã sẵn sàng khi ngắt đọc
    return 1;
}
// Chờ ngắt TIM9 báo xong. hook != NULL: gọi liên tục trong lúc chờ (vd. in log), không được chặn lâu
static uint8_t Motion_Wait(uint32_t timeout_ms, void (*hook)(void))
{
    uint32_t t0 = HAL_GetTick();
    while (motion_mode != MOTION_IDLE) {
        if (HAL_GetTick() - t0 > timeout_ms) {
            Motor_SetSpeed(0.0f, 0.0f);       // Hủy, giảm tốc rồi dừng
            motion_result = MOTION_TIMEOUT;
            break;
        }
        if (hook) hook();
    }
    if (motion_result == MOTION_ABORT) Motor_Stop();   // Lệch hướng bất thường: phanh ngay
    return motion_result == MOTION_OK;
}
// Chuẩn bị đoạn đi thẳng, trả về thời gian tối đa: MOTION_TIMEOUT_MS + 2 lần thời gian chạy hết đoạn
// ở tốc độ tối đa (đoạn dài không bị cắt)
static uint32_t Straight_Prepare(float dist_mm)
{
    if (!heading_set) Motion_Begin();
    Motor_SetSpeed(0.0f, 0.0f);           // Bật PID vận tốc (vòng ngoài sẽ đặt vận tốc)
    move_start_mm    = Robot_Distance();
    move_target_mm   = dist_mm;
    motion_abort_deg = MOTION_ABORT_DEG;
    return MOTION_TIMEOUT_MS + (uint32_t)(2000.0f * fabsf(dist_mm) / MOVE_SPEED_MMS);
}
uint8_t Move_Straight(float dist_mm)
{
    uint32_t timeout_ms = Straight_Prepare(dist_mm);
    if (!Motion_Start(MOTION_STRAIGHT)) return 0;
    return Motion_Wait(timeout_ms, NULL);
}
uint8_t Turn_Angle(float deg)
{
    if (!heading_set) Motion_Begin();
    Motor_SetSpeed(0.0f, 0.0f);
    heading_target  += deg;
    motion_turn_dir  = (deg >= 0.0f) ? 1.0f : -1.0f;
    motion_abort_deg = fabsf(deg) + MOTION_ABORT_DEG;   // Quay ngược chiều quá MOTION_ABORT_DEG -> sai dấu gyro
    if (!Motion_Start(MOTION_TURN)) return 0;
    return Motion_Wait(MOTION_TIMEOUT_MS + (uint32_t)(2000.0f * fabsf(deg) / TURN_SPEED_DPS), NULL);
}
uint8_t Turn_Left(float deg)  { return Turn_Angle(deg); }
uint8_t Turn_Right(float deg) { return Turn_Angle(-deg); }

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
                                                 "HUY: qua thoi gian (ket banh?)", "HUY: chua san sang" };
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
            ok = Move_Straight(s->target);
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
void Straight_Test(float dist_mm)
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
    Move_Straight(dist_mm);
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
             dist_mm, MOVE_SPEED_MMS, MOVE_DECEL_MMS2, MOVE_TOL_MM);
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

// ===== ĐI THẲNG + IN LOG QUA UART (HC-05) =====
// Luồng chính in, ngắt TIM9 vẫn điều khiển xe -> in chậm (UART chờ) cũng không ảnh hưởng chuyển động
static TickStat TickStat_Get(void);
static void TickStat_Print(const char *name, const TickStat *s);
static struct {
    uint32_t t0, t_prev, next;
    float    s0, yaw0, enc0;              // Giá trị lúc bắt đầu
    float    yaw_prev, enc_prev;          // Dòng trước, để tính tốc độ quay trung bình giữa 2 dòng
    float    sum_dw, sum_dw2;             // Thống kê (w_gyro - w_enc) lúc xe đang chạy
    uint32_t n_dw;
} ulog;
// In 1 dòng: tốc độ quay gyro và encoder tính trên CÙNG khoảng thời gian giữa 2 dòng (so sánh công bằng;
// tốc độ quay encoder tức thời rất nhiễu vì 1 xung / 2ms đã là ~67 mm/s)
static void Log_UART_Line(void)
{
    uint32_t now = HAL_GetTick();
    float yaw = Robot_Heading()  - ulog.yaw0;
    float enc = enc_heading_deg  - ulog.enc0;
    float dt  = (now - ulog.t_prev) / 1000.0f;
    float w_gyro = (dt > 0.0f) ? (yaw - ulog.yaw_prev) / dt : 0.0f;
    float w_enc  = (dt > 0.0f) ? (enc - ulog.enc_prev) / dt : 0.0f;
    ulog.t_prev   = now;
    ulog.yaw_prev = yaw;
    ulog.enc_prev = enc;
    if (fabsf(speed_left_mms) + fabsf(speed_right_mms) > 2.0f * MOTION_STOP_MMS) {
        float dw = w_gyro - w_enc;
        ulog.sum_dw  += dw;
        ulog.sum_dw2 += dw * dw;
        ulog.n_dw++;
    }
    snprintf(msg, sizeof(msg), "LOG,%lu,%.1f,%.0f,%.0f,%+.2f,%+.2f,%+.1f,%+.1f,%+.2f,%+.1f\r\n",
             (unsigned long)(now - ulog.t0), Robot_Distance() - ulog.s0, speed_left_mms, speed_right_mms,
             yaw, enc, w_gyro, w_enc, heading_target - Robot_Heading(), motion_w_corr);
    UART_Print(msg);
}
// Gọi liên tục trong lúc chờ xe chạy: đến hạn thì in 1 dòng (in chậm hơn chu kỳ thì bỏ qua, không dồn)
static void Log_UART_Hook(void)
{
    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - ulog.next) < 0) return;
    ulog.next += LOG_UART_PERIOD_MS;
    if ((int32_t)(now - ulog.next) >= 0) ulog.next = now + LOG_UART_PERIOD_MS;
    Log_UART_Line();
}
// ĐI THẲNG dist_mm, giữ hướng bằng PID gyro, VỪA CHẠY VỪA IN qua USART1 (HC-05).
// Chờ nhận 1 ký tự bất kỳ qua UART (gửi từ app Bluetooth) rồi đứng yên 1s (IMU chỉnh offset) mới chạy.
// Mỗi dòng LOG (CSV, vẽ được bằng Excel / Serial Plotter):
//   t_ms, s_mm (quãng đường tâm xe), vT/vP (vận tốc bánh trái/phải mm/s),
//   yaw_gyro / yaw_enc (góc quay theo gyro / theo encoder, độ),
//   w_gyro / w_enc (tốc độ quay theo gyro / theo encoder, độ/s), lech_huong (độ), w_pid (độ/s PID sửa hướng)
void Straight_Log_UART(float dist_mm)
{
    if (!motor_hw_ok || !imu_ready) {
        UART_Print("\r\nLOI: dong co bi khoa (CubeMX) hoac IMU chua san sang (MPU6500_Setup) - khong chay duoc\r\n");
        HAL_Delay(1000);
        return;
    }
    snprintf(msg, sizeof(msg), "\r\nSan sang di thang %.0f mm. Gui 1 ky tu bat ky de chay...\r\n", dist_mm);
    UART_WaitStart(msg);

    // 1. Đứng yên 1s (kịp bỏ tay ra, IMU chỉnh offset), lấy gốc
    Motor_SetSpeed(0.0f, 0.0f);
    HAL_Delay(1000);
    Motion_Begin();
    uint32_t timeout_ms = Straight_Prepare(dist_mm);
    float l0 = dist_left_mm, r0 = dist_right_mm;
    ulog.t0 = ulog.t_prev = ulog.next = HAL_GetTick();
    ulog.s0 = Robot_Distance();
    ulog.yaw0 = Robot_Heading();
    ulog.enc0 = enc_heading_deg;
    ulog.yaw_prev = ulog.enc_prev = 0.0f;
    ulog.sum_dw = ulog.sum_dw2 = 0.0f;
    ulog.n_dw = 0;

    UART_Print("================ DI THANG + LOG GYRO / ENCODER ================\r\n");
    snprintf(msg, sizeof(msg), "Dich %.0f mm, toi da %.0f mm/s | PID huong Kp=%.1f Ki=%.1f Kd=%.2f | TRACK_WIDTH %.1f mm\r\n",
             dist_mm, MOVE_SPEED_MMS, HEADING_KP, HEADING_KI, HEADING_KD, TRACK_WIDTH_MM);
    UART_Print(msg);
    UART_Print("LOG,t_ms,s_mm,vT_mms,vP_mms,yaw_gyro,yaw_enc,w_gyro,w_enc,lech_huong,w_pid\r\n");

    // 2. Chạy, trong lúc chờ thì in log
    TickStat_Reset();
    if (Motion_Start(MOTION_STRAIGHT)) Motion_Wait(timeout_ms, Log_UART_Hook);
    TickStat tick_move = TickStat_Get();
    Log_UART_Line();                      // Dòng cuối: lúc đã dừng

    // 3. Tổng kết
    float dl = dist_left_mm - l0, dr = dist_right_mm - r0, dc = 0.5f * (dl + dr);
    float yaw = Robot_Heading() - ulog.yaw0, enc = enc_heading_deg - ulog.enc0;
    float mean = (ulog.n_dw > 0) ? ulog.sum_dw / ulog.n_dw : 0.0f;
    float var  = (ulog.n_dw > 0) ? ulog.sum_dw2 / ulog.n_dw - mean * mean : 0.0f;
    UART_Print("---------------- TONG KET ----------------\r\n");
    snprintf(msg, sizeof(msg), "Ket qua: %s | %lu ms | quang duong tam %+.1f mm (trai %+.1f, phai %+.1f)\r\n",
             motion_result_str[motion_result], (unsigned long)(HAL_GetTick() - ulog.t0), dc, dl, dr);
    UART_Print(msg);
    snprintf(msg, sizeof(msg), "Goc quay: gyro %+.2f do | encoder %+.2f do | chenh %+.2f do\r\n", yaw, enc, enc - yaw);
    UART_Print(msg);
    snprintf(msg, sizeof(msg), "Toc do quay (encoder - gyro) luc chay: TB %+.2f do/s, lech chuan %.2f do/s (%lu mau)\r\n",
             -mean, (var > 0.0f) ? sqrtf(var) : 0.0f, (unsigned long)ulog.n_dw);
    UART_Print(msg);
    snprintf(msg, sizeof(msg), "Lech huong: lon nhat %.2f do, luc dung %+.2f do\r\n",
             motion_max_err, heading_target - Robot_Heading());
    UART_Print(msg);
    TickStat_Print("luc chay + in UART", &tick_move);
    // Gyro giữ xe thẳng mà encoder báo đã quay = 2 bánh đi được quãng đường khác nhau
    if (fabsf(dc) > 1.0f) {
        snprintf(msg, sizeof(msg), "Banh phai di nhieu hon banh trai %+.1f mm (%+.2f%%)%s\r\n", dr - dl,
                 (dr - dl) * 100.0f / fabsf(dc),
                 (fabsf(dr - dl) > 0.005f * fabsf(dc)) ? " -> duong kinh 2 banh khac nhau hoac 1 banh truot" : " -> 2 banh deu");
        UART_Print(msg);
        // Phần encoder quay mà gyro không quay = chênh quãng đường do lệch đường kính. Chia đều cho 2 bánh
        // (giữ trung bình): encoder báo quay trái nhiều hơn thật -> bánh phải khai báo to hơn thật -> giảm phải, tăng trái
        float k = 0.5f * (enc - yaw) * DEG2RAD * TRACK_WIDTH_MM / dc;
        snprintf(msg, sizeof(msg), "De xuat (tin gyro, nen lay TB vai lan): WHEEL_DIAMETER_L = %.2f, WHEEL_DIAMETER_R = %.2f\r\n",
                 WHEEL_DIAMETER_L * (1.0f + k), WHEEL_DIAMETER_R * (1.0f - k));
        UART_Print(msg);
        snprintf(msg, sizeof(msg), "Quang duong: do bang thuoc roi nhan CA 2 duong kinh voi (quang duong that / %.1f)\r\n", fabsf(dc));
        UART_Print(msg);
    }
    UART_Print("================ HET ================\r\n");
}

// ===== ĐI THẲNG + GHI DỮ LIỆU RUNG MPU VÀO RAM (Straight_IMU_Test) =====
static TickStat TickStat_Get(void)        // Chụp số đo ngắt TIM9 hiện tại
{
    TickStat s;
    s.n = tick_stat.n;
    s.period_min = tick_stat.period_min;
    s.period_max = tick_stat.period_max;
    s.exec_max = tick_stat.exec_max;
    return s;
}
static void TickStat_Print(const char *name, const TickStat *s)
{
    float cyc_per_us = SystemCoreClock / 1000000.0f;
    snprintf(msg, sizeof(msg), "Ngat TIM9 %s: %lu lan, chu ky %.1f..%.1f us (chuan 2000), xu ly lau nhat %.1f us (%.1f%% CPU)\r\n",
             name, (unsigned long)s->n, s->period_min / cyc_per_us, s->period_max / cyc_per_us,
             s->exec_max / cyc_per_us, s->exec_max / cyc_per_us / 20.0f);
    UART_Print(msg);
}
// Kênh ch của 1 mẫu: 0..2 gyro x y z, 3..5 accel x y z (số thô)
static float Rec_Val(const ImuRec *r, int ch)
{
    return (ch < 3) ? r->g[ch] : r->a[ch - 3];
}
// Thống kê các mẫu [i0, i1): trung bình + độ lệch chuẩn từng kênh (số thô, tính 2 lượt cho chính xác)
typedef struct {
    uint32_t n, fresh;
    float mean[6], std[6];
    int32_t enc_abs;                      // Tổng |xung| 2 bánh: đứng yên phải = 0
    float v_mms;                          // Vận tốc tâm xe trung bình
} RecStat;
static void Rec_Stats(uint32_t i0, uint32_t i1, RecStat *s)
{
    memset(s, 0, sizeof(*s));
    if (i1 <= i0) return;
    s->n = i1 - i0;
    float dist = 0.0f;
    for (uint32_t i = i0; i < i1; i++) {
        const ImuRec *r = &rec_buf.imu[i];
        for (int c = 0; c < 6; c++) s->mean[c] += Rec_Val(r, c);
        s->fresh   += r->fresh;
        s->enc_abs += abs(r->enc_l) + abs(r->enc_r);
        dist += 0.5f * (ENC_L_SIGN * r->enc_l * MM_PER_COUNT_L + ENC_R_SIGN * r->enc_r * MM_PER_COUNT_R);
    }
    for (int c = 0; c < 6; c++) s->mean[c] /= s->n;
    for (uint32_t i = i0; i < i1; i++) {
        for (int c = 0; c < 6; c++) {
            float d = Rec_Val(&rec_buf.imu[i], c) - s->mean[c];
            s->std[c] += d * d;
        }
    }
    for (int c = 0; c < 6; c++) s->std[c] = sqrtf(s->std[c] / s->n);
    s->v_mms = dist / (s->n * CONTROL_DT);
}
static void Rec_PrintRow(const char *name, const RecStat *s)
{
    const float G = 1.0f / 16.4f, A = 1000.0f / 16384.0f;   // Số thô -> độ/s, mg
    snprintf(msg, sizeof(msg), "%-16s %4lu %4lu %+7.3f %6.3f %6.3f %6.3f %6.2f %6.2f %6.2f %5ld %6.0f\r\n",
             name, (unsigned long)s->n, (unsigned long)s->fresh,
             (s->mean[2] - gyro_z_offset) * G, s->std[2] * G, s->std[0] * G, s->std[1] * G,
             s->std[3] * A, s->std[4] * A, s->std[5] * A, (long)s->enc_abs, s->v_mms);
    UART_Print(msg);
}
// Biên độ thành phần tần số f_hz của kênh ch trong [i0, i1) (thuật toán Goertzel, lấy mẫu 500Hz)
static float Rec_Goertzel(uint32_t i0, uint32_t i1, int ch, float mean, float f_hz)
{
    float coeff = 2.0f * cosf(2.0f * 3.14159265f * f_hz * CONTROL_DT);
    float s1 = 0.0f, s2 = 0.0f;
    for (uint32_t i = i0; i < i1; i++) {
        float s = Rec_Val(&rec_buf.imu[i], ch) - mean + coeff * s1 - s2;
        s2 = s1;
        s1 = s;
    }
    float power = s1 * s1 + s2 * s2 - coeff * s1 * s2;
    return (power > 0.0f) ? 2.0f * sqrtf(power) / (i1 - i0) : 0.0f;
}
// Quét 0.5..100Hz (bước 0.25Hz), in 3 đỉnh lớn nhất của kênh ch. scale: số thô -> đơn vị in
static void Rec_PrintPeaks(uint32_t i0, uint32_t i1, int ch, float mean, float scale, const char *name)
{
    float pf[3] = { 0, 0, 0 }, pa[3] = { 0, 0, 0 };
    float a_prev2 = 0.0f, a_prev = 0.0f;
    for (int k = 2; k <= 401; k++) {
        float a = Rec_Goertzel(i0, i1, ch, mean, k * 0.25f) * scale;
        if (k >= 4 && a_prev > a_prev2 && a_prev >= a) {          // Đỉnh tại k-1
            float f = (k - 1) * 0.25f;
            for (int j = 0; j < 3; j++) {                         // Chèn vào top 3 theo biên độ
                if (a_prev > pa[j]) {
                    for (int m = 2; m > j; m--) { pa[m] = pa[m - 1]; pf[m] = pf[m - 1]; }
                    pa[j] = a_prev;
                    pf[j] = f;
                    break;
                }
            }
        }
        a_prev2 = a_prev;
        a_prev = a;
    }
    snprintf(msg, sizeof(msg), "  %-14s %6.2f Hz (%.3f) | %6.2f Hz (%.3f) | %6.2f Hz (%.3f)\r\n",
             name, pf[0], pa[0], pf[1], pa[1], pf[2], pa[2]);
    UART_Print(msg);
}
// Chờ tới khi ngắt ghi đủ n mẫu (hoặc đầy bộ nhớ). Có giới hạn thời gian: ngắt TIM9 không chạy thì không treo
static void Rec_WaitCount(uint32_t n)
{
    if (n > IMU_REC_MAX) n = IMU_REC_MAX;
    uint32_t t0 = HAL_GetTick();
    uint32_t limit_ms = (n > imu_rec_n) ? (n - imu_rec_n) * 2U + 1000U : 0U;
    while (imu_rec_n < n && HAL_GetTick() - t0 < limit_ms) {}
}
void Straight_IMU_Test(float dist_mm, uint8_t dump_raw)
{
    static const char *phase_name[REC_PHASES] = { "1.DungYen", "2.DungYen+UART", "3.Chay", "4.SauKhiDung" };
    if (!motor_hw_ok || !imu_ready) {
        UART_Print("\r\nLOI: dong co bi khoa (CubeMX) hoac IMU chua san sang (MPU6500_Setup) - khong chay duoc\r\n");
        HAL_Delay(1000);
        return;
    }
    snprintf(msg, sizeof(msg), "\r\nTest rung MPU: di thang %.0f mm. Gui 1 ky tu bat ky de chay...\r\n", dist_mm);
    UART_WaitStart(msg);

    // 1. Đứng yên 1s (bỏ tay ra, IMU chỉnh offset), lấy gốc
    Motor_SetSpeed(0.0f, 0.0f);
    HAL_Delay(1000);
    Motion_Begin();
    uint32_t err0 = MPU6500_ErrorCount(), cnt0 = imu_sample_count;

    // 2. Ghi 0.5s đứng yên IM LẶNG (không gửi gì) -> mức nhiễu nền
    TickStat_Reset();
    imu_rec_last_count = imu_sample_count;
    imu_rec_phase = REC_STILL;
    imu_rec_n = 0;
    Rec_WaitCount(IMU_REC_STILL);
    TickStat tick_still = TickStat_Get();

    // 3. Ghi 0.5s đứng yên, GỬI UART LIÊN TỤC -> UART/Bluetooth có làm nhiễu encoder, IMU, ngắt không
    TickStat_Reset();
    imu_rec_phase = REC_UART;
    uint32_t t_uart = HAL_GetTick();
    while (imu_rec_n < 2 * IMU_REC_STILL && HAL_GetTick() - t_uart < 3000U)
        UART_Print("NHIEU_UART,UUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUU\r\n");
    TickStat tick_uart = TickStat_Get();

    // 4. Chạy thẳng, KHÔNG gửi gì trong lúc chạy
    uint32_t timeout_ms = Straight_Prepare(dist_mm);
    float l0 = dist_left_mm, r0 = dist_right_mm;
    TickStat_Reset();
    imu_rec_phase = REC_MOVE;
    if (Motion_Start(MOTION_STRAIGHT)) Motion_Wait(timeout_ms, NULL);
    TickStat tick_move = TickStat_Get();
    MotionResult result = motion_result;

    // 5. Thêm 0.3s sau khi dừng rồi ngừng ghi
    imu_rec_phase = REC_AFTER;
    Rec_WaitCount((uint32_t)imu_rec_n + 150);
    uint32_t n = imu_rec_n;
    imu_rec_n = IMU_REC_MAX;              // Ngừng ghi
    float dl = dist_left_mm - l0, dr = dist_right_mm - r0;
    uint32_t imu_got = imu_sample_count - cnt0, err = MPU6500_ErrorCount() - err0;

    // 6. Tách giai đoạn (các giai đoạn ghi nối tiếp nhau)
    uint32_t p0[REC_PHASES], p1[REC_PHASES];
    for (int p = 0; p < REC_PHASES; p++) { p0[p] = n; p1[p] = 0; }
    for (uint32_t i = 0; i < n; i++) {
        int p = rec_buf.imu[i].phase;
        if (i < p0[p]) p0[p] = i;
        if (i + 1 > p1[p]) p1[p] = i + 1;
    }
    for (int p = 0; p < REC_PHASES; p++) if (p1[p] < p0[p]) p0[p] = p1[p];
    // Chạy đều = 60% giữa đoạn chạy (bỏ lúc tăng tốc / giảm tốc)
    uint32_t mlen = p1[REC_MOVE] - p0[REC_MOVE];
    uint32_t c0 = p0[REC_MOVE] + mlen / 5, c1 = p1[REC_MOVE] - mlen / 5;
    RecStat st[REC_PHASES], cruise;
    for (int p = 0; p < REC_PHASES; p++) Rec_Stats(p0[p], p1[p], &st[p]);
    Rec_Stats(c0, c1, &cruise);

    // 7. Báo cáo
    UART_Print("\r\n================ TEST RUNG MPU KHI DI THANG ================\r\n");
    snprintf(msg, sizeof(msg), "Dich %.0f mm | ghi %lu mau x 2ms%s | PID huong Kp=%.1f Ki=%.1f Kd=%.2f | DLPF gyro %d accel %d\r\n",
             dist_mm, (unsigned long)n, (n >= IMU_REC_MAX) ? " (DAY BO NHO)" : "",
             HEADING_KP, HEADING_KI, HEADING_KD, GYRO_DLPF_CFG, ACCEL_DLPF_CFG);
    UART_Print(msg);
    snprintf(msg, sizeof(msg), "Ket qua: %s | quang duong tam %+.1f mm (trai %+.1f, phai %+.1f) | huong lech luc dung %+.2f do\r\n",
             motion_result_str[result], 0.5f * (dl + dr), dl, dr, heading_target - Robot_Heading());
    UART_Print(msg);
    UART_Print("Giai doan         Mau IMUmoi  GzTB     Gz     Gx     Gy     Ax     Ay     Az  Xung V(mm/s)\r\n");
    Rec_PrintRow(phase_name[REC_STILL], &st[REC_STILL]);
    Rec_PrintRow(phase_name[REC_UART],  &st[REC_UART]);
    Rec_PrintRow(phase_name[REC_MOVE],  &st[REC_MOVE]);
    Rec_PrintRow("3b.Chay deu", &cruise);
    Rec_PrintRow(phase_name[REC_AFTER], &st[REC_AFTER]);
    UART_Print("Don vi: GzTB = Gz TB tru offset (do/s) | Gz Gx Gy = do lech chuan (do/s) | Ax Ay Az = do lech chuan (mg)\r\n");
    UART_Print("        IMUmoi = so mau IMU moi / so lan ghi | Xung = tong |xung| encoder 2 banh (dung yen phai = 0)\r\n");
    UART_Print("        3b = 60% giua doan chay (bo tang/giam toc). Cot giai_doan trong RAW: 1..4 nhu tren\r\n");

    // Nhận xét
    const RecStat *q = &st[REC_STILL];
    float gz0 = (q->std[2] > 0.0f) ? q->std[2] : 1.0f, az0 = (q->std[5] > 0.0f) ? q->std[5] : 1.0f;
    float gx0 = (q->std[0] > 0.0f) ? q->std[0] : 1.0f, gy0 = (q->std[1] > 0.0f) ? q->std[1] : 1.0f;
    UART_Print("---------------- NHAN XET ----------------\r\n");
    snprintf(msg, sizeof(msg), "UART/Bluetooth -> encoder: %ld xung khi dung yen luc gui UART %s\r\n", (long)st[REC_UART].enc_abs,
             (st[REC_UART].enc_abs == 0) ? "-> OK, khong nhieu" : "-> NHIEU LOT VAO ENCODER: bat Input Filter TIM2/TIM3 trong CubeMX");
    UART_Print(msg);
    snprintf(msg, sizeof(msg), "UART/Bluetooth -> IMU: nhieu Gz x%.2f, Az x%.2f so voi dung yen im lang (~1.0 = khong anh huong)\r\n",
             st[REC_UART].std[2] / gz0, st[REC_UART].std[5] / az0);
    UART_Print(msg);
    snprintf(msg, sizeof(msg), "Rung khi chay deu (dong co + banh): Gz x%.1f, Gx x%.1f, Gy x%.1f, Az x%.1f so voi dung yen\r\n",
             cruise.std[2] / gz0, cruise.std[0] / gx0, cruise.std[1] / gy0, cruise.std[5] / az0);
    UART_Print(msg);
    uint32_t fresh = 0;
    for (int p = 0; p < REC_PHASES; p++) fresh += st[p].fresh;
    snprintf(msg, sizeof(msg), "Mau IMU moi: %lu / %lu lan ghi (2ms/lan) | IMU xu ly %lu mau | loi I2C them %lu\r\n",
             (unsigned long)fresh, (unsigned long)n, (unsigned long)imu_got, (unsigned long)err);
    UART_Print(msg);
    TickStat_Print("dung yen     ", &tick_still);
    TickStat_Print("luc gui UART ", &tick_uart);
    TickStat_Print("luc xe chay  ", &tick_move);
    UART_Print("(Chu ky lech <~20 us va xu ly << 2000 us = in UART KHONG lam tre / sai vong dieu khien)\r\n");

    // Tần số rung lúc chạy đều
    if (c1 > c0 + 100) {
        float f_wheel = cruise.v_mms / (3.14159265f * WHEEL_DIAMETER);
        snprintf(msg, sizeof(msg), "---------------- TAN SO RUNG (chay deu, %lu mau, phan giai %.2f Hz) ----------------\r\n",
                 (unsigned long)(c1 - c0), 1.0f / ((c1 - c0) * CONTROL_DT));
        UART_Print(msg);
        snprintf(msg, sizeof(msg), "Du kien: 1 vong banh = %.2f Hz (2 lan = %.2f Hz) | dong co = %.2f Hz x ti so hop so\r\n",
                 f_wheel, 2.0f * f_wheel, f_wheel);
        UART_Print(msg);
        UART_Print("  Kenh           3 dinh lon nhat: tan so (bien do)\r\n");
        Rec_PrintPeaks(c0, c1, 2, cruise.mean[2], 1.0f / 16.4f, "Gz (do/s)");
        Rec_PrintPeaks(c0, c1, 0, cruise.mean[0], 1.0f / 16.4f, "Gx (do/s)");
        Rec_PrintPeaks(c0, c1, 1, cruise.mean[1], 1.0f / 16.4f, "Gy (do/s)");
        Rec_PrintPeaks(c0, c1, 5, cruise.mean[5], 1000.0f / 16384.0f, "Az (mg)");
        Rec_PrintPeaks(c0, c1, 3, cruise.mean[3], 1000.0f / 16384.0f, "Ax (mg)");
        UART_Print("(Gyro loc 41Hz, accel loc 20Hz -> rung tren ~50Hz bi lam yeu, khong thay ro)\r\n");
    }

    // Dữ liệu thô từng mẫu
    if (dump_raw) {
        UART_Print("RAW,i,giai_doan,imu_moi,gx,gy,gz,ax,ay,az,xungT,xungP,wpid_x10\r\n");
        for (uint32_t i = 0; i < n; i++) {
            const ImuRec *r = &rec_buf.imu[i];
            snprintf(msg, sizeof(msg), "RAW,%lu,%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n", (unsigned long)i, r->phase + 1, r->fresh,
                     r->g[0], r->g[1], r->g[2], r->a[0], r->a[1], r->a[2], r->enc_l, r->enc_r, r->w_pid);
            UART_Print(msg);
        }
    }
    UART_Print("================ HET TEST RUNG ================\r\n");
}

// ===== TEST PID GIỮ HƯỚNG (Heading_PID_Test) =====
// Đọc "Kp Ki Kd" (cách nhau bằng dấu cách / phẩy) từ 1 dòng UART. Thiếu số nào giữ số cũ. 1: đã đổi hệ số
static uint8_t HP_ParseGains(const char *line)
{
    float v[3] = { heading_kp, heading_ki, heading_kd };
    const char *p = line;
    int got = 0;
    for (int i = 0; i < 3; i++) {
        char *end;
        float x = strtof(p, &end);
        if (end == p) break;
        v[i] = x;
        got++;
        p = end;
        while (*p == ' ' || *p == ',' || *p == ';' || *p == '\t') p++;
    }
    if (got == 0) return 0;
    if (v[0] < 0.0f || v[0] > 100.0f || v[1] < 0.0f || v[1] > 200.0f || v[2] < 0.0f || v[2] > 5.0f) {
        UART_Print("He so ngoai khoang (Kp 0..100, Ki 0..200, Kd 0..5) -> giu nguyen\r\n");
        return 0;
    }
    heading_kp = v[0];
    heading_ki = v[1];
    heading_kd = v[2];
    return 1;
}
// Lịch bậc hướng đích: chạy HP_T0_MS rồi đổi +STEP, 0, -STEP, 0, mỗi bậc HP_STEP_MS (luồng chính đổi)
static const float hp_steps[4] = { HP_STEP_DEG, 0.0f, -HP_STEP_DEG, 0.0f };
static struct { uint32_t t0; int k; } hp;
static void HP_Hook(void)
{
    uint32_t t = HAL_GetTick() - hp.t0;
    if (t < HP_T0_MS) return;
    int k = (int)((t - HP_T0_MS) / HP_STEP_MS);
    if (k <= 3 && k != hp.k) {
        hp.k = k;
        heading_step_deg = hp_steps[k];
    }
}
// Phân tích 1 bậc [s, e) và in 1 dòng. Đơn vị ghi: 0.01 độ, 0.1 độ/s
static void HP_StepReport(uint32_t s, uint32_t e)
{
    const HeadRec *h = rec_buf.head;
    float old_ref = (s > 0) ? h[s - 1].ref : 0.0f, new_ref = h[s].ref;
    float delta = new_ref - old_ref, dir = (delta >= 0.0f) ? 1.0f : -1.0f;
    int t_rise = -1, t_settle = 0;
    float over = 0.0f, w_max = 0.0f, ess = 0.0f, rms = 0.0f;
    uint32_t len = e - s, n_ss = 0, n_rms = 0;
    for (uint32_t i = s; i < e; i++) {
        float y = h[i].y, err = new_ref - y;
        if (t_rise < 0 && delta != 0.0f && (y - old_ref) / delta >= 0.9f) t_rise = (int)(i - s) * 2;
        if (dir * (y - new_ref) > over) over = dir * (y - new_ref);
        if (fabsf(err) > HP_SETTLE_DEG * 100.0f) t_settle = (int)(i - s + 1) * 2;
        if (fabsf((float)h[i].w_pid) > w_max) w_max = fabsf((float)h[i].w_pid);
        if (i >= e - len * 3 / 10) { ess += err; n_ss++; }
        if (i >= e - len / 2) { rms += err * err; n_rms++; }
    }
    char rise_str[12], settle_str[12];
    if (t_rise >= 0) snprintf(rise_str, sizeof(rise_str), "%d", t_rise); else snprintf(rise_str, sizeof(rise_str), "--");
    if ((uint32_t)t_settle < len * 2) snprintf(settle_str, sizeof(settle_str), "%d", t_settle); else snprintf(settle_str, sizeof(settle_str), "--");
    snprintf(msg, sizeof(msg), "%+5.1f -> %+5.1f  %9s %9.0f %8s %+10.2f %8.2f %10.1f %6lu\r\n",
             old_ref / 100.0f, new_ref / 100.0f, rise_str, (delta != 0.0f) ? over * 100.0f / fabsf(delta) : 0.0f,
             settle_str, (n_ss > 0) ? ess / n_ss / 100.0f : 0.0f, (n_rms > 0) ? sqrtf(rms / n_rms) / 100.0f : 0.0f,
             w_max / 10.0f, (unsigned long)len * 2);
    UART_Print(msg);
}
void Heading_PID_Test(float dist_mm, uint8_t print_raw)
{
    if (!motor_hw_ok || !imu_ready) {
        UART_Print("\r\nLOI: dong co bi khoa (CubeMX) hoac IMU chua san sang (MPU6500_Setup) - khong chay duoc\r\n");
        HAL_Delay(1000);
        return;
    }
    // 1. Nhận hệ số qua Bluetooth (hoặc 1 ký tự = giữ hệ số hiện tại)
    char line[48];
    snprintf(msg, sizeof(msg), "\r\nTest PID huong: gui 'Kp Ki Kd' (vd: 10 15 0.2) hoac 1 ky tu (vd: x) = giu Kp=%.2f Ki=%.2f Kd=%.3f\r\n",
             heading_kp, heading_ki, heading_kd);
    do { UART_Print(msg); } while (UART_ReadLine(line, sizeof(line), 5000) < 0);
    if (HP_ParseGains(line)) UART_Print("Da doi he so.\r\n");
    // Đủ chỗ cho 3 bậc đầu + 0.5s của bậc cuối (+ ~50mm tăng/giảm tốc)
    float need_mm = MOVE_SPEED_MMS * (HP_T0_MS + 3 * HP_STEP_MS + 500) / 1000.0f + 50.0f;
    if (fabsf(dist_mm) < need_mm) {
        snprintf(msg, sizeof(msg), "Canh bao: %.0f mm ngan hon %.0f mm can cho du 4 buoc -> buoc cuoi bi cat\r\n", dist_mm, need_mm);
        UART_Print(msg);
    }

    // 2. Đứng yên 1s, chỉnh offset gyro + lấy gốc
    Motor_SetSpeed(0.0f, 0.0f);
    HAL_Delay(1000);
    Motion_Begin();
    float h0 = heading_target * DEG2RAD;
    uint32_t timeout_ms = Straight_Prepare(dist_mm);
    float l0 = dist_left_mm, r0 = dist_right_mm;

    // 3. Chạy, ghi mỗi 2ms, luồng chính đổi bậc hướng theo lịch (không gửi UART lúc chạy)
    head_enc0 = enc_heading_deg;
    hp.k = -1;
    hp.t0 = HAL_GetTick();
    head_rec_n = 0;
    if (Motion_Start(MOTION_STRAIGHT)) Motion_Wait(timeout_ms, HP_Hook);
    uint32_t n = head_rec_n;
    head_rec_n = HEAD_REC_MAX;            // Ngừng ghi
    heading_step_deg = 0.0f;
    MotionResult result = motion_result;
    float dl = dist_left_mm - l0, dr = dist_right_mm - r0;
    float along   =  odo_x_mm * cosf(h0) + odo_y_mm * sinf(h0);
    float lateral = -odo_x_mm * sinf(h0) + odo_y_mm * cosf(h0);

    // 4. Tìm chỗ đổi bậc
    uint32_t idx[6], n_idx = 0;
    for (uint32_t i = 1; i < n && n_idx < 4; i++)
        if (rec_buf.head[i].ref != rec_buf.head[i - 1].ref) idx[n_idx++] = i;
    idx[n_idx] = n;

    // 5. Báo cáo
    const HeadRec *h = rec_buf.head;
    UART_Print("\r\n================ TEST PID GIU HUONG (MPU6500) ================\r\n");
    snprintf(msg, sizeof(msg), "PID huong: Kp=%.2f Ki=%.2f Kd=%.3f | bac +-%.1f do, moi bac %d ms | %.0f mm/s\r\n",
             heading_kp, heading_ki, heading_kd, HP_STEP_DEG, HP_STEP_MS, MOVE_SPEED_MMS);
    UART_Print(msg);
    snprintf(msg, sizeof(msg), "Chinh offset gyro truoc khi chay: %s, da sua %+.2f do/s\r\n",
             (rezero_status == HAL_OK) ? "OK" : "KHONG DUOC (xe bi cham / chua dung yen)",
             (rezero_status == HAL_OK) ? gyro_z_rezero_dps : 0.0f);
    UART_Print(msg);
    snprintf(msg, sizeof(msg), "Ket qua: %s | quang duong %+.1f mm (trai %+.1f, phai %+.1f) | %lu mau\r\n",
             motion_result_str[result], 0.5f * (dl + dr), dl, dr, (unsigned long)n);
    UART_Print(msg);
    // Giữ thẳng trước bậc đầu (bỏ 0.1s đầu lúc xuất phát)
    uint32_t b1 = (n_idx > 0) ? idx[0] : n, b0 = (b1 > 50) ? 50 : 0;
    float bsum = 0.0f, bsq = 0.0f;
    for (uint32_t i = b0; i < b1; i++) { bsum += h[i].y; bsq += (float)h[i].y * h[i].y; }
    if (b1 > b0) {
        float bm = bsum / (b1 - b0);
        snprintf(msg, sizeof(msg), "Giu thang truoc bac 1: lech TB %+.2f do, RMS %.2f do\r\n",
                 -bm / 100.0f, sqrtf(bsq / (b1 - b0)) / 100.0f);
        UART_Print(msg);
    }
    UART_Print("Bac (do)        t_len(ms) Vot_lo(%) t_od(ms) Sai_so(do)  RMS(do) w_max(do/s) dai(ms)\r\n");
    for (uint32_t k = 0; k < n_idx; k++) HP_StepReport(idx[k], idx[k + 1]);
    if (n_idx == 0) UART_Print("(Khong co bac nao: xe dung truoc HP_T0_MS)\r\n");
    UART_Print("t_len: dat 90% bac | Vot_lo: vuot qua dich | t_od: vao +-0.5 do | Sai_so: TB 30% cuoi | RMS: 50% cuoi\r\n");
    if (n > 1) {
        float dy = (h[n - 1].y - h[0].y) / 100.0f, de = (h[n - 1].enc - h[0].enc) / 100.0f;
        snprintf(msg, sizeof(msg), "Trong luc chay: gyro quay %+.2f do, encoder quay %+.2f do (chenh %+.2f do)\r\n", dy, de, de - dy);
        UART_Print(msg);
    }
    snprintf(msg, sizeof(msg), "Vi tri cuoi (uoc luong): doc %+.1f mm, ngang %+.1f mm (bac +5/-5 do moi bac %d ms -> ngang ~0)\r\n",
             along, lateral, HP_STEP_MS);
    UART_Print(msg);
    UART_Print("Goi y: Vot_lo > 20% hoac dao dong -> giam Kp / tang Kd | t_len > 300ms -> tang Kp\r\n");
    UART_Print("       Sai_so > 0.3 do -> tang Ki | w_max = 90 (cham gioi han) -> bac qua lon so voi Kp\r\n");

    if (print_raw) {
        UART_Print("HP,t_ms,dat_do,gyro_do,enc_do,w_gyro_dps,w_pid_dps\r\n");
        for (uint32_t i = 0; i < n; i += HP_RAW_DECIM) {
            snprintf(msg, sizeof(msg), "HP,%lu,%.2f,%.2f,%.2f,%.1f,%.1f\r\n", (unsigned long)i * 2,
                     h[i].ref / 100.0f, h[i].y / 100.0f, h[i].enc / 100.0f, h[i].rate / 10.0f, h[i].w_pid / 10.0f);
            UART_Print(msg);
        }
    }
    UART_Print("================ HET TEST PID HUONG ================\r\n");
}
