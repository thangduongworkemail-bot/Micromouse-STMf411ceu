/**
  ******************************************************************************
  * @file    mpu6500.c
  * @brief   IMU MPU6500 qua I2C1 (DMA1 Stream0 cho chiều nhận)
  *
  * Luồng chạy:
  *   main: MPU6500_Setup() = Init + Calibrate (đọc kiểu chờ) -> imu_ready = 1
  *   ngắt TIM9 (2ms): MPU6500_Tick() -> HAL_I2C_Mem_Read_DMA 14 byte từ 0x3B
  *   DMA xong: HAL_I2C_MemRxCpltCallback() -> MPU6500_Process_Data(dt)
  ******************************************************************************
  */
#include "mpu6500.h"
#include "pid.h"            // điều khiển động cơ trong MPU6500_Test
#include <math.h>           // atan2f, sqrtf, fabsf
#include <stdio.h>          // snprintf

extern I2C_HandleTypeDef hi2c1;           // main.c (CubeMX)

// gyro và gia tốc accel
static uint8_t imu_rx_buffer[14];         // DMA ghi vào đây, phải là biến toàn cục/static
float gyro_x_offset = 0.0f;
float gyro_y_offset = 0.0f;
float gyro_z_offset = 0.0f;
float pitch_angle = 0.0f;                 // Góc ngóc đầu/chúi mũi
float roll_angle = 0.0f;                  // Góc lật nghiêng xe
float yaw_angle = 0.0f;
volatile float yaw_rate_dps = 0.0f;       // Tốc độ quay Z mới nhất (độ/s), dương = quay trái
volatile int16_t imu_raw[7];              // Mẫu thô mới nhất (thứ tự như 14 byte từ 0x3B)
volatile uint32_t imu_sample_count = 0;
// Ngắt TIM9 chỉ bắt đầu đọc IMU bằng DMA sau khi MPU6500_Init + MPU6500_Calibrate xong
// (trước đó hiệu chỉnh dùng lệnh đọc I2C kiểu chờ, đọc DMA chen vào sẽ tranh bus -> offset gyro sai)
volatile uint8_t imu_ready = 0;
// Số chu kỳ TIM9 (2ms) kể từ lần xử lý IMU trước -> dt thật cho tích phân gyro,
// kể cả khi có lần đọc I2C bị lỗi/bỏ qua
static volatile uint16_t imu_ticks = 0;
volatile uint8_t imu_still = 0;           // 1: xe đứng yên -> giữ yaw, offset Gz tự chỉnh
static float still_time = 0.0f;           // Thời gian (s) điều kiện đứng yên đã thỏa liên tục
static float gz_avg = 0.0f, gz_dev = 0.0f;  // Trung bình trượt Gz và độ dao động của Gz (độ/s), xem Process_Data
static float gyro_z_cal = 0.0f;           // Offset Gz lúc hiệu chỉnh khởi động (gyro_z_offset thì tự chỉnh dần)
uint8_t mpu_whoami = 0;                   // Thanh ghi WHO_AM_I: 0x70 MPU6500, 0x71 lõi MPU9250
static volatile uint32_t imu_i2c_errors = 0;   // Lỗi bus I2C khi đọc DMA (HAL_I2C_ErrorCallback)
static volatile uint32_t imu_read_fail  = 0;   // Lệnh đọc DMA bị HAL từ chối (bus đang bận/kẹt)

// ===== NHẤC XE / ĐẶT XE (xem mpu6500.h) =====
volatile uint8_t robot_lifted = 0;
volatile uint8_t imu_state = IMU_STATE_NORMAL;
float lift_tilt_deg = 0.0f;
float lift_vz_ms    = 0.0f;
float lift_acc_dev  = 0.01f;
float lift_tilt_rate_dps = 0.0f;
static float   lift_acc_avg = 1.0f;      // |a| trung bình trượt ~0.1s (g), cho độ rung lift_acc_dev
static float   lift_ref[3] = { 0.0f, 0.0f, 1.0f };  // Hướng "lên" (vector đơn vị, hệ trục chip) lúc nằm trên sàn
static float   lift_e1[3] = { 1.0f, 0.0f, 0.0f };   // 2 trục nằm ngang vuông góc với lift_ref (Lift_SetBasis)
static float   lift_e2[3] = { 0.0f, 1.0f, 0.0f };
// Va chạm: trung vị 3 mẫu từng kênh (ax, ay, az theo g; Gz độ/s) + lịch sử đã lọc để lấy mẫu cách CRASH_LAG_SAMPLES
#define CRASH_CH              4
#define CRASH_HIST            (CRASH_LAG_SAMPLES + 1)
static float   crash_raw[3][CRASH_CH];   // 3 mẫu thô gần nhất (vòng)
static float   crash_hist[CRASH_HIST][CRASH_CH];   // Đã lọc trung vị (vòng)
static uint8_t crash_i = 0, crash_h = 0; // Vị trí ghi tiếp theo của 2 vòng
static uint8_t crash_n = 0;              // Số mẫu đã có (tới 2 + CRASH_HIST thì đủ để tính)
static uint8_t crash_spike_count = 0;    // Số mẫu liên tiếp vượt ngưỡng va chạm
static float   crash_jerk_peak = 0.0f, crash_gzjerk_peak = 0.0f;   // Lớn nhất từ lần in trước (print_mpu6500)
static float   lift_ref_g  = 1.0f;       // |gia tốc| lúc nằm yên (g; chip này đọc ~1.19 vì offset accel)
static uint8_t lift_ref_ok = 0;          // 0: chưa có hướng gốc (lấy ở mẫu đầu tiên sau MPU6500_Setup)
static float   lift_af[3];               // Gia tốc lọc thông thấp (g) để tính nghiêng
static float   lift_cond_t = 0.0f;       // Thời gian điều kiện "bị nhấc" đã đúng liên tục (s)
static float   lift_shock_t = 0.0f;      // Còn bao lâu bỏ vận tốc đứng sau cú xóc (s)
// Cửa sổ nằm yên lúc chờ đặt xuống: cộng dồn gyro thô (offset mới) + gia tốc (hướng gốc mới)
static float    place_t, place_g[3], place_gz2, place_gz_half, place_a[3];
static uint32_t place_n, place_n_half;
static void Lift_Update(const volatile int16_t *raw, float rate_gx, float rate_gy, float rate_gz, float dt);

// ===== THỐNG KÊ CHO MPU6500_Test =====
// Callback I2C cộng dồn từng mẫu (trong ngắt), luồng chính tính trung bình / độ nhiễu khi đo xong.
// Dùng số nguyên 64 bit: cộng bình phương az (~32767^2) x 1000 mẫu vẫn chính xác tuyệt đối.
typedef struct {
    int64_t sum;
    int64_t sumsq;
    int16_t min, max;
} IMU_ChStat;
// 7 kênh, đúng thứ tự 14 byte đọc từ thanh ghi 0x3B
enum { CH_AX, CH_AY, CH_AZ, CH_TEMP, CH_GX, CH_GY, CH_GZ, IMU_CH_COUNT };
static volatile IMU_ChStat imu_stat[IMU_CH_COUNT];
static volatile uint32_t   imu_stat_n = 0;
static volatile uint32_t   imu_stat_still = 0;  // Số mẫu lúc imu_still = 1
static volatile uint8_t    imu_stat_active = 0;

static char msg[256];                     // Bộ đệm in USB của module này

// ===== KHỞI ĐỘNG =====
// khởi tạo MPU6500
HAL_StatusTypeDef MPU6500_Init(void) {
    uint8_t data[2];
    HAL_StatusTypeDef status;

    // 1. Thoát Sleep Mode, dùng PLL trục Gyro X làm clock reference
    data[0] = REG_PWR_MGMT_1;
    data[1] = 0x01;
    status = HAL_I2C_Master_Transmit(&hi2c1, MPU6500_ADDR, data, 2, 100);
    if (status != HAL_OK) return status;
    HAL_Delay(10);

    // Đọc WHO_AM_I (0x75): 0x70 MPU6500, 0x71 lõi MPU9250 (cùng bản đồ thanh ghi); MPU6500_Test in ra
    status = HAL_I2C_Mem_Read(&hi2c1, MPU6500_ADDR, REG_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &mpu_whoami, 1, 100);
    if (status != HAL_OK) return status;

    // 2. DLPF GYRO = GYRO_DLPF_CFG (3: băng thông 41Hz, trễ ~5.9ms) - lý do chọn xem mpu6500.h
    // MPU6500: thanh ghi này CHỈ lọc gyro (khác MPU6050 lọc cả accel) -> accel lọc riêng ở bước 4b
    data[0] = REG_CONFIG;
    data[1] = GYRO_DLPF_CFG;
    status = HAL_I2C_Master_Transmit(&hi2c1, MPU6500_ADDR, data, 2, 100);
    if (status != HAL_OK) return status;

    // 3. Tốc độ lấy mẫu (Sample Rate) = 1kHz / (1 + 1) = 500 Hz
    // Khớp với ngắt TIM9 đọc IMU mỗi 2ms -> mỗi lần đọc là 1 mẫu mới, không đọc lặp mẫu cũ
    data[0] = REG_SMPLRT_DIV;
    data[1] = 0x01;
    status = HAL_I2C_Master_Transmit(&hi2c1, MPU6500_ADDR, data, 2, 100);
    if (status != HAL_OK) return status;

    // 4. Accel Full Scale: ACCEL_FS_CFG (±8g, ACCEL_LSB_PER_G = 4096 LSB/g) - lý do xem mpu6500.h
    data[0] = REG_ACCEL_CONFIG;
    data[1] = ACCEL_FS_CFG;
    status = HAL_I2C_Master_Transmit(&hi2c1, MPU6500_ADDR, data, 2, 100);
    if (status != HAL_OK) return status;

    // 4b. Bộ lọc riêng của Accel (chỉ MPU6500 có): A_DLPF_CFG = ACCEL_DLPF_CFG (4: ~20Hz)
    // Mặc định (0) gần như không lọc (~460Hz) trong khi đọc 500Hz -> rung động cơ lọt vào pitch/roll
    data[0] = REG_ACCEL_CONFIG2;
    data[1] = ACCEL_DLPF_CFG;
    status = HAL_I2C_Master_Transmit(&hi2c1, MPU6500_ADDR, data, 2, 100);
    if (status != HAL_OK) return status;

    // 5. Gyro Full Scale: ±2000 deg/s -> Độ nhạy 16.4 LSB/(deg/s)
    // BẮT BUỘC dùng thang đo lớn nhất vì Micromouse xoay rất gắt
    data[0] = REG_GYRO_CONFIG;
    data[1] = 0x18;
    status = HAL_I2C_Master_Transmit(&hi2c1, MPU6500_ADDR, data, 2, 100);
    if (status != HAL_OK) return status;

    return HAL_OK;
}
// Hiệu chỉnh offset gyro lúc khởi động, xe phải đứng yên. Bỏ GYRO_CAL_DISCARD mẫu đầu (gyro vừa
// khởi động / bộ lọc vừa đổi cấu hình), lấy trung bình GYRO_CAL_SAMPLES mẫu. Đo lại khi: nhiễu lớn (xe bị
// chạm, rung) hoặc Gz trung bình nửa đầu khác nửa sau (xe đang bị xoay chậm lúc đặt xuống).
// Về sau offset Z còn tự chỉnh mỗi khi xe đứng yên (MPU6500_Process_Data) và trước mỗi lần chạy.
HAL_StatusTypeDef MPU6500_Calibrate(void) {
    uint8_t raw_data[6];
    uint32_t t0 = HAL_GetTick(), t_try = 0;   // t_try: thời gian 1 lần đo (lấy từ lần trước)

    HAL_Delay(50);                        // Gyro khởi động từ sleep mất ~35ms (datasheet)
    for (int attempt = 0; attempt < GYRO_CAL_RETRY; attempt++) {
        uint32_t ta = HAL_GetTick();
        if (ta - t0 + t_try > GYRO_CAL_MAX_MS) return HAL_TIMEOUT;   // Không kịp đo thêm 1 lần: giữ kết quả lần trước
        int64_t sum[3] = { 0, 0, 0 }, sumsq[3] = { 0, 0, 0 };
        int64_t sum_z_half = 0;           // Tổng Gz nửa đầu
        int n = 0, n_half = 0;

        for (int i = 0; i < GYRO_CAL_DISCARD + GYRO_CAL_SAMPLES; i++) {
            if (HAL_GetTick() - t0 > GYRO_CAL_MAX_MS) return HAL_TIMEOUT;   // Bus I2C lỗi kéo dài lần đo
            HAL_Delay(2);                 // 1 mẫu mới mỗi 2ms (500Hz)
            // Đọc trực tiếp 6 byte thanh ghi Gyro (0x43 đến 0x48), lần đọc lỗi thì bỏ mẫu đó
            if (HAL_I2C_Mem_Read(&hi2c1, MPU6500_ADDR, 0x43, I2C_MEMADD_SIZE_8BIT, raw_data, 6,
                                 GYRO_CAL_I2C_TIMEOUT_MS) != HAL_OK) continue;
            if (i < GYRO_CAL_DISCARD) continue;
            for (int a = 0; a < 3; a++) {
                int16_t v = (int16_t)((raw_data[2 * a] << 8) | raw_data[2 * a + 1]);
                sum[a]   += v;
                sumsq[a] += (int32_t)v * v;
            }
            if (i < GYRO_CAL_DISCARD + GYRO_CAL_SAMPLES / 2) {
                sum_z_half = sum[2];
                n_half = n + 1;
            }
            n++;
        }
        t_try = HAL_GetTick() - ta;
        if (n < GYRO_CAL_SAMPLES / 2 || n_half == 0 || n_half >= n) continue;   // Bus I2C lỗi quá nhiều: đo lại
        // Gz trung bình nửa đầu so với nửa sau (độ/s)
        float drift = ((float)sum_z_half / n_half - (float)(sum[2] - sum_z_half) / (n - n_half)) / 16.4f;

        // Trung bình = sai số tĩnh. Độ lệch chuẩn lớn nhất của 3 trục kiểm tra xe có đứng yên không
        float mean[3], worst_std = 0.0f;
        for (int a = 0; a < 3; a++) {
            double m   = (double)sum[a] / n;
            double var = (double)sumsq[a] / n - m * m;
            float std  = (var > 0.0) ? (float)sqrt(var) / 16.4f : 0.0f;
            mean[a] = (float)m;
            if (std > worst_std) worst_std = std;
        }
        gyro_x_offset = mean[0];
        gyro_y_offset = mean[1];
        gyro_z_offset = mean[2];
        gyro_z_cal    = mean[2];
        if (worst_std < GYRO_CAL_MAX_STD_DPS && fabsf(drift) < GYRO_CAL_MAX_DRIFT_DPS) return HAL_OK;
    }
    return HAL_BUSY;                      // Giữ kết quả lần đo cuối, offset Z sẽ tự sửa khi xe đứng yên
}
// Khởi tạo + hiệu chỉnh gyro (~1.2s, xe phải đứng yên), xong mới cho ngắt TIM9 đọc IMU bằng DMA.
// Lúc hiệu chỉnh ngắt TIM9 chưa đọc IMU nên không tranh bus I2C. MPU lỗi thì không bật đọc.
HAL_StatusTypeDef MPU6500_Setup(void)
{
    HAL_StatusTypeDef status = MPU6500_Init();
    if (status != HAL_OK) return status;

    status = MPU6500_Calibrate();
    yaw_angle  = 0.0f;
    still_time = 0.0f;
    gz_avg     = 0.0f;
    gz_dev     = 1.0f;                    // Bắt đầu "chưa êm": phải êm thật mới tính là đứng yên
    imu_still  = 0;
    imu_ticks  = 0;
    // Hiệu chỉnh được (xe nằm yên) = xe đang trên sàn: hướng gốc lấy ở mẫu đầu tiên, bật LED.
    // Không yên (đang cầm trên tay) = coi như đang bị nhấc: khóa động cơ tới khi đặt xuống nằm yên
    lift_ref_ok  = 0;
    lift_cond_t  = 0.0f;
    lift_vz_ms   = 0.0f;
    place_t      = 0.0f;
    place_n      = 0;
    crash_n      = 0;
    crash_spike_count = 0;
    robot_lifted = (status != HAL_OK);
    imu_state    = robot_lifted ? IMU_STATE_PICKED_UP : IMU_STATE_NORMAL;
    HAL_GPIO_WritePin(LED_RUN_PORT, LED_RUN_PIN, robot_lifted ? LED_RUN_OFF : LED_RUN_ON);
    imu_ready  = 1;
    return status;
}

// ===== GỌI TRONG NGẮT =====
// Ngắt TIM9 mỗi 2ms: chỉ RA LỆNH đọc bằng DMA, không xử lý MPU ở đây
// Chỉ đọc khi IMU đã khởi tạo + hiệu chỉnh xong (tránh tranh bus I2C với lệnh đọc kiểu chờ)
void MPU6500_Tick(void)
{
    imu_ticks++;
    if (imu_ready) {
        if (HAL_I2C_Mem_Read_DMA(&hi2c1, MPU6500_ADDR, 0x3B, I2C_MEMADD_SIZE_8BIT, imu_rx_buffer, 14) != HAL_OK) {
            imu_read_fail++;              // Lần đọc trước chưa xong / bus I2C kẹt
        }
    }
}
// Cộng 1 mẫu vừa đọc vào thống kê (gọi trong callback I2C khi MPU6500_Test đang đo)
static void IMU_Stat_Add(void)
{
    for (int c = 0; c < IMU_CH_COUNT; c++) {
        int16_t v = (int16_t)((imu_rx_buffer[2 * c] << 8) | imu_rx_buffer[2 * c + 1]);
        imu_stat[c].sum   += v;
        imu_stat[c].sumsq += (int32_t)v * v;
        if (v < imu_stat[c].min) imu_stat[c].min = v;
        if (v > imu_stat[c].max) imu_stat[c].max = v;
    }
    if (imu_still) imu_stat_still++;
    imu_stat_n++;
}
void MPU6500_Process_Data(float dt) {
    // 1. Lắp ghép 14 byte từ mảng DMA thành số nguyên 16-bit
    int16_t raw_ax = (int16_t)((imu_rx_buffer[0] << 8) | imu_rx_buffer[1]);
    int16_t raw_ay = (int16_t)((imu_rx_buffer[2] << 8) | imu_rx_buffer[3]);
    int16_t raw_az = (int16_t)((imu_rx_buffer[4] << 8) | imu_rx_buffer[5]);
    // byte [6][7] là nhiệt độ, bỏ qua.
    int16_t raw_gx = (int16_t)((imu_rx_buffer[8] << 8) | imu_rx_buffer[9]);
    int16_t raw_gy = (int16_t)((imu_rx_buffer[10] << 8) | imu_rx_buffer[11]);
    int16_t raw_gz = (int16_t)((imu_rx_buffer[12] << 8) | imu_rx_buffer[13]);
    imu_raw[0] = raw_ax; imu_raw[1] = raw_ay; imu_raw[2] = raw_az;
    imu_raw[3] = (int16_t)((imu_rx_buffer[6] << 8) | imu_rx_buffer[7]);
    imu_raw[4] = raw_gx; imu_raw[5] = raw_gy; imu_raw[6] = raw_gz;

    // 2. Trừ đi nhiễu tĩnh và chia hệ số nhạy (16.4 cho thang đo ±2000 dps)
    float rate_gx = (raw_gx - gyro_x_offset) / 16.4f;
    float rate_gy = (raw_gy - gyro_y_offset) / 16.4f;
    float rate_gz = (raw_gz - gyro_z_offset) / 16.4f;

    // 3. Tính góc tuyệt đối từ Gia tốc kế
    // Lưu ý: Tùy chiều gắn chip trên xe của bạn mà trục X/Y có thể đảo cho nhau
    float ax = raw_ax, ay = raw_ay, az = raw_az;      // float: ay^2 + az^2 kiểu int có thể tràn khi va đập
    float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / 3.141592f;
    float roll_acc  = atan2f(ay, az) * 180.0f / 3.141592f;

    // 4. BỘ LỌC BÙ (COMPLEMENTARY FILTER), hằng số thời gian ANGLE_COMP_TAU_S
    // Ngắn hơn tau: tin gyro; dài hơn tau: accel kéo góc về (gyro không trôi mãi). Tính alpha theo dt thật
    float alpha = ANGLE_COMP_TAU_S / (ANGLE_COMP_TAU_S + dt);
    pitch_angle = alpha * (pitch_angle + rate_gy * dt) + (1.0f - alpha) * pitch_acc;
    roll_angle  = alpha * (roll_angle  + rate_gx * dt) + (1.0f - alpha) * roll_acc;

    // 5. Nhận biết đứng yên (xem mpu6500.h): 2 bánh không quay (encoder, pid.c) + Gz êm + |Gz| nhỏ, liên tục
    //    gz_avg: trung bình trượt Gz (~0.1s), gz_dev: độ dao động TB |Gz - gz_avg| (~0.25s).
    //    Tay xoay / cầm xe: Gz dao động -> không bao giờ tính là đứng yên, kể cả lúc xoay rất chậm
    float k_avg = dt / 0.1f, k_dev = dt / 0.25f;
    if (k_avg > 1.0f) k_avg = 1.0f;
    if (k_dev > 1.0f) k_dev = 1.0f;
    gz_avg += k_avg * (rate_gz - gz_avg);
    gz_dev += k_dev * (fabsf(rate_gz - gz_avg) - gz_dev);
    if (fabsf(speed_left_mms) < STILL_SPEED_MMS && fabsf(speed_right_mms) < STILL_SPEED_MMS
        && gz_dev < STILL_DEV_DPS && fabsf(rate_gz) < STILL_GZ_MAX_DPS) {
        if (still_time < STILL_HOLD_LONG_S) still_time += dt;
    } else {
        still_time = 0.0f;
    }
    // |Gz| < 1 độ/s: đứng yên sau 0.25s. 1..5 độ/s (chỉ có thể là offset sai): phải êm liên tục 2s
    imu_still = (still_time >= ((fabsf(rate_gz) < STILL_GZ_DPS) ? STILL_HOLD_S : STILL_HOLD_LONG_S));

    // 6. Góc Yaw (Heading): không có accel để bù, chỉ tích phân gyro
    yaw_rate_dps = YAW_SIGN * rate_gz;    // Cho khâu D giữ hướng (pid.c) và so sánh với encoder
    if (imu_still) {
        // Đứng yên: Gz đo được lúc này chính là bias -> offset bám theo (lọc thông thấp GYRO_BIAS_TAU_S),
        // sửa sai số hiệu chỉnh lúc khởi động và bias trôi theo nhiệt độ. Yaw giữ nguyên.
        float k = dt / GYRO_BIAS_TAU_S;
        if (k > 1.0f) k = 1.0f;
        gyro_z_offset += k * (raw_gz - gyro_z_offset);
    } else {
        // Đang chạy / đang quay: cộng toàn bộ, không ngưỡng chết (offset đã chính xác, nhiễu trung bình 0)
        // YAW_SIGN: yaw tăng khi xe quay trái
        yaw_angle += YAW_SIGN * rate_gz * dt;
    }

    // 7. Nhấc xe / đặt xe xuống
    Lift_Update(imu_raw, rate_gx, rate_gy, rate_gz, dt);
}

// ===== NHẤC XE / ĐẶT XE (gọi trong ngắt, mỗi mẫu IMU) =====
static void Lift_ResetWindow(void)
{
    place_t = 0.0f;
    place_n = 0;
    place_n_half = 0;
    place_g[0] = place_g[1] = place_g[2] = 0.0f;
    place_a[0] = place_a[1] = place_a[2] = 0.0f;
    place_gz2 = place_gz_half = 0.0f;
}
// Hướng "lên" lift_ref vừa đổi: dựng 2 trục nằm ngang e1 (trục x chip chiếu xuống mặt phẳng ngang), e2 = lên x e1
static void Lift_SetBasis(void)
{
    const float *u = lift_ref;
    float e[3] = { 1.0f, 0.0f, 0.0f };
    float d = u[0];
    float n;
    if (fabsf(d) > 0.9f) {                // Trục x chip gần thẳng đứng: dùng trục y
        e[0] = 0.0f;
        e[1] = 1.0f;
        d = u[1];
    }
    for (int i = 0; i < 3; i++) e[i] -= d * u[i];
    n = sqrtf(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
    for (int i = 0; i < 3; i++) lift_e1[i] = e[i] / n;
    lift_e2[0] = u[1] * lift_e1[2] - u[2] * lift_e1[1];
    lift_e2[1] = u[2] * lift_e1[0] - u[0] * lift_e1[2];
    lift_e2[2] = u[0] * lift_e1[1] - u[1] * lift_e1[0];
}
static float Median3(float a, float b, float c)
{
    return fmaxf(fminf(a, b), fminf(fmaxf(a, b), c));
}
// KHÓA AN TOÀN (bị nhấc / va chạm): khóa động cơ, hủy chuyển động, tắt LED PB15, chờ nằm yên (Lift_Update bước 2)
static void Lift_Lock(uint8_t state)
{
    robot_lifted = 1;
    imu_state    = state;
    lift_cond_t  = 0.0f;
    crash_spike_count = 0;
    Lift_ResetWindow();
    HAL_GPIO_WritePin(LED_RUN_PORT, LED_RUN_PIN, LED_RUN_OFF);
    Motion_OnLifted();                    // pid.c: hủy chuyển động, tắt PID, phanh 2 bánh
}
// VA CHẠM (xem CRASH_ ở mpu6500.h), mỗi mẫu IMU. a: gia tốc (g), gz: tốc độ quay trục Z chip đã trừ offset (độ/s)
static void Crash_Update(const float a[3], float gz)
{
    const float in[CRASH_CH] = { a[0], a[1], a[2], gz };
    float *now = crash_hist[crash_h];
    const float *old;
    float d[3], planar, gzj;
    // 1. Trung vị 3 mẫu từng kênh: 1 mẫu I2C hỏng bị loại hẳn
    for (int k = 0; k < CRASH_CH; k++) crash_raw[crash_i][k] = in[k];
    crash_i = (uint8_t)((crash_i + 1U) % 3U);
    for (int k = 0; k < CRASH_CH; k++) now[k] = Median3(crash_raw[0][k], crash_raw[1][k], crash_raw[2][k]);
    // 2. Mẫu cũ nhất trong vòng lịch sử = cách CRASH_LAG_SAMPLES mẫu
    crash_h = (uint8_t)((crash_h + 1U) % CRASH_HIST);
    old = crash_hist[crash_h];
    if (crash_n < 2U + CRASH_HIST) {      // Chưa đủ 3 mẫu thô + đầy vòng lịch sử (sau MPU6500_Setup)
        crash_n++;
        return;
    }
    // 3. Độ giật trong mặt phẳng ngang + độ giật Gz
    for (int i = 0; i < 3; i++) d[i] = now[i] - old[i];
    planar = fabsf(d[0] * lift_e1[0] + d[1] * lift_e1[1] + d[2] * lift_e1[2])
           + fabsf(d[0] * lift_e2[0] + d[1] * lift_e2[1] + d[2] * lift_e2[2]);
    gzj = fabsf(now[3] - old[3]);
    if (planar > crash_jerk_peak) crash_jerk_peak = planar;
    if (gzj > crash_gzjerk_peak) crash_gzjerk_peak = gzj;
    // 4. Chỉ báo khi xe đang chạy bằng PID, trên sàn
    if (robot_lifted || !lift_ref_ok || !motor_pid_enable) {
        crash_spike_count = 0;
        return;
    }
    if (planar > CRASH_JERK_G || (planar > CRASH_JERK_SIDE_G && gzj > CRASH_GZ_JERK_DPS)) {
        if (++crash_spike_count >= CRASH_CONFIRM_N) Lift_Lock(IMU_STATE_CRASHED);
    } else {
        crash_spike_count = 0;
    }
}
static void Lift_Update(const volatile int16_t *raw, float rate_gx, float rate_gy, float rate_gz, float dt)
{
    float a[3] = { raw[0] / ACCEL_LSB_PER_G, raw[1] / ACCEL_LSB_PER_G, raw[2] / ACCEL_LSB_PER_G };   // g
    float an = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    if (an < 0.2f) return;                // Rơi tự do / đọc lỗi: không có hướng trọng lực để so

    if (!lift_ref_ok) {                   // Mẫu đầu sau MPU6500_Setup
        for (int i = 0; i < 3; i++) lift_af[i] = a[i];
        lift_acc_avg = an;
        if (!robot_lifted) {              // Đang nằm yên trên sàn (hiệu chỉnh được): lấy làm hướng gốc
            for (int i = 0; i < 3; i++) lift_ref[i] = a[i] / an;
            lift_ref_g  = an;
            lift_ref_ok = 1;
            Lift_SetBasis();
        }
    }
    Crash_Update(a, rate_gz);

    // Nghiêng: góc giữa gia tốc đã lọc và hướng "lên" lúc nằm trên sàn
    float k = dt / (LIFT_TILT_TAU_S + dt);
    for (int i = 0; i < 3; i++) lift_af[i] += k * (a[i] - lift_af[i]);
    float afn = sqrtf(lift_af[0] * lift_af[0] + lift_af[1] * lift_af[1] + lift_af[2] * lift_af[2]);
    float c = (lift_af[0] * lift_ref[0] + lift_af[1] * lift_ref[1] + lift_af[2] * lift_ref[2]) / afn;
    if (c > 1.0f) c = 1.0f;
    if (c < -1.0f) c = -1.0f;
    lift_tilt_deg = acosf(c) * 180.0f / 3.141592f;
    // Độ rung gia tốc: TB |(|a| - TB trượt 0.1s)| lọc 0.1s (tay cầm luôn run, nằm trên sàn thì gần bằng nhiễu chip)
    float ka = dt / 0.1f;
    if (ka > 1.0f) ka = 1.0f;
    lift_acc_avg += ka * (an - lift_acc_avg);
    lift_acc_dev += ka * (fabsf(an - lift_acc_avg) - lift_acc_dev);
    // Vận tốc đứng: gia tốc theo hướng "lên" trừ trọng lực lúc nằm yên, tích phân có rò
    float a_up = a[0] * lift_ref[0] + a[1] * lift_ref[1] + a[2] * lift_ref[2] - lift_ref_g;
    if (fabsf(a_up) > LIFT_VZ_SHOCK_G) lift_shock_t = LIFT_VZ_SHOCK_S;   // Xóc (gờ sàn / va): không phải tay nhấc
    if (lift_shock_t > 0.0f) {
        lift_shock_t -= dt;
        lift_vz_ms = 0.0f;
    } else {
        lift_vz_ms += a_up * 9.81f * dt - lift_vz_ms * dt / LIFT_VZ_TAU_S;
    }
    // Tốc độ lật: gyro quanh 2 trục nằm ngang (roll + pitch theo trọng lực) + tốc độ quay quanh trục đứng
    float w_up = rate_gx * lift_ref[0] + rate_gy * lift_ref[1] + rate_gz * lift_ref[2];
    lift_tilt_rate_dps = fabsf(rate_gx * lift_e1[0] + rate_gy * lift_e1[1] + rate_gz * lift_e1[2])
                       + fabsf(rate_gx * lift_e2[0] + rate_gy * lift_e2[1] + rate_gz * lift_e2[2]);

    if (!robot_lifted) {
        // 1. Đang trên sàn: nghiêng nhiều / bị nâng lên / tay lật xe (lúc xe không tự quay nhanh), LIÊN TỤC
        //    LIFT_CONFIRM_S -> bị nhấc. Vấp gờ sàn chỉ làm các dấu hiệu này nháy vài chục ms -> đếm lại từ 0
        uint8_t tilting = (lift_tilt_rate_dps > LIFT_TILT_RATE_DPS) && (fabsf(w_up) < LIFT_TILT_RATE_YAW_DPS);
        if (lift_ref_ok && (lift_tilt_deg > LIFT_TILT_DEG || fabsf(lift_vz_ms) > LIFT_VZ_MS || tilting)) lift_cond_t += dt;
        else lift_cond_t = 0.0f;
        if (lift_cond_t >= LIFT_CONFIRM_S) Lift_Lock(IMU_STATE_PICKED_UP);
        return;
    }

    // 2. Đang bị nhấc: chờ đặt xuống. Mẫu nào không yên (còn tay cầm, đang hạ xuống...) thì đếm lại từ đầu
    uint8_t calm = fabsf(rate_gx) < LIFT_PLACE_GYRO_DPS && fabsf(rate_gy) < LIFT_PLACE_GYRO_DPS
                && fabsf(rate_gz) < LIFT_PLACE_GYRO_DPS && gz_dev < STILL_DEV_DPS
                && lift_acc_dev < LIFT_PLACE_ACC_DEV_G
                && fabsf(speed_left_mms) < STILL_SPEED_MMS && fabsf(speed_right_mms) < STILL_SPEED_MMS
                && (!lift_ref_ok || lift_tilt_deg < LIFT_PLACE_TILT_DEG);
    if (!calm) {
        Lift_ResetWindow();
        return;
    }
    for (int i = 0; i < 3; i++) {
        place_g[i] += raw[4 + i];
        place_a[i] += a[i];
    }
    place_gz2 += (float)raw[6] * raw[6];
    place_n++;
    place_t += dt;
    if (place_n_half == 0 && place_t >= 0.5f * LIFT_PLACE_S) {
        place_gz_half = place_g[2];
        place_n_half  = place_n;
    }
    if (place_t < LIFT_PLACE_S || place_n_half == 0 || place_n_half >= place_n) return;

    // 3. Đủ thời gian nằm yên: kiểm tra như hiệu chỉnh gyro (Gz nhiễu nhỏ, nửa đầu = nửa sau: không bị xoay chậm)
    float n       = (float)place_n;
    float gz_mean = place_g[2] / n;
    float gz_var  = place_gz2 / n - gz_mean * gz_mean;
    float drift   = (place_gz_half / place_n_half - (place_g[2] - place_gz_half) / (place_n - place_n_half)) / 16.4f;
    if ((gz_var > 0.0f && sqrtf(gz_var) / 16.4f > GYRO_CAL_MAX_STD_DPS)
        || fabsf(drift) > GYRO_CAL_MAX_DRIFT_DPS) {
        Lift_ResetWindow();
        return;
    }

    // 4. ĐÃ ĐẶT XUỐNG: offset gyro mới, hướng "lên" mới, yaw = 0 -> góc ban đầu = góc lúc đặt xuống
    gyro_x_offset = place_g[0] / n;
    gyro_y_offset = place_g[1] / n;
    gyro_z_offset = gz_mean;
    float m[3] = { place_a[0] / n, place_a[1] / n, place_a[2] / n };
    float mn = sqrtf(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]);
    for (int i = 0; i < 3; i++) {
        lift_ref[i] = m[i] / mn;
        lift_af[i]  = m[i];
    }
    lift_ref_g  = mn;
    lift_ref_ok = 1;
    Lift_SetBasis();
    lift_vz_ms  = 0.0f;
    yaw_angle   = 0.0f;
    robot_lifted = 0;
    imu_state    = IMU_STATE_NORMAL;
    Lift_ResetWindow();
    HAL_GPIO_WritePin(LED_RUN_PORT, LED_RUN_PIN, LED_RUN_ON);
    Motion_OnPlaced();                    // pid.c: hướng đích + gốc vị trí = chỗ đặt
}
// DMA I2C1 đọc xong 14 byte (gọi từ ngắt DMA1_Stream0 / I2C1_EV qua HAL)
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        // 1. Tính toán MPU6500 (Góc Pitch, Roll, Yaw)
        //    dt = thời gian thật kể từ lần xử lý trước: 1 chu kỳ TIM9 = 2ms
        //    (trước đây cố định 0.005f trong khi gọi mỗi 2ms -> góc yaw bị cộng gấp 2.5 lần)
        float dt = imu_ticks * CONTROL_DT;
        imu_ticks = 0;
        MPU6500_Process_Data(dt);
        imu_sample_count++;
        if (imu_stat_active) IMU_Stat_Add();   // MPU6500_Test đang đo

        // 2. Chạy thuật toán PID ngay tại đây
    }
}
// Lỗi bus I2C (NACK, nhiễu làm hỏng xung SCL/SDA...): HAL đã hủy lần đọc DMA và trả I2C về READY,
// chu kỳ TIM9 sau đọc lại. Chỉ đếm để MPU6500_Test báo cáo.
// Bản KHÔNG CHẶN (luồng chính gọi Poll mỗi vòng lặp; mỗi lần Poll xử lý mẫu DMA mới nhất nếu có)
static struct {
    float    sum, sumsq, sum_half;
    uint32_t n, last, t0;
    uint8_t  active;
} gz0;
void MPU6500_ZeroGyroZ_Start(void)
{
    gz0.sum = gz0.sumsq = gz0.sum_half = 0.0f;
    gz0.n      = 0;
    gz0.last   = imu_sample_count;
    gz0.t0     = HAL_GetTick();
    gz0.active = 1;
}
uint8_t MPU6500_ZeroGyroZ_Poll(void)
{
    if (!gz0.active || !imu_ready) {
        gz0.active = 0;
        return HAL_ERROR;
    }
    if (gz0.n < GYRO_REZERO_SAMPLES) {
        if (HAL_GetTick() - gz0.t0 >= GYRO_REZERO_SAMPLES * 2U + 500U) {   // Mất mẫu IMU quá nhiều
            gz0.active = 0;
            return HAL_TIMEOUT;
        }
        if (imu_sample_count == gz0.last) return GYRO_ZERO_RUNNING;       // Chưa có mẫu DMA mới (2ms/mẫu)
        gz0.last = imu_sample_count;
        if (fabsf(speed_left_mms) >= STILL_SPEED_MMS || fabsf(speed_right_mms) >= STILL_SPEED_MMS) {
            gz0.active = 0;
            return HAL_BUSY;
        }
        float v = imu_raw[6];
        gz0.sum   += v;
        gz0.sumsq += v * v;
        gz0.n++;
        if (gz0.n == GYRO_REZERO_SAMPLES / 2) gz0.sum_half = gz0.sum;
        if (gz0.n < GYRO_REZERO_SAMPLES) return GYRO_ZERO_RUNNING;
    }
    gz0.active = 0;
    float mean = gz0.sum / gz0.n, var = gz0.sumsq / gz0.n - mean * mean;
    if (var > 0.0f && sqrtf(var) / 16.4f > GYRO_CAL_MAX_STD_DPS) return HAL_BUSY;   // Bị chạm / rung
    // Nửa đầu khác nửa sau = đang bị xoay chậm (tay chỉnh xe) -> không đổi offset
    float drift = (gz0.sum_half - (gz0.sum - gz0.sum_half)) / (GYRO_REZERO_SAMPLES / 2) / 16.4f;
    if (fabsf(drift) > GYRO_CAL_MAX_DRIFT_DPS) return HAL_BUSY;
    gyro_z_offset = mean;
    return HAL_OK;
}
HAL_StatusTypeDef MPU6500_ZeroGyroZ(void)
{
    uint8_t r;
    if (!imu_ready) return HAL_ERROR;
    MPU6500_ZeroGyroZ_Start();
    while ((r = MPU6500_ZeroGyroZ_Poll()) == GYRO_ZERO_RUNNING) {
    }
    return (HAL_StatusTypeDef)r;
}
// Bộ phát hiện khác (Check_Robot_Picked_Up: encoder / IR, mode_manager.c) báo bị nhấc: vào đúng trạng thái "bị nhấc"
// như gia tốc kế (khóa động cơ, hủy chuyển động, tắt LED PB15). Đặt xuống vẫn do Lift_Update xác nhận (nằm yên ~1s).
// Gọi trong ngắt TIM9: cùng mức ưu tiên với ngắt I2C (Lift_Update) nên 2 bên không chen ngang nhau
void MPU6500_ForceLifted(void)
{
    if (robot_lifted) return;
    Lift_Lock(IMU_STATE_PICKED_UP);
}
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) imu_i2c_errors++;
}

// Quấn góc về [0, 360) - CHỈ để hiển thị / đổi ra hướng Bắc-Đông-Nam-Tây.
// yaw_angle (và Robot_Heading trong pid.c) cố ý KHÔNG quấn: điều khiển cần góc liên tục (Turn_Angle(3600) đếm 10
// vòng, Rectangle_Test cộng dồn hướng đích -90 -180..., Gyro_Spin_Calibrate đo góc giữa các vòng). Quấn yaw_angle
// thì lúc đi qua mốc 360/0 sai lệch hướng nhảy ~360 độ -> xe quay loạn
float MPU6500_Wrap360(float deg)
{
    float w = fmodf(deg, 360.0f);
    if (w < 0.0f) w += 360.0f;
    if (w >= 360.0f) w = 0.0f;            // -0.00001 + 360 làm tròn float ra đúng 360
    return w;
}

// ===== DEBUG =====
// IN GIÁ TRỊ MPU SAU BỘ LỌC qua USART1 (HC-05): mỗi lần gọi 1 dòng CSV (gọi lặp trong while(1), giãn nhịp
// bằng HAL_Delay bên ngoài). Dòng tiêu đề in lần đầu và mỗi 50 dòng (mở terminal giữa chừng vẫn thấy).
//   pitch, roll : bộ lọc bù gyro + accel, hằng số thời gian ANGLE_COMP_TAU_S (độ)
//   yaw         : hướng xe quấn về 0..360 (độ, tăng khi quay trái: 90 = đã quay trái 90, 270 = đã quay phải 90).
//                 Chỉ quấn khi IN RA; yaw_angle dùng tính toán vẫn cộng dồn (vd 450 = 1 vòng + 90)
//   gx, gy      : gyro đã trừ offset, qua DLPF gyro 41Hz (độ/s)
//   gz          : tốc độ quay yaw - đúng giá trị yaw và PID giữ hướng dùng (độ/s, dương = trái)
//   ax, ay, az  : gia tốc qua DLPF accel 20Hz (g, nằm yên trên sàn thì |a| ~ 1)
//   nhiet       : nhiệt độ chip (°C) | dy: 1 = đang nhận là đứng yên (yaw giữ, offset Z tự chỉnh)
//   nghieng, vz : độ nghiêng so với lúc nằm trên sàn (độ) và vận tốc thẳng đứng (m/s) - so với ngưỡng
//                 LIFT_TILT_DEG / LIFT_VZ_MS | rung: độ rung gia tốc (mg), so với LIFT_PLACE_ACC_DEV_G
//   nhac        : 1 = động cơ đang bị khóa an toàn (bị nhấc hoặc va chạm)
//   lat         : tốc độ lật roll + pitch (độ/s), so với LIFT_TILT_RATE_DPS
//   giat, giatz : độ giật ngang (g) / độ giật Gz (độ/s) LỚN NHẤT kể từ dòng trước, so với CRASH_JERK_G,
//                 CRASH_JERK_SIDE_G / CRASH_GZ_JERK_DPS -> chạy thử, đâm tường, vấp gờ để chỉnh ngưỡng va chạm
//   tt          : trạng thái IMU (0 bình thường, 1 bị nhấc, 2 va chạm)
// 9600 baud: 1 dòng ~130 ký tự mất ~135ms -> tối đa ~7 dòng/s
void print_mpu6500(void)
{
    static uint16_t lines = 0;
    int16_t raw[7];
    float pitch, roll, yaw, gz, offx, offy, tilt, vz, dev, trate, jerk, gzjerk;
    uint8_t still, lifted, state;

    if (!imu_ready) {
        UART_Print("MPU chua san sang (MPU6500_Setup loi hoac chua goi)\r\n");
        return;
    }
    // Chụp 1 bộ số cùng 1 mẫu (ngắt DMA I2C có thể cập nhật giữa chừng), chặn ngắt vài micro giây
    __disable_irq();
    for (int i = 0; i < 7; i++) raw[i] = imu_raw[i];
    pitch = pitch_angle;
    roll  = roll_angle;
    yaw   = yaw_angle;
    gz    = yaw_rate_dps;
    offx  = gyro_x_offset;
    offy  = gyro_y_offset;
    still = imu_still;
    tilt  = lift_tilt_deg;
    vz    = lift_vz_ms;
    dev   = lift_acc_dev;
    lifted = robot_lifted;
    trate  = lift_tilt_rate_dps;
    jerk   = crash_jerk_peak;
    gzjerk = crash_gzjerk_peak;
    crash_jerk_peak = crash_gzjerk_peak = 0.0f;   // Đỉnh tính lại cho dòng sau
    state  = imu_state;
    __enable_irq();

    if (lines++ % 50 == 0) UART_Print("MPU,t_ms,pitch,roll,yaw,gx,gy,gz,ax,ay,az,nhiet,dy,nghieng,vz,rung,nhac,lat,giat,giatz,tt\r\n");
    snprintf(msg, sizeof(msg), "MPU,%lu,%+.2f,%+.2f,%.2f,%+.2f,%+.2f,%+.2f,%+.3f,%+.3f,%+.3f,%.1f,%u,%.1f,%+.3f,%.1f,%u,%.1f,%.3f,%.1f,%u\r\n",
             (unsigned long)HAL_GetTick(), pitch, roll, MPU6500_Wrap360(yaw),   // yaw in ra: 0..360
             (raw[4] - offx) / 16.4f, (raw[5] - offy) / 16.4f, gz,
             raw[0] / ACCEL_LSB_PER_G, raw[1] / ACCEL_LSB_PER_G, raw[2] / ACCEL_LSB_PER_G,
             raw[3] / 333.87f + 21.0f, still, tilt, vz, dev * 1000.0f, lifted, trate, jerk, gzjerk, state);
    UART_Print(msg);
}

// ===== TEST NHIỄU / NHIỆT ĐỘ (MPU6500_Test) =====
// Kết quả 1 lần đo, giá trị thô của chip (đổi đơn vị lúc in)
typedef struct {
    uint32_t n;                    // Số mẫu nhận được
    uint32_t expected;             // Số mẫu lẽ ra phải có (1 mẫu / 2ms)
    uint32_t errors;               // Lỗi I2C + lệnh đọc DMA bị từ chối trong lúc đo
    float mean[IMU_CH_COUNT];
    float std[IMU_CH_COUNT];       // Độ lệch chuẩn = độ nhiễu
    float p2p[IMU_CH_COUNT];       // Đỉnh - đỉnh
    float yaw_drift;               // Yaw thay đổi trong lúc đo (độ)
    float seconds;
    float speed_l, speed_r;        // Vận tốc bánh lúc đo (mm/s): kiểm tra bánh có quay hay không
    float still_pct;               // % mẫu nhận ra đứng yên (yaw giữ nguyên, offset Gz tự chỉnh)
    float gz_off;                  // Offset Gz (số thô) lúc đo xong, offset tự chỉnh nên thay đổi dần
} IMU_Result;
// 1 điểm theo dõi nhiệt độ (trung bình 1 giây)
typedef struct {
    float temp;                    // °C
    float gx, gy, gz;              // Gyro trung bình trừ offset LÚC KHỞI ĐỘNG (độ/s) = bias đã trôi
    float gz_off;                  // Offset Gz tự chỉnh - offset lúc khởi động (độ/s): phải bám theo gz
    float yaw;                     // Góc yaw lúc đó (độ)
} IMU_TempPoint;
static IMU_TempPoint imu_temp_log[MPU_TEMP_LOG_MAX];

// Đổi số thô nhiệt độ sang °C (datasheet MPU6500: 333.87 LSB/°C, 0 = 21°C)
static float IMU_TempC(float raw)
{
    return raw / 333.87f + 21.0f;
}
// Đo 'samples' mẫu (2ms/mẫu) ở điều kiện hiện tại của xe. Gọi ở luồng chính.
// IMU ngừng trả dữ liệu (bus I2C kẹt) thì thoát sau thời gian dự kiến + 1s, không treo.
static void IMU_Measure(uint32_t samples, IMU_Result *r)
{
    imu_stat_active = 0;
    for (int c = 0; c < IMU_CH_COUNT; c++) {
        imu_stat[c].sum   = 0;
        imu_stat[c].sumsq = 0;
        imu_stat[c].min   = INT16_MAX;
        imu_stat[c].max   = INT16_MIN;
    }
    imu_stat_n = 0;
    imu_stat_still = 0;
    uint32_t err0 = imu_i2c_errors + imu_read_fail;
    float yaw0 = yaw_angle;
    uint32_t t0 = HAL_GetTick();
    imu_stat_active = 1;
    while (imu_stat_n < samples && HAL_GetTick() - t0 < samples * 2 + 1000) {}
    imu_stat_active = 0;              // Callback I2C chạy xong trọn vẹn trước khi luồng chính chạy tiếp
    uint32_t ms = HAL_GetTick() - t0;

    r->n         = imu_stat_n;
    r->expected  = ms / 2;
    r->errors    = imu_i2c_errors + imu_read_fail - err0;
    r->yaw_drift = yaw_angle - yaw0;
    r->seconds   = ms / 1000.0f;
    r->speed_l   = speed_left_mms;
    r->speed_r   = speed_right_mms;
    r->still_pct = (r->n > 0) ? 100.0f * imu_stat_still / r->n : 0.0f;
    r->gz_off    = gyro_z_offset;
    for (int c = 0; c < IMU_CH_COUNT; c++) {
        if (r->n == 0) {
            r->mean[c] = r->std[c] = r->p2p[c] = 0.0f;
            continue;
        }
        double mean = (double)imu_stat[c].sum / r->n;
        double var  = (double)imu_stat[c].sumsq / r->n - mean * mean;
        r->mean[c] = (float)mean;
        r->std[c]  = (var > 0.0) ? (float)sqrt(var) : 0.0f;
        r->p2p[c]  = (float)(imu_stat[c].max - imu_stat[c].min);
    }
}
static uint32_t IMU_Lost(const IMU_Result *r)
{
    return (r->expected > r->n) ? r->expected - r->n : 0;
}
// Bias Gz còn lại (độ/s) = Gz trung bình trừ offset đang dùng: phần yaw thật sự cộng nhầm khi xe chạy
static float IMU_GzResidual(const IMU_Result *r)
{
    return (r->mean[CH_GZ] - r->gz_off) / 16.4f;
}
// 1 dòng bảng kết quả. Gyro: độ/s (chia 16.4), accel: mg (chia ACCEL_LSB_PER_G x 1000)
static void IMU_PrintRow(const char *name, const IMU_Result *r)
{
    snprintf(msg, sizeof(msg),
             "%-22s %6.2f %+7.3f %7.3f %8.2f %7.3f %7.3f %7.2f %7.2f %7.2f %8.2f %6.0f %5lu/%-5lu %3lu %4.0f/%-4.0f\r\n",
             name, IMU_TempC(r->mean[CH_TEMP]),
             IMU_GzResidual(r), r->std[CH_GZ] / 16.4f, r->p2p[CH_GZ] / 16.4f,
             r->std[CH_GX] / 16.4f, r->std[CH_GY] / 16.4f,
             r->std[CH_AX] * 1000.0f / ACCEL_LSB_PER_G, r->std[CH_AY] * 1000.0f / ACCEL_LSB_PER_G, r->std[CH_AZ] * 1000.0f / ACCEL_LSB_PER_G,
             (r->seconds > 0.0f) ? r->yaw_drift * 60.0f / r->seconds : 0.0f, r->still_pct,
             (unsigned long)r->n, (unsigned long)r->expected, (unsigned long)r->errors, r->speed_l, r->speed_r);
    CDC_Print(msg);
}
// Tên chip theo WHO_AM_I
static const char *IMU_ChipName(void)
{
    if (mpu_whoami == MPU6500_WHO_AM_I) return "MPU6500";
    if (mpu_whoami == MPU9250_WHO_AM_I) return "loi MPU9250, gyro/accel/nhiet giong MPU6500 -> dung duoc";
    return "LA - khong phai MPU6500/9250, kiem tra chip";
}
// ĐO NHIỄU IMU VÀ ẢNH HƯỞNG CỦA NHIỆT ĐỘ
//   1. Tắt PWM (phanh: 2 chân IN giữ mức cao, không có xung đóng cắt)
//   2. PWM băm xung MPU_TEST_PWM_LOW, bánh chưa quay   -> chỉ có nhiễu điện từ dòng đóng cắt 20kHz
//   3. Bánh quay đều MPU_TEST_SPEED bằng PID           -> nhiễu điện + rung cơ khí
//   4. Theo dõi bias gyro theo nhiệt độ temp_seconds giây, xe đứng yên tắt PWM (0 = bỏ qua)
// KÊ BÁNH LÊN (bước 3 bánh quay ~2.5s). Xe phải đứng yên tuyệt đối, kể cả lúc khởi động (hiệu chỉnh gyro).
// Chạy xong đứng phanh, CHỜ máy tính mở cổng COM rồi mới in (chạy không cắm dây vẫn lấy được kết quả).
void MPU6500_Test(uint16_t temp_seconds)
{
    static const char *names[3] = { "1.Tat PWM (phanh)", "2.PWM bam xung, dung", "3.Banh quay" };
    IMU_Result res[3];
    uint8_t n_res = 0;
    uint16_t n_temp = (temp_seconds > MPU_TEMP_LOG_MAX) ? MPU_TEMP_LOG_MAX : temp_seconds;

    if (!imu_ready) {
        CDC_WaitHost();
        CDC_Print("\r\nLOI: IMU chua san sang (MPU6500_Setup loi hoac chua goi) - khong test duoc\r\n");
        return;
    }

    // 1. Tắt PWM
    Motor_Stop();
    HAL_Delay(300);
    IMU_Measure(MPU_TEST_SAMPLES, &res[n_res++]);

    if (motor_hw_ok) {
        // 2. PWM băm xung, bánh đứng yên (PID đang tắt nên ngắt TIM9 không ghi đè PWM này)
        Motor_SetPWM(MPU_TEST_PWM_LOW, MPU_TEST_PWM_LOW);
        HAL_Delay(300);
        IMU_Measure(MPU_TEST_SAMPLES, &res[n_res++]);

        // 3. Bánh quay đều: chờ tăng tốc xong rồi mới đo
        Motor_SetSpeed(MPU_TEST_SPEED, MPU_TEST_SPEED);
        HAL_Delay(600);
        IMU_Measure(MPU_TEST_SAMPLES, &res[n_res++]);
        Motor_SetSpeed(0.0f, 0.0f);
        HAL_Delay(300);
    }
    Motor_Stop();

    // 4. Theo dõi nhiệt độ: mỗi giây 1 điểm (trung bình 500 mẫu)
    float yaw_start = yaw_angle;
    for (uint16_t i = 0; i < n_temp; i++) {
        IMU_Result r;
        IMU_Measure(500, &r);
        imu_temp_log[i].temp = IMU_TempC(r.mean[CH_TEMP]);
        imu_temp_log[i].gx   = (r.mean[CH_GX] - gyro_x_offset) / 16.4f;
        imu_temp_log[i].gy   = (r.mean[CH_GY] - gyro_y_offset) / 16.4f;
        imu_temp_log[i].gz   = (r.mean[CH_GZ] - gyro_z_cal) / 16.4f;
        imu_temp_log[i].gz_off = (r.gz_off - gyro_z_cal) / 16.4f;
        imu_temp_log[i].yaw  = yaw_angle - yaw_start;
    }

    // 5. Chờ máy tính rồi in báo cáo
    CDC_WaitHost();
    CDC_Print("\r\n================ TEST MPU6500 ================\r\n");
    snprintf(msg, sizeof(msg), "Chip: WHO_AM_I=0x%02X (%s)\r\n", mpu_whoami, IMU_ChipName());
    CDC_Print(msg);
    snprintf(msg, sizeof(msg), "Offset gyro luc khoi dong (do/s): X=%+.3f Y=%+.3f Z=%+.3f | Offset Z tu chinh hien tai %+.3f (da sua %+.3f)\r\n",
             gyro_x_offset / 16.4f, gyro_y_offset / 16.4f, gyro_z_cal / 16.4f,
             gyro_z_offset / 16.4f, (gyro_z_offset - gyro_z_cal) / 16.4f);
    CDC_Print(msg);
    snprintf(msg, sizeof(msg), "Bo loc: gyro DLPF cfg %d, accel DLPF cfg %d, bu pitch/roll tau %.2f s | Dung yen: banh <%.1f mm/s, |Gz| <%.1f do/s trong %.2f s\r\n",
             GYRO_DLPF_CFG, ACCEL_DLPF_CFG, ANGLE_COMP_TAU_S, STILL_SPEED_MMS, STILL_GZ_DPS, STILL_HOLD_S);
    CDC_Print(msg);
    snprintf(msg, sizeof(msg), "Moi dieu kien %d mau (%.1f s) | PWM bam xung = %d/%d | Banh quay = %.0f mm/s\r\n",
             MPU_TEST_SAMPLES, MPU_TEST_SAMPLES * CONTROL_DT, MPU_TEST_PWM_LOW, PWM_MAX, MPU_TEST_SPEED);
    CDC_Print(msg);
    if (!motor_hw_ok) CDC_Print("(Dong co dang bi khoa -> bo qua dieu kien 2, 3)\r\n");
    snprintf(msg, sizeof(msg), "%-22s %6s %7s %7s %8s %7s %7s %7s %7s %7s %8s %6s %11s %3s %9s\r\n",
             "Dieu kien", "Nhiet", "GzTB", "GzNhieu", "GzDinh", "GxNhieu", "GyNhieu",
             "AxNhieu", "AyNhieu", "AzNhieu", "YawTroi", "DYen%", "Mau/DuKien", "Loi", "VbanhT/P");
    CDC_Print(msg);
    for (int k = 0; k < n_res; k++) IMU_PrintRow(names[k], &res[k]);
    CDC_Print("Don vi: Nhiet C | Gz/Gx/Gy do/s | Ax/Ay/Az mg | YawTroi do/phut | Vbanh mm/s\r\n");
    CDC_Print("GzTB: Gz trung binh tru offset dang dung = bias con lai | Nhieu: do lech chuan | GzDinh: dinh-dinh\r\n");
    CDC_Print("DYen%: % mau nhan ra dung yen (yaw giu nguyen, offset Z tu chinh) - dieu kien 1, 2 phai ~100, 3 phai 0\r\n");
    CDC_Print("Mau/DuKien: mau nhan duoc / le ra (1 mau 2ms) | Loi: loi I2C + lenh doc DMA bi tu choi\r\n");

    // Nhận xét: bias còn lại là phần yaw cộng nhầm khi xe chạy (nhiễu trung bình 0 thì tích phân tự triệt tiêu)
    float base = res[0].std[CH_GZ] / 16.4f;
    for (int k = 0; k < n_res; k++) {
        float noise = res[k].std[CH_GZ] / 16.4f;
        float resid = fabsf(IMU_GzResidual(&res[k]));
        uint8_t bad_bus = (res[k].errors > 0) || (IMU_Lost(&res[k]) > res[k].expected / 100);
        uint8_t spun = (k == 1) && (fabsf(res[k].speed_l) > 20.0f || fabsf(res[k].speed_r) > 20.0f);
        uint8_t no_still = (k < 2) && !spun && (res[k].still_pct < 50.0f);
        snprintf(msg, sizeof(msg), "%s: nhieu Gz x%.1f so voi tat PWM | bias con lai %.3f do/s (~%.1f do/phut khi chay) %s%s%s%s\r\n",
                 names[k], (base > 0.0f) ? noise / base : 0.0f, resid, resid * 60.0f,
                 (resid > YAW_BIAS_WARN_DPS) ? "CAO" : "OK",
                 no_still ? " | CANH BAO: khong nhan ra dung yen (encoder rung?)" : "",
                 bad_bus ? " | CANH BAO: mat mau/loi I2C" : "",
                 spun ? " | banh da quay -> giam MPU_TEST_PWM_LOW" : "");
        CDC_Print(msg);
    }

    // Nhiệt độ: in từng giây + hệ số trôi bias theo nhiệt độ (đường thẳng bình phương nhỏ nhất)
    if (n_temp > 0) {
        snprintf(msg, sizeof(msg), "---------------- THEO DOI NHIET DO %u s (xe dung yen, tat PWM) ----------------\r\n", n_temp);
        CDC_Print(msg);
        CDC_Print("gz: bias Gz so voi luc khoi dong | gz_off: offset tu chinh so voi luc khoi dong (phai bam theo gz)\r\n");
        CDC_Print("TEMP,t_s,nhiet_C,gx_dps,gy_dps,gz_dps,gz_off_dps,yaw_do\r\n");
        for (uint16_t i = 0; i < n_temp; i++) {
            snprintf(msg, sizeof(msg), "TEMP,%u,%.2f,%+.4f,%+.4f,%+.4f,%+.4f,%.2f\r\n", i + 1,
                     imu_temp_log[i].temp, imu_temp_log[i].gx, imu_temp_log[i].gy, imu_temp_log[i].gz,
                     imu_temp_log[i].gz_off, imu_temp_log[i].yaw);
            CDC_Print(msg);
        }
        double mt = 0.0, mg[3] = { 0.0, 0.0, 0.0 };
        for (uint16_t i = 0; i < n_temp; i++) {
            mt    += imu_temp_log[i].temp;
            mg[0] += imu_temp_log[i].gx;
            mg[1] += imu_temp_log[i].gy;
            mg[2] += imu_temp_log[i].gz;
        }
        mt /= n_temp;
        for (int a = 0; a < 3; a++) mg[a] /= n_temp;
        double stt = 0.0, stg[3] = { 0.0, 0.0, 0.0 };
        for (uint16_t i = 0; i < n_temp; i++) {
            double d = imu_temp_log[i].temp - mt;
            stt    += d * d;
            stg[0] += d * (imu_temp_log[i].gx - mg[0]);
            stg[1] += d * (imu_temp_log[i].gy - mg[1]);
            stg[2] += d * (imu_temp_log[i].gz - mg[2]);
        }
        float t_first = imu_temp_log[0].temp, t_last = imu_temp_log[n_temp - 1].temp;
        snprintf(msg, sizeof(msg), "Nhiet do: %.2f -> %.2f C (thay doi %+.2f C) | Yaw troi %.2f do trong %u s (dung yen: phai ~0)\r\n",
                 t_first, t_last, t_last - t_first, imu_temp_log[n_temp - 1].yaw, n_temp);
        CDC_Print(msg);
        snprintf(msg, sizeof(msg), "Bias Gz tu luc khoi dong: %+.4f do/s | offset tu chinh con lech %+.4f do/s\r\n",
                 imu_temp_log[n_temp - 1].gz, imu_temp_log[n_temp - 1].gz - imu_temp_log[n_temp - 1].gz_off);
        CDC_Print(msg);
        if (fabsf(t_last - t_first) >= 1.0f && stt > 0.0) {
            snprintf(msg, sizeof(msg), "He so troi bias theo nhiet do: Gx %+.4f  Gy %+.4f  Gz %+.4f do/s/C\r\n",
                     stg[0] / stt, stg[1] / stt, stg[2] / stt);
        } else {
            snprintf(msg, sizeof(msg), "Nhiet do thay doi < 1 C: chua du de tinh he so troi (chay lau hon tu luc nguoi, hoac lam nong MPU)\r\n");
        }
        CDC_Print(msg);
    }
    CDC_Print("================ HET TEST MPU6500 ================\r\n");
}
