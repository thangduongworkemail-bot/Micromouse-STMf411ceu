/**
  ******************************************************************************
  * @file    mpu6500.h
  * @brief   IMU MPU6500 qua I2C1: khởi tạo, hiệu chỉnh gyro, đọc DMA mỗi 2ms,
  *          tính góc pitch/roll (bộ lọc bù) và yaw (tích phân gyro)
  ******************************************************************************
  */
#ifndef MPU6500_H
#define MPU6500_H

#include "main.h"

// ===== THANH GHI / ĐỊA CHỈ =====
#define MPU6500_ADDR          (0x68 << 1) // Địa chỉ I2C 8-bit (0xD0) nếu AD0 nối GND
#define REG_SMPLRT_DIV        0x19        // Sample Rate Divider
#define REG_CONFIG            0x1A        // Cấu hình DLPF & FSYNC (MPU6500: chỉ lọc GYRO)
#define REG_GYRO_CONFIG       0x1B        // Dải đo Gyro
#define REG_ACCEL_CONFIG      0x1C        // Dải đo Accel
#define REG_ACCEL_CONFIG2     0x1D        // Bộ lọc riêng của Accel (MPU6500)
#define REG_PWR_MGMT_1        0x6B        // Quản lý nguồn & Clock Source
#define REG_WHO_AM_I          0x75        // Mã nhận dạng chip
#define MPU6500_WHO_AM_I      0x70        // MPU6500
#define MPU9250_WHO_AM_I      0x71        // Lõi MPU9250 (nhiều module bán tên MPU6500): gyro/accel/nhiệt độ giống hệt

// ===== BỘ LỌC PHẦN CỨNG (chọn theo dữ liệu MPU6500_Test) =====
// Gyro 41Hz (trễ ~5.9ms): rung cơ khí lúc bánh quay chỉ là nhiễu trung bình 0, tích phân yaw tự triệt tiêu,
// hạ xuống 20Hz chỉ làm trễ gần gấp đôi (xấu cho vòng điều khiển góc quay) mà yaw không tốt hơn
#define GYRO_DLPF_CFG         0x03
// Accel ~20Hz: accel chỉ dùng neo pitch/roll (thay đổi rất chậm) -> lọc mạnh không mất gì, giảm rung lọt vào
#define ACCEL_DLPF_CFG        0x04
// Bộ lọc bù pitch/roll: hằng số thời gian (s). Alpha cũ 0.96 ~ 0.05s: xe tăng tốc 2000mm/s2 (0.2g)
// là pitch sai ~11 độ; 0.5s thì gia tốc ngắn của xe gần như không làm lệch góc
#define ANGLE_COMP_TAU_S      0.5f

// ===== YAW: CHIỀU + NHẬN BIẾT ĐỨNG YÊN + TỰ CHỈNH OFFSET GYRO Z =====
// Quy ước: yaw_angle TĂNG khi xe quay TRÁI (nhìn từ trên xuống). Dấu phụ thuộc chiều gắn chip (ngửa/úp):
// xoay tay xe sang trái mà yaw giảm thì đổi dấu. Xe này: gyro Z ngược quy ước -> -1 (đã kiểm tra trên xe)
#define YAW_SIGN              (-1)
// Đứng yên = 2 bánh không quay (encoder) VÀ |Gz| nhỏ, liên tục STILL_HOLD_S giây. Khi đứng yên:
// giữ nguyên yaw và cho offset Gz bám theo bias thật (sai số hiệu chỉnh lúc khởi động, trôi theo nhiệt độ).
// Khi chạy: cộng TOÀN BỘ gyro vào yaw, không có ngưỡng chết (ngưỡng chết cũ 0.8 độ/s nuốt mất các
// góc lệch nhỏ khi chạy thẳng và biến nhiễu rung thành trôi góc ~1.4 độ/phút)
#define STILL_SPEED_MMS       1.0f        // |vận tốc bánh đã lọc| dưới mức này coi như bánh đứng
// |Gz| dưới mức này. Phải lớn hơn sai số offset có thể có: hiệu chỉnh lúc khởi động bị xoay nhẹ (đặt xe xuống)
// từng lệch 2.68 độ/s -> với ngưỡng cũ 1.0 không bao giờ "đứng yên" nên không tự sửa được, xe quay ~12 độ/m.
// Bánh đứng yên thì xe trên sàn không thể quay, nên ngưỡng rộng vẫn an toàn (MPU6500: offset xuất xưởng +-5 độ/s)
#define STILL_GZ_DPS          5.0f
#define GYRO_REZERO_SAMPLES   250         // MPU6500_ZeroGyroZ: lấy trung bình 250 mẫu (0.5s)
#define STILL_HOLD_S          0.25f       // Phải thỏa liên tục chừng này giây mới tính là đứng yên
#define GYRO_BIAS_TAU_S       1.0f        // Tốc độ offset Gz bám theo bias khi đứng yên (hằng số thời gian)

// ===== HIỆU CHỈNH GYRO LÚC KHỞI ĐỘNG =====
#define GYRO_CAL_SAMPLES      500         // Số mẫu lấy trung bình (~1.2s)
#define GYRO_CAL_DISCARD      50          // Bỏ các mẫu đầu: gyro vừa khởi động / DLPF vừa đổi cấu hình
#define GYRO_CAL_MAX_STD_DPS  0.3f        // Nhiễu lớn hơn = xe bị chạm/rung lúc hiệu chỉnh -> đo lại
#define GYRO_CAL_RETRY        3           // Số lần đo lại tối đa

// ===== CẤU HÌNH MPU6500_Test =====
#define MPU_TEST_SAMPLES      1000        // Số mẫu mỗi điều kiện đo nhiễu (1000 x 2ms = 2s)
#define MPU_TEST_PWM_LOW      150         // PWM băm xung mà bánh chưa quay (300: bánh kê lên đã quay 30-50mm/s)
#define MPU_TEST_SPEED        200.0f      // Vận tốc bánh khi đo nhiễu lúc động cơ quay (mm/s)
#define MPU_TEMP_LOG_MAX      600         // Theo dõi nhiệt độ tối đa 600s, 1 điểm/giây (~14KB RAM)
#define YAW_BIAS_WARN_DPS     0.05f       // Bias Gz còn lại lớn hơn mức này (~3 độ/phút) thì báo cao

// ===== KẾT QUẢ =====
extern float pitch_angle;                 // Góc ngóc đầu/chúi mũi (độ)
extern float roll_angle;                  // Góc lật nghiêng xe (độ)
extern float yaw_angle;                   // Góc hướng xe (độ), chỉ tích phân gyro
extern volatile float yaw_rate_dps;       // Tốc độ quay quanh Z (độ/s) đã trừ offset, cùng chiều yaw (dương = trái)
extern volatile int16_t imu_raw[7];       // Mẫu thô mới nhất: ax, ay, az, nhiệt, gx, gy, gz (ghi dữ liệu rung)
extern volatile uint32_t imu_sample_count; // Tăng 1 mỗi mẫu IMU xử lý xong
extern float gyro_x_offset, gyro_y_offset, gyro_z_offset;
extern int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;   // Từ MPU6500_Read_All()
extern int16_t Gyro_X_RAW,  Gyro_Y_RAW,  Gyro_Z_RAW;
extern volatile uint8_t imu_ready;        // 1: ngắt TIM9 đang đọc IMU bằng DMA
extern volatile uint8_t imu_still;        // 1: xe đang đứng yên (yaw giữ nguyên, offset Gz đang tự chỉnh)
extern uint8_t mpu_whoami;                // Đọc lúc khởi tạo: 0x70 MPU6500, 0x71 lõi MPU9250

// ===== KHỞI ĐỘNG (luồng chính) =====
HAL_StatusTypeDef MPU6500_Init(void);
HAL_StatusTypeDef MPU6500_Calibrate(void); // HAL_BUSY: xe không đứng yên sau GYRO_CAL_RETRY lần đo
// Init + Calibrate + cho phép ngắt TIM9 đọc DMA. Khác HAL_OK: MPU lỗi (không đọc) hoặc hiệu chỉnh
// bị rung (vẫn chạy, offset Gz tự sửa dần khi xe đứng yên)
HAL_StatusTypeDef MPU6500_Setup(void);
// Chỉnh lại offset gyro Z khi xe đứng yên trên sàn (luồng chính, sau khi imu_ready = 1, dùng mẫu DMA):
// trung bình GYRO_REZERO_SAMPLES mẫu. Bánh quay / bị chạm (nhiễu lớn) thì giữ offset cũ, trả HAL_BUSY
HAL_StatusTypeDef MPU6500_ZeroGyroZ(void);
extern float gyro_z_rezero_dps;           // Lần chỉnh gần nhất đã sửa offset Z bao nhiêu (độ/s)
void MPU6500_Read_All(void);              // Đọc kiểu chờ, KHÔNG gọi sau khi imu_ready = 1
void I2C_ScanAndPrint_CDC(void);

// ===== GỌI TRONG NGẮT =====
void MPU6500_Tick(void);                  // Ngắt TIM9 mỗi 2ms: ra lệnh đọc 14 byte bằng DMA
void MPU6500_Process_Data(float dt);      // Callback I2C DMA xong: tính góc
uint32_t MPU6500_ErrorCount(void);        // Tổng lỗi bus I2C + lệnh đọc DMA bị từ chối từ lúc khởi động

// ===== DEBUG / TEST (luồng chính) =====
void print_mpu6500(void);
// Đo nhiễu IMU khi tắt PWM / PWM băm xung / bánh quay, rồi theo dõi bias gyro theo nhiệt độ
// trong temp_seconds giây (0 = bỏ qua). KÊ BÁNH LÊN. Xong chờ máy tính mở cổng COM rồi in báo cáo.
void MPU6500_Test(uint16_t temp_seconds);

#endif /* MPU6500_H */
