/**
  ******************************************************************************
  * @file    mode_manager.h
  * @brief   Quản lý chế độ chạy (Mode Manager) + hệ thống bảo vệ (Safety) cho Micromouse
  *
  * 3 CHẾ ĐỘ (bản đồ lưu ở Flash sector 7, flash_storage.c - còn sau khi tắt nguồn):
  *   MODE 1  dò TIẾP map cũ: đọc Flash, đúng Magic_Word -> chép bản đồ vào RAM, giữ mọi vách đã biết, flood fill dò
  *           tiếp phần chưa biết (map đã đủ vẫn chạy: tới đích rồi về, kiểm tra lại)
  *   MODE 2  chạy nhanh: bản đồ Flash phải có đường xuất phát -> đích chỉ qua cạnh đã biết. Bứt tốc tới đích, còi
  *           2 nhịp (0.5s - nghỉ 0.5s - 0.5s), quay 180 độ, chạy đường ngắn nhất về xuất phát, mũi về Bắc
  *   MODE 3  dò map MỚI: xóa trắng bản đồ RAM (ghi luôn 1 bản đồ trống đè lên bản cũ trong Flash), dò từ đầu
  *   Lưu Flash (xe đứng yên, đã phanh): dò map chạm đích, dò xong, bị nhấc, lỗi. Không lưu trong lúc chạy.
  * GIAO DIỆN bằng cảm biến (không có nút): đưa tay sát mắt IR TRÁI (IRL thô > UI_IR_PRESS_RAW) = 1 lần "che".
  *   SỐ LẦN CHE = SỐ MODE, không gợi ý, không tự đổi mode. Mỗi lần che còi kêu 1 tiếng (0.5s) để đếm.
  *   Che xong, còi tắt + UI_WINDOW_MS (3s) không che nữa -> chốt số lần che, KIỂM TRA AN TOÀN (xe chưa chạy):
  *     Che 1 lần  MODE 1. Flash trống / hỏng            -> 3 bíp lỗi ("tít tít tít"), không chạy, về IDLE
  *     Che 2 lần  MODE 2. Flash trống / hỏng / map dở   -> 3 bíp lỗi, không chạy (map dở = chưa có đường tới đích)
  *     Che 3 lần  MODE 3. Luôn hợp lệ                   -> 1 tiếng bíp dài (CONFIRM_LONG_BEEP_MS) rồi chạy
  *     Che >= 4 lần: không hợp lệ -> 3 bíp lỗi (tránh che lố tay mà xóa mất map)
  * Bảo vệ: bị nhấc (gia tốc kế / encoder / IR, Check_Robot_Picked_Up) -> STATE_PAUSED: động cơ tắt, giữ bản đồ + ô
  *   hiện tại. Đặt lại xuống ĐÚNG Ô ĐÓ, ĐÚNG HƯỚNG ĐÓ (in qua UART), đứng im liên tục PAUSE_STILL_MS (5s) ->
  *   STATE_RESUME -> chạy tiếp phần đang dở.
  *
  * KHÔNG CHẶN: ModeManager_Update() gọi liên tục trong while(1). Không có HAL_Delay: mọi khoảng thời gian (còi,
  * cửa sổ 3s, đứng im 5s) là "mốc HAL_GetTick() lúc bắt đầu + so (now - mốc) >= thời lượng" trong máy trạng thái.
  * Phép trừ không dấu 32 bit nên đúng cả khi HAL_GetTick() tràn (49.7 ngày). Ngắt PID / IR / IMU chạy độc lập.
  * Check_Robot_Picked_Up() chạy trong ngắt TIM9 (2ms) -> phát hiện nhấc xe kể cả lúc luồng chính bận.
  ******************************************************************************
  */
#ifndef MODE_MANAGER_H
#define MODE_MANAGER_H

#include <stdint.h>
#include "main.h"

/* ===== CÒI (PB8, CubeMX: GPIO_Output) ===== */
#define BUZZ_ON_MS               250U      /* MỖI LẦN còi kêu dài chừng này (người dùng đặt 15/10/2026, trước: 2s / 150ms) */
#define BUZZER_PORT              GPIOB
#define BUZZER_PIN               GPIO_PIN_8
#define BUZZER_ON                GPIO_PIN_SET      /* Mức làm còi kêu (còi kêu ở mức thấp thì đổi 2 dòng này) */
#define BUZZER_OFF               GPIO_PIN_RESET

/* ===== NÚT ẢO = MẮT IR TRÁI (adc_value[1], số thô như cột IRL của print_LED) ===== */
#define UI_IR_PRESS_RAW          2000U     /* > mức này = đang nhấn (đưa tay sát mắt trái) */
#define UI_IR_RELEASE_RAW        1500U     /* < mức này = đã bỏ tay; phải bỏ tay rồi mới nhận lần nhấn tiếp */
#define UI_DEBOUNCE_MS           30U       /* Trên ngưỡng liên tục chừng này mới tính 1 lần nhấn (chống nhiễu) */
#define UI_BEEP_MS               BUZZ_ON_MS /* Mỗi lần che: còi kêu 1 tiếng chừng này */
#define UI_BEEP_GAP_MS           300U      /* Che lúc còi đang kêu: lặng chừng này rồi mới kêu tiếng mới (đếm được) */
#define UI_WINDOW_MS             3000U     /* Hết còi mà chừng này không che nữa -> chốt số lần che (người dùng: 3s) */
#define MODE_START_DELAY_MS      500U     /* Chọn xong: chờ bỏ tay ra khỏi xe thêm chừng này rồi mới chạy */

/* ===== CÒI BÁO ===== */
#define CONFIRM_LONG_BEEP_MS     1000U     /* Che 3 lần (MODE 3, luôn hợp lệ): 1 tiếng bíp dài xác nhận rồi chạy */
#define GOAL_BEEP_MS             BUZZ_ON_MS /* MODE 2 tới đích: kêu - nghỉ GOAL_BEEP_GAP_MS - kêu */
#define GOAL_BEEP_GAP_MS         500U
#define ERROR_BEEP_MS            BUZZ_ON_MS /* Lỗi ("tít tít tít"): chọn mode không hợp lệ, kẹt / lệch hướng lúc chạy... */
#define ERROR_BEEP_GAP_MS        150U      /* Nghỉ giữa 2 bíp lỗi (3 bíp dồn dập, khác nhịp 300ms lúc đếm che) */
/* ===== MODE 1 ===== */
#define MODE1_START_AT_SAVED_POS 0         /* 0: dò tiếp từ ô xuất phát (0,0) hướng Bắc. 1: từ ô / hướng lưu cuối cùng
                                              trong Flash (phải đặt xe đúng ô, đúng hướng đó - in qua UART) */

/* ===== TẠM DỪNG / CHẠY TIẾP ===== */
#define PAUSE_STILL_MS           3000U     /* Đứng im liên tục chừng này (và IMU xác nhận đã đặt xuống) -> chạy tiếp */
#define PAUSE_IR_TOL             25.0f     /* 1 mắt IR (đã lọc) đổi quá max(mức này, PAUSE_IR_TOL_FRAC x giá trị) */
#define PAUSE_IR_TOL_FRAC        0.08f     /*   -> chưa đứng im (tay còn gần, đang chỉnh xe), đếm lại từ đầu */
#define PAUSE_ENC_TOL_MM         1.0f      /* Bánh lăn quá chừng này -> chưa đứng im */
#define PAUSE_GYRO_DPS           2.0f      /* Quay nhanh hơn chừng này -> chưa đứng im */
                                           /* Gia tốc: rung quá LIFT_PLACE_ACC_DEV_G (4mg, mpu6500.h) -> chưa đứng im */

/* ===== PHÁT HIỆN NHẤC XE (Check_Robot_Picked_Up, ngắt TIM9) =====
 * 1. Gia tốc kế (mpu6500.c, luôn bật, nhạy nhất): nghiêng > LIFT_TILT_DEG hoặc vận tốc đứng > LIFT_VZ_MS
 * 2. Encoder (chỉ khi đang chạy MODE): bánh quay NHANH HƠN tốc độ đặt quá PICKUP_ENC_OVERSPEED_MMS liên tục
 *    PICKUP_ENC_MS = mất tải (nhấc lên, bánh quay tự do). Va tường / kẹt bánh làm bánh CHẬM lại nên không bị nhầm */
#define PICKUP_ENC_ENABLE        1
#define PICKUP_ENC_OVERSPEED_MMS 120.0f
#define PICKUP_ENC_MIN_REF_MMS   50.0f     /* Chỉ xét bánh đang được lệnh chạy nhanh hơn mức này */
#define PICKUP_ENC_MS            30U
/* 3. IR (chỉ khi đang chạy MODE): xe ĐỨNG YÊN đang thấy tường rõ mà cả 4 mắt cùng tụt về mức không tường =
 *    bị nhấc khỏi mê cung (tường không tự biến mất). Lúc đang chạy thì không dùng: đi ra chỗ trống cũng tụt */
#define PICKUP_IR_ENABLE         0
#define PICKUP_IR_SEEN_FRONT     300.0f    /* "Thấy tường rõ": mắt trước >= mức này ... */
#define PICKUP_IR_SEEN_SIDE      170.0f    /* ... hoặc mắt bên >= mức này (= WALL_SIDE_PRESENT_IR) */
#define PICKUP_IR_LOST_FRONT     130.0f    /* "Mất hết": cả 2 mắt trước < mức này VÀ ... */
#define PICKUP_IR_LOST_SIDE      110.0f    /* ... cả 2 mắt bên < mức này (không tường: ~40-110) */
#define PICKUP_IR_RECENT_MS      300U      /* Lúc bắt đầu mất, tường phải vừa thấy trong khoảng này (mất đột ngột) */
#define PICKUP_IR_MS             60U       /* "Mất hết" liên tục chừng này */
#define PICKUP_STILL_MMS         10.0f     /* Đứng yên: cả 2 bánh chậm hơn mức này */

/* Lý do bị nhấc (Check_Robot_Picked_Up, in khi tạm dừng) */
#define PICKUP_BY_IMU            0x01U
#define PICKUP_BY_ENCODER        0x02U
#define PICKUP_BY_IR             0x04U
#define PICKUP_BY_CRASH          0x08U     /* IMU báo va chạm (độ giật, mpu6500.c) - xử lý như bị nhấc */

/* ===== TRẠNG THÁI ===== */
typedef enum
{
    STATE_IDLE = 0,       /* Đứng yên, động cơ tắt, chờ che mắt IR trái */
    STATE_UI_BEEP,        /* Còi kêu 1 tiếng cho lần che vừa nhận (che nữa = đếm thêm 1) */
    STATE_UI_WINDOW,      /* Hết còi: UI_WINDOW_MS không che -> số lần che = số mode, kiểm tra an toàn */
    STATE_START,          /* Mode hợp lệ: chờ còi xong + bỏ tay + xe nằm trên sàn, đo lại gyro (không chặn), chạy */
    STATE_EXPLORE,        /* MODE 1 (dò tiếp) / MODE 3 (dò mới): dò map */
    STATE_MODE2_RUN,      /* MODE 2: chạy nhanh xuất phát -> đích */
    STATE_GOAL_BEEP,      /* MODE 2 tới đích: còi 2 nhịp */
    STATE_RETURN_HOME,    /* Quay 180 độ, chạy đường ngắn nhất về ô xuất phát */
    STATE_PAUSED,         /* Bị nhấc: động cơ tắt, giữ vị trí trong map, chờ đặt xuống + đứng im 5s */
    STATE_RESUME,         /* Phục hồi rồi quay lại trạng thái đang chạy dở */
    STATE_ERROR           /* Lỗi (chọn mode không hợp lệ / lỗi lúc chạy): 3 bíp rồi về IDLE */
} RobotState;

typedef enum
{
    MODE_NONE = 0,        /* Chưa chọn mode nào từ lúc bật nguồn */
    MODE_1 = 1,           /* Dò tiếp map cũ (từ Flash) */
    MODE_2 = 2,           /* Chạy nhanh + quay về */
    MODE_3 = 3            /* Dò map mới tinh */
} RunMode;

/* ===== API ===== */
void       ModeManager_Init(void);        /* Gọi 1 lần trước while(1), sau MPU6500_Setup() */
void       ModeManager_Update(void);      /* Gọi liên tục trong while(1). Không chặn */
uint8_t    Check_Robot_Picked_Up(void);   /* Ngắt TIM9 sau Motor_Speed_Tick: PICKUP_BY_* đang bị nhấc, 0 = trên sàn */
RobotState ModeManager_GetState(void);
RunMode    ModeManager_GetMode(void);     /* Mode chọn hợp lệ gần nhất (đang / vừa chạy), MODE_NONE: chưa chọn */

#endif /* MODE_MANAGER_H */
