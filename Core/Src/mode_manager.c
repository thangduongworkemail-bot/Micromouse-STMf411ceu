/**
  ******************************************************************************
  * @file    mode_manager.c
  * @brief   Quản lý chế độ chạy + bảo vệ nhấc xe (xem mode_manager.h)
  *
  * Bố cục:
  *   1. Còi: phát mẫu nhịp kêu / nghỉ không chặn
  *   2. Nút ảo IR trái: nhấn = IR > ngưỡng liên tục UI_DEBOUNCE_MS, phải bỏ tay rồi mới nhận lần sau
  *   3. Phát hiện nhấc xe (ngắt TIM9): gia tốc kế / encoder / IR
  *   4. Theo dõi đứng im lúc tạm dừng
  *   5. Máy trạng thái chính
  *
  * QUẢN LÝ THỜI GIAN: mọi chờ đợi = lưu mốc t0 = HAL_GetTick() khi bắt đầu, mỗi vòng lặp so (now - t0) >= thời lượng.
  * Không vòng lặp chờ, không HAL_Delay -> ModeManager_Update() luôn trả về ngay, các ngắt chạy độc lập.
  ******************************************************************************
  */
#include "mode_manager.h"
#include "pid.h"            /* Motor_Stop, tốc độ / tốc độ đặt bánh, quãng đường encoder */
#include "adc.h"            /* adc_value[], IR_*_Filt */
#include "mpu6500.h"        /* robot_lifted, lift_acc_dev, yaw_rate_dps, đo lại gyro không chặn */
#include "solving_maze.h"   /* Maze_* */
#include "flash_storage.h"  /* Flash_LoadMap / Flash_SaveMap (sector 7) */
#include <math.h>           /* fabsf, fmaxf */
#include <stdbool.h>
#include <stdio.h>          /* snprintf */

#define CTRL_TICK_MS        2U                    /* Chu kỳ ngắt TIM9 (CONTROL_DT) */
#define COUNT_OF(a)         (sizeof(a) / sizeof((a)[0]))

static volatile RobotState mm_state = STATE_IDLE; /* Ngắt TIM9 đọc (Check_Robot_Picked_Up), chỉ luồng chính ghi */
static volatile uint8_t    pickup_reason = 0U;    /* PICKUP_BY_* của lần nhấc đang diễn ra */
static char                mm_msg[112];           /* Bộ đệm in UART của module này */

/* =====================================================================================================
 * 1. CÒI: mẫu = mảng thời lượng (ms) xen kẽ KÊU, NGHỈ, KÊU... Bắt đầu: bật còi, mốc bz_t0 = now.
 *    Mỗi Buzzer_Update(now): hết bước hiện tại (now - bz_t0 >= bước) -> mốc bước sau = bz_t0 + bước (cộng đúng
 *    thời lượng, không lấy = now, nên vòng lặp có chậm vài ms cũng không làm nhịp còi trôi dần), đảo còi.
 * =================================================================================================== */
static const uint16_t bz_press[]   = { UI_BEEP_MS };                                  /* Mỗi lần che: 1 tiếng */
static const uint16_t bz_confirm[] = { CONFIRM_LONG_BEEP_MS };                        /* MODE 3: 1 tiếng dài */
static const uint16_t bz_goal[]    = { GOAL_BEEP_MS, GOAL_BEEP_GAP_MS, GOAL_BEEP_MS };
static const uint16_t bz_error[]   = { ERROR_BEEP_MS, ERROR_BEEP_GAP_MS, ERROR_BEEP_MS, ERROR_BEEP_GAP_MS,
                                       ERROR_BEEP_MS };                               /* "tít tít tít" */
static const uint16_t *bz_pat = NULL;              /* Mẫu đang phát, NULL = còi im */
static uint8_t  bz_len  = 0U;
static uint8_t  bz_step = 0U;                      /* Bước đang phát: chẵn = kêu, lẻ = nghỉ */
static uint32_t bz_t0   = 0U;                      /* Mốc HAL_GetTick() lúc bắt đầu bước hiện tại */

static void Buzzer_Out(bool on)
{
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, on ? BUZZER_ON : BUZZER_OFF);
}
static uint16_t bz_lead = 0U;                      /* Khoảng lặng (ms) trước tiếng đầu của mẫu */
/* lead_ms > 0: im lặng chừng này rồi mới kêu tiếng đầu (đổi mẫu lúc còi đang kêu thì 2 tiếng không dính liền) */
static void Buzzer_PlayAfter(const uint16_t *pat, uint8_t len, uint32_t now, uint16_t lead_ms)
{
    bz_pat  = pat;
    bz_len  = len;
    bz_step = 0U;
    bz_t0   = now;
    bz_lead = lead_ms;
    if ((len == 0U) || (pat[0] == 0U)) {
        bz_pat = NULL;                             /* Mẫu rỗng (thời lượng 0): không kêu */
        Buzzer_Out(false);
    } else {
        Buzzer_Out(lead_ms == 0U);
    }
}
#define BUZZ(pat, now)      Buzzer_PlayAfter((pat), (uint8_t)COUNT_OF(pat), (now), 0U)
static void Buzzer_Update(uint32_t now)
{
    if ((bz_pat != NULL) && (bz_lead != 0U) && ((now - bz_t0) >= bz_lead)) {
        bz_t0  += bz_lead;                         /* Hết khoảng lặng: bắt đầu tiếng đầu */
        bz_lead = 0U;
        Buzzer_Out(true);
    }
    while ((bz_pat != NULL) && (bz_lead == 0U) && ((now - bz_t0) >= bz_pat[bz_step])) {
        bz_t0 += bz_pat[bz_step];
        bz_step++;
        if (bz_step >= bz_len) {
            bz_pat = NULL;
            Buzzer_Out(false);
        } else {
            Buzzer_Out((bz_step & 1U) == 0U);
        }
    }
}
static bool Buzzer_Busy(void)
{
    return bz_pat != NULL;
}

/* =====================================================================================================
 * 2. NÚT ẢO = MẮT IR TRÁI (số thô adc_value[1]). Trả true đúng 1 lần cho mỗi lần nhấn: IR > UI_IR_PRESS_RAW liên tục
 *    UI_DEBOUNCE_MS (mốc ui_t_above), và trước đó đã bỏ tay (IR < UI_IR_RELEASE_RAW). Giữ tay mãi không tính 2 lần.
 * =================================================================================================== */
static bool     ui_armed   = false;                /* Đã bỏ tay từ lần nhấn trước (bật nguồn mà tay đang che: chưa nhận) */
static bool     ui_above   = false;                /* Đang trên ngưỡng nhấn */
static uint32_t ui_t_above = 0U;                   /* Mốc lúc bắt đầu trên ngưỡng */

static bool UI_Pressed(uint32_t now)
{
    const uint16_t raw = adc_value[1];
    bool event = false;
    if (raw < UI_IR_RELEASE_RAW) {
        ui_armed = true;
        ui_above = false;
    } else if ((raw > UI_IR_PRESS_RAW) && ui_armed) {
        if (!ui_above) {
            ui_above   = true;
            ui_t_above = now;
        } else if ((now - ui_t_above) >= UI_DEBOUNCE_MS) {
            ui_armed = false;
            ui_above = false;
            event    = true;
        } else {
            /* Đang chờ đủ thời gian chống nhiễu */
        }
    } else {
        ui_above = false;                          /* Giữa 2 ngưỡng, hoặc chưa bỏ tay từ lần trước */
    }
    return event;
}

/* =====================================================================================================
 * 3. PHÁT HIỆN NHẤC XE - gọi trong ngắt TIM9 mỗi 2ms, sau Motor_Speed_Tick (tốc độ + IR vừa cập nhật).
 *    Đếm số chu kỳ 2ms liên tục (không cần HAL_GetTick trong ngắt). Phát hiện -> MPU6500_ForceLifted(): cùng trạng
 *    thái "bị nhấc" như gia tốc kế (hủy chuyển động, khóa động cơ, LED PB15 tắt; đặt xuống do gia tốc kế xác nhận)
 * =================================================================================================== */
static uint16_t pk_enc_n      = 0U;                /* Số chu kỳ bánh vượt tốc độ đặt liên tục */
static uint16_t pk_ir_lost_n  = 0U;                /* Số chu kỳ cả 4 mắt IR "mất hết" liên tục */
static uint16_t pk_ir_seen_age = 0xFFFFU;          /* Số chu kỳ từ lần cuối thấy tường rõ */
static bool     pk_ir_recent  = false;             /* Lúc bắt đầu mất, tường vừa mới thấy */

static bool State_Moving(RobotState s)
{
    return (s == STATE_EXPLORE) || (s == STATE_MODE2_RUN) || (s == STATE_RETURN_HOME);
}
static void Pickup_ResetCounters(void)
{
    pk_enc_n       = 0U;
    pk_ir_lost_n   = 0U;
    pk_ir_seen_age = 0xFFFFU;
    pk_ir_recent   = false;
}
/* Bánh quay nhanh hơn tốc độ đặt (theo chiều đang chạy) quá PICKUP_ENC_OVERSPEED_MMS */
static bool Wheel_Overspeed(float meas, float ref)
{
    bool over = false;
    if (ref > PICKUP_ENC_MIN_REF_MMS) {
        over = (meas - ref) > PICKUP_ENC_OVERSPEED_MMS;
    } else if (ref < -PICKUP_ENC_MIN_REF_MMS) {
        over = (ref - meas) > PICKUP_ENC_OVERSPEED_MMS;
    } else {
        /* Bánh đứng / chạy chậm: không xét */
    }
    return over;
}
uint8_t Check_Robot_Picked_Up(void)
{
    uint8_t reason = 0U;
    if (robot_lifted != 0U) {                      /* Đang bị khóa (IMU: nhấc / va chạm, hoặc lần phát hiện trước) */
        Pickup_ResetCounters();
        if (pickup_reason == 0U) {
            pickup_reason = (imu_state == IMU_STATE_CRASHED) ? PICKUP_BY_CRASH : PICKUP_BY_IMU;
        }
        return pickup_reason;
    }
    pickup_reason = 0U;
    if (!State_Moving(mm_state)) {                 /* Encoder / IR chỉ dùng khi đang chạy MODE */
        Pickup_ResetCounters();
        return 0U;
    }
#if PICKUP_ENC_ENABLE
    if ((motor_pid_enable != 0U) &&
        (Wheel_Overspeed(speed_left_mms, ref_speed_left) || Wheel_Overspeed(speed_right_mms, ref_speed_right))) {
        if (pk_enc_n < 0xFFFFU) {
            pk_enc_n++;
        }
    } else {
        pk_enc_n = 0U;
    }
    if (pk_enc_n >= (PICKUP_ENC_MS / CTRL_TICK_MS)) {
        reason |= PICKUP_BY_ENCODER;
    }
#endif
#if PICKUP_IR_ENABLE
    {
        const float front = fmaxf(IR_FL_Filt(), IR_FR_Filt());
        const float side  = fmaxf(IR_L_Filt(), IR_R_Filt());
        const bool  still = (fabsf(speed_left_mms) < PICKUP_STILL_MMS) && (fabsf(speed_right_mms) < PICKUP_STILL_MMS);
        if ((front >= PICKUP_IR_SEEN_FRONT) || (side >= PICKUP_IR_SEEN_SIDE)) {
            pk_ir_seen_age = 0U;
        } else if (pk_ir_seen_age < 0xFFFFU) {
            pk_ir_seen_age++;
        } else {
            /* Đã lâu không thấy tường */
        }
        if (still && (front < PICKUP_IR_LOST_FRONT) && (side < PICKUP_IR_LOST_SIDE)) {
            if (pk_ir_lost_n == 0U) {
                pk_ir_recent = (pk_ir_seen_age <= (PICKUP_IR_RECENT_MS / CTRL_TICK_MS));
            }
            if (pk_ir_lost_n < 0xFFFFU) {
                pk_ir_lost_n++;
            }
        } else {
            pk_ir_lost_n = 0U;
        }
        if (pk_ir_recent && (pk_ir_lost_n >= (PICKUP_IR_MS / CTRL_TICK_MS))) {
            reason |= PICKUP_BY_IR;
        }
    }
#endif
    if (reason != 0U) {
        pickup_reason = reason;
        MPU6500_ForceLifted();
    }
    return reason;
}

/* =====================================================================================================
 * 4. ĐỨNG IM (lúc tạm dừng): chụp mốc IR / encoder + thời điểm still_t0. Có biến động quá ngưỡng -> chụp lại mốc,
 *    đếm lại từ đầu. Giữ nguyên liên tục PAUSE_STILL_MS (now - still_t0) và gia tốc kế xác nhận đã đặt xuống -> chạy tiếp
 * =================================================================================================== */
static float    still_ir[ADC_IR_CHANNELS];
static float    still_dl  = 0.0f;
static float    still_dr  = 0.0f;
static uint32_t still_t0  = 0U;

static void IR_ReadAll(float ir[ADC_IR_CHANNELS])
{
    ir[0] = IR_FL_Filt();
    ir[1] = IR_L_Filt();
    ir[2] = IR_R_Filt();
    ir[3] = IR_FR_Filt();
}
static void Still_Restart(uint32_t now)
{
    IR_ReadAll(still_ir);
    still_dl = dist_left_mm;
    still_dr = dist_right_mm;
    still_t0 = now;
}
static bool Still_Holding(void)
{
    float ir[ADC_IR_CHANNELS];
    bool  ok = (fabsf(dist_left_mm - still_dl) <= PAUSE_ENC_TOL_MM) && (fabsf(dist_right_mm - still_dr) <= PAUSE_ENC_TOL_MM)
               && (lift_acc_dev <= LIFT_PLACE_ACC_DEV_G) && (fabsf(yaw_rate_dps) <= PAUSE_GYRO_DPS);
    IR_ReadAll(ir);
    for (uint8_t k = 0U; k < ADC_IR_CHANNELS; k++) {
        if (fabsf(ir[k] - still_ir[k]) > fmaxf(PAUSE_IR_TOL, PAUSE_IR_TOL_FRAC * still_ir[k])) {
            ok = false;
        }
    }
    return ok;
}

/* =====================================================================================================
 * 5. MÁY TRẠNG THÁI
 * =================================================================================================== */
static RunMode      mm_mode = MODE_NONE;           /* Mode chọn hợp lệ gần nhất (đang / vừa chạy) */
static uint8_t      ui_count = 0U;                 /* Số lần che mắt trái trong lượt chọn đang diễn ra */
static uint8_t      ui_beeps_due = 0U;             /* Số tiếng đếm che chưa kêu (che nhanh hơn còi) */
static uint32_t     mm_t0   = 0U;                  /* Mốc HAL_GetTick() lúc vào trạng thái hiện tại */
static RobotState   mm_resume_to = STATE_IDLE;     /* Trạng thái đang chạy dở lúc bị nhấc */
static bool         mm_gyro_zeroing = false;       /* STATE_START: đang đo lại offset gyro */
static FlashMapData mm_rec;                        /* Bản ghi Flash (static: 280 byte, không để trên stack) */

static void Enter(RobotState s, uint32_t now)
{
    mm_state = s;
    mm_t0    = now;
}
static const char *Dir_Name(uint8_t h)
{
    static const char *const names[4] = { "Bac", "Dong", "Nam", "Tay" };
    return (h < 4U) ? names[h] : "?";
}
static const char *Mode_Name(RunMode m)
{
    static const char *const names[4] = { "?", "do tiep map cu", "chay nhanh + quay ve", "do map moi" };
    return ((uint32_t)m < 4U) ? names[m] : "?";
}
/* Mỗi lần che: 1 tiếng UI_BEEP_MS, 2 tiếng luôn cách nhau ít nhất UI_BEEP_GAP_MS. Che nhanh hơn còi: dồn lại
 * (ui_beeps_due), Beep_PressDue kêu lần lượt -> số tiếng nghe được luôn bằng số lần che */
static void Beep_Press(uint32_t now)
{
    if (Buzzer_Busy()) {
        if (ui_beeps_due < 0xFFU) {
            ui_beeps_due++;
        }
    } else {
        const uint32_t quiet = now - bz_t0;        /* Còi đã im bao lâu (bz_t0 = lúc tiếng trước tắt) */
        Buzzer_PlayAfter(bz_press, (uint8_t)COUNT_OF(bz_press), now,
                         (quiet < UI_BEEP_GAP_MS) ? (uint16_t)(UI_BEEP_GAP_MS - quiet) : 0U);
    }
}
static void Beep_PressDue(uint32_t now)
{
    if ((ui_beeps_due > 0U) && !Buzzer_Busy()) {
        ui_beeps_due--;
        Beep_Press(now);
    }
}
/* KIỂM TRA AN TOÀN lúc chốt mode (xe đứng yên, chưa chạy). MODE 1 / 2 hợp lệ: bản đồ Flash đã chép vào RAM.
 * Trả NULL = hợp lệ, khác NULL = lý do không chạy */
static const char *Check_Mode(RunMode m)
{
    const char *err = NULL;
    if (m == MODE_3) {
        /* Luôn hợp lệ: Start_Mode3 xóa trắng bản đồ */
    } else if (Flash_LoadMap(&mm_rec) != FLASH_OK) {
        err = "Flash trong, chua co map (che 3 lan = do moi)";
    } else if ((m == MODE_2) && ((mm_rec.flags & FLASH_MAP_HAS_PATH) == 0U)) {
        err = "map do, chua co duong toi dich (che 1 lan = do tiep)";
    } else if (!Maze_ImportMap(mm_rec.maze_map)) {
        err = "map trong Flash hong (che 3 lan = do moi)";
    } else if ((m == MODE_2) && !Maze_HasPath()) {
        err = "map do, chua co duong toi dich (che 1 lan = do tiep)";
    } else {
        /* Hợp lệ */
    }
    return err;
}

/* ===== LƯU FLASH: chỉ ở các mốc xe ĐỨNG YÊN (chạm đích lúc dò, dò xong, bị nhấc, lỗi), không lưu trong lúc chạy.
 * Phanh trước: lúc ghi / xóa Flash CPU dừng đọc lệnh -> ngắt PID bị hoãn (ghi ~vài ms, xóa sector khi đầy ~1-2s) */
static void Save_Map(uint8_t reason)
{
    static const char *const why[] = { "map moi", "cham dich", "do xong", "bi nhac", "loi" };
    const MazeStats *st = Maze_GetStats();
    uint8_t x;
    uint8_t y;
    uint8_t h;
    FlashResult r;
    Motor_Stop();
    Maze_ExportMap(mm_rec.maze_map);
    Maze_GetPose(&x, &y, &h);
    mm_rec.last_x       = x;
    mm_rec.last_y       = y;
    mm_rec.last_heading = h;
    mm_rec.phase        = (uint8_t)Maze_GetPhase();
    mm_rec.rounds       = st->rounds;
    mm_rec.reason       = reason;
    mm_rec.flags        = (uint8_t)((Maze_HasPath() ? FLASH_MAP_HAS_PATH : 0U) | (Maze_IsExplored() ? FLASH_MAP_EXPLORED : 0U));
    mm_rec.reserved     = 0U;
    r = Flash_SaveMap(&mm_rec);
    snprintf(mm_msg, sizeof(mm_msg), "LUU FLASH (%s) lan #%lu: o (%u,%u) huong %s, %s%s -> %s\r\n",
             (reason < 5U) ? why[reason] : "?", (unsigned long)mm_rec.sequence, (unsigned)x, (unsigned)y, Dir_Name(h),
             ((mm_rec.flags & FLASH_MAP_HAS_PATH) != 0U) ? "co duong toi dich" : "chua co duong",
             ((mm_rec.flags & FLASH_MAP_EXPLORED) != 0U) ? " (do xong)" : "", Flash_ResultName(r));
    UART_Print(mm_msg);
}

static void Enter_Idle(void)
{
    Motor_Stop();                                  /* Đứng yên, tắt PID, phanh 2 bánh */
    ui_count     = 0U;
    ui_beeps_due = 0U;
    UART_Print("SAN SANG: che IR trai 1 lan = MODE 1 do tiep, 2 lan = MODE 2 chay nhanh, 3 lan = MODE 3 do moi\r\n");
    Enter(STATE_IDLE, HAL_GetTick());
}
/* Lỗi lúc chạy: lưu Flash nếu đang có lượt chạy (save), gửi bản đồ, 3 bíp, rồi IDLE */
static void Enter_Error(const char *why, bool save)
{
    Motor_Stop();
    snprintf(mm_msg, sizeof(mm_msg), "LOI: %s\r\n", why);
    UART_Print(mm_msg);
    if (save) {
        Save_Map(FLASH_SAVE_ERROR);
    }
    Maze_SendMap();                                /* Xe đứng yên: gửi bản đồ để xem chỗ hỏng (~1s ở 9600 baud) */
    BUZZ(bz_error, HAL_GetTick());
    Enter(STATE_ERROR, HAL_GetTick());
}
static void Enter_Paused(void)
{
    uint8_t x;
    uint8_t y;
    uint8_t h;
    uint8_t why = pickup_reason;
    if ((why == 0U) && (robot_lifted != 0U)) {    /* IMU vừa báo, ngắt TIM9 chưa kịp ghi lý do */
        why = (imu_state == IMU_STATE_CRASHED) ? PICKUP_BY_CRASH : PICKUP_BY_IMU;
    }
    Maze_Pause();                                  /* Chốt ô hiện tại (đang chạy thẳng: tâm ô gần nhất), giữ bản đồ */
    Motor_Stop();
    mm_resume_to = mm_state;
    Maze_GetPose(&x, &y, &h);
    snprintf(mm_msg, sizeof(mm_msg), "TAM DUNG (%s%s%s%s): dat lai o (%u,%u) mui huong %s, dung im %us se chay tiep\r\n",
             ((why & PICKUP_BY_CRASH) != 0U) ? "VA CHAM" : "nhac xe:", (why & PICKUP_BY_IMU) ? " gia toc" : "",
             (why & PICKUP_BY_ENCODER) ? " encoder" : "", (why & PICKUP_BY_IR) ? " IR" : "",
             (unsigned)x, (unsigned)y, Dir_Name(h), (unsigned)(PAUSE_STILL_MS / 1000U));
    UART_Print(mm_msg);
    Save_Map(FLASH_SAVE_LIFTED);                   /* Tắt nguồn lúc đang bị nhấc: lần sau MODE 1 dò tiếp được */
    Still_Restart(HAL_GetTick());
    Enter(STATE_PAUSED, HAL_GetTick());
}

/* MODE 3: xóa trắng bản đồ RAM, ghi 1 bản đồ trống đè lên bản đồ cũ trong Flash (tắt nguồn giữa chừng không bị
 * nhầm sang mê cung cũ), dò từ đầu */
static void Start_Mode3(uint32_t now)
{
    Maze_Init();
    UART_Print("MODE 3: do map moi tu dau\r\n");
    Save_Map(FLASH_SAVE_NEW);
    Enter(STATE_EXPLORE, now);
}
/* MODE 1 (Check_Mode đã chép bản đồ Flash vào RAM): giữ mọi vách đã biết, flood fill dò tiếp phần chưa biết.
 * Map đã đủ vẫn chạy (người dùng chọn MODE 1 thì luôn là MODE 1): tới đích rồi về, kiểm tra lại các vách */
static void Start_Mode1(uint32_t now)
{
    uint8_t x = 0U;
    uint8_t y = 0U;
    uint8_t h = 0U;                                /* (0,0) hướng Bắc */
#if (MODE1_START_AT_SAVED_POS != 0)
    x = mm_rec.last_x;
    y = mm_rec.last_y;
    h = mm_rec.last_heading;
#endif
    snprintf(mm_msg, sizeof(mm_msg), "MODE 1: do tiep map cu (lan luu #%lu) tu o (%u,%u) huong %s%s\r\n",
             (unsigned long)mm_rec.sequence, (unsigned)x, (unsigned)y, Dir_Name(h),
             Maze_IsExplored() ? ", map da du: di kiem tra lai" : "");
    UART_Print(mm_msg);
    Maze_StartContinue(x, y, h);
    Enter(STATE_EXPLORE, now);
}
/* MODE 2 (Check_Mode đã chép bản đồ Flash có đường tới đích vào RAM) */
static void Start_Mode2(uint32_t now)
{
    if (Maze_StartSpeedRun()) {
        UART_Print("MODE 2: chay nhanh toi dich\r\n");
        Enter(STATE_MODE2_RUN, now);
    } else {
        Enter_Error("MODE 2: map khong co duong toi dich", false);
    }
}
static void Start_SelectedMode(uint32_t now)
{
    if (mm_mode == MODE_1) {
        Start_Mode1(now);
    } else if (mm_mode == MODE_2) {
        Start_Mode2(now);
    } else {
        Start_Mode3(now);
    }
}
/* Hết cửa sổ chọn: số lần che = số mode. Kiểm tra an toàn -> hợp lệ: STATE_START (MODE 3 kêu 1 tiếng dài trước),
 * không hợp lệ: 3 bíp lỗi rồi IDLE, xe không chạy */
static void Select_Mode(uint8_t count, uint32_t now)
{
    const char *err = "che qua 3 lan (1 = MODE 1, 2 = MODE 2, 3 = MODE 3)";
    if ((count >= (uint8_t)MODE_1) && (count <= (uint8_t)MODE_3)) {
        err = Check_Mode((RunMode)count);
    }
    if (err != NULL) {
        snprintf(mm_msg, sizeof(mm_msg), "Che %u lan: KHONG CHAY - %s\r\n", (unsigned)count, err);
        UART_Print(mm_msg);
        BUZZ(bz_error, now);
        Enter(STATE_ERROR, now);
    } else {
        mm_mode = (RunMode)count;
        snprintf(mm_msg, sizeof(mm_msg), "Che %u lan: MODE %u (%s)\r\n", (unsigned)count, (unsigned)mm_mode,
                 Mode_Name(mm_mode));
        UART_Print(mm_msg);
        if (mm_mode == MODE_3) {
            BUZZ(bz_confirm, now);                 /* State_Start chờ hết tiếng dài này mới chạy */
        }
        Enter(STATE_START, now);
    }
}
/* STATE_START: (1) đủ MODE_START_DELAY_MS + đã bỏ tay khỏi mắt trái + xe nằm trên sàn + còi xong,
 * (2) đo lại offset gyro Z không chặn (~0.5s, xe đứng yên), (3) bắt đầu mode đã chọn */
static void State_Start(uint32_t now)
{
    if (!mm_gyro_zeroing) {
        if (((now - mm_t0) >= MODE_START_DELAY_MS) && (adc_value[1] < UI_IR_RELEASE_RAW) &&
            (robot_lifted == 0U) && !Buzzer_Busy()) {
            MPU6500_ZeroGyroZ_Start();
            mm_gyro_zeroing = true;
        }
    } else if (MPU6500_ZeroGyroZ_Poll() != GYRO_ZERO_RUNNING) {
        mm_gyro_zeroing = false;                   /* Bị chạm / rung: giữ offset cũ (vẫn tự bám khi đứng yên) */
        Start_SelectedMode(now);
    } else {
        /* Đang đo gyro */
    }
}
/* Máy tính (maze_viewer.py) gửi 1 ký tự = xin bản đồ. Chỉ gọi ở IDLE (gửi ~1s ở 9600 baud, xe đứng yên) */
static void Serve_MapRequest(void)
{
    if (UART_TryRead() >= 0) {
        Maze_SendMap();
    }
}

void ModeManager_Init(void)
{
    Buzzer_Out(false);
    ui_armed = false;
    Pickup_ResetCounters();
    mm_mode = MODE_NONE;
    if (Flash_LoadMap(&mm_rec) == FLASH_OK) {
        snprintf(mm_msg, sizeof(mm_msg), "Flash: co map (lan luu #%lu, %s%s)\r\n", (unsigned long)mm_rec.sequence,
                 ((mm_rec.flags & FLASH_MAP_HAS_PATH) != 0U) ? "co duong toi dich" : "chua co duong",
                 ((mm_rec.flags & FLASH_MAP_EXPLORED) != 0U) ? ", do xong" : "");
        UART_Print(mm_msg);
    } else {
        UART_Print("Flash: trong (chua co map)\r\n");
    }
    Enter_Idle();
}

void ModeManager_Update(void)
{
    const uint32_t now = HAL_GetTick();
    MazeState ms;
    bool pressed;

    Buzzer_Update(now);
    pressed = UI_Pressed(now);                     /* Luôn cập nhật để theo dõi bỏ tay */

    /* BẢO VỆ: đang chạy mà bị nhấc (gia tốc kế / encoder / IR) hoặc lượt chạy tự dừng vì bị nhấc -> tạm dừng */
    if (State_Moving(mm_state) && ((robot_lifted != 0U) || (Maze_GetState() == MAZE_STATE_PAUSED))) {
        Enter_Paused();
        return;
    }

    switch (mm_state) {
    case STATE_IDLE:
        if (pressed) {
            ui_count = 1U;                         /* Che lần đầu: đếm 1, kêu 1 tiếng */
            Beep_Press(now);
            Enter(STATE_UI_BEEP, now);
        } else {
            Serve_MapRequest();
        }
        break;
    case STATE_UI_BEEP:
    case STATE_UI_WINDOW:
        if (pressed) {                             /* Che thêm: đếm thêm 1, kêu 1 tiếng, chờ lại từ đầu */
            if (ui_count < 0xFFU) {
                ui_count++;
            }
            Beep_Press(now);
            Enter(STATE_UI_BEEP, now);
        } else if (mm_state == STATE_UI_BEEP) {
            Beep_PressDue(now);                    /* Còn tiếng đếm dồn lại: kêu tiếp */
            if (!Buzzer_Busy()) {
                Enter(STATE_UI_WINDOW, now);       /* Còi vừa tắt: mốc mm_t0 = now, đếm UI_WINDOW_MS */
            }
        } else if ((now - mm_t0) >= UI_WINDOW_MS) {
            Select_Mode(ui_count, now);            /* Hết 3s không che: chốt số lần che, kiểm tra an toàn */
        } else {
            /* Đang chờ */
        }
        break;
    case STATE_START:
        State_Start(now);
        break;
    case STATE_EXPLORE:
        ms = Maze_Step();
        if (ms == MAZE_STATE_CHECKPOINT) {         /* Dò map chạm đích, xe đứng yên: lưu rồi đi tiếp */
            Save_Map(FLASH_SAVE_GOAL);
            Maze_CheckpointDone();
        } else if (ms == MAZE_STATE_FINISHED) {    /* Dò xong, đã về (0,0) nhìn Bắc */
            Save_Map(FLASH_SAVE_EXPLORED);
            UART_Print("Do map xong: da ve o xuat phat\r\n");
            Maze_SendMap();
            Enter_Idle();                          /* Chạy nhanh: che 2 lần rồi chờ */
        } else if (ms == MAZE_STATE_ERROR) {
            Enter_Error("dang do map thi dung (khong co duong / lech huong / ket banh)", true);
        } else {
            /* Đang dò */
        }
        break;
    case STATE_MODE2_RUN:
        ms = Maze_Step();
        if (ms == MAZE_STATE_FINISHED) {           /* Tới đích: còi 2 nhịp rồi quay về */
            BUZZ(bz_goal, now);
            Enter(STATE_GOAL_BEEP, now);
        } else if (ms == MAZE_STATE_ERROR) {
            Enter_Error("MODE 2 dung giua chung", true);
        } else {
            /* Đang chạy nhanh */
        }
        break;
    case STATE_GOAL_BEEP:
        if (!Buzzer_Busy()) {
            Maze_StartReturn();                    /* Quay 180 độ, đường ngắn nhất về (0,0) */
            Enter(STATE_RETURN_HOME, now);
        }
        break;
    case STATE_RETURN_HOME:
        ms = Maze_Step();
        if (ms == MAZE_STATE_FINISHED) {
            UART_Print("Da ve o xuat phat\r\n");
            Maze_SendMap();
            Enter_Idle();
        } else if (ms == MAZE_STATE_ERROR) {
            Enter_Error("quay ve dung giua chung", true);
        } else {
            /* Đang về */
        }
        break;
    case STATE_PAUSED:
        if (!Still_Holding()) {
            Still_Restart(now);                    /* Còn biến động: đếm lại 5s từ đầu */
        } else if (((now - still_t0) >= PAUSE_STILL_MS) && (robot_lifted == 0U)) {
            Enter(STATE_RESUME, now);
        } else {
            /* Đang đếm 5s đứng im */
        }
        break;
    case STATE_RESUME:
        Maze_Resume();                             /* Đọc lại tường trước, tính lại đường từ ô đã chốt */
        UART_Print("CHAY TIEP\r\n");
        Enter(mm_resume_to, HAL_GetTick());
        break;
    case STATE_ERROR:
        if (!Buzzer_Busy()) {
            Enter_Idle();
        }
        break;
    default:
        Enter_Idle();
        break;
    }
}

RobotState ModeManager_GetState(void)
{
    return mm_state;
}
RunMode ModeManager_GetMode(void)
{
    return mm_mode;
}
