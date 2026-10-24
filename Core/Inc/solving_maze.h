/**
  ******************************************************************************
  * @file    solving_maze.h
  * @brief   Giải mê cung Micromouse bằng flood fill, xe chạy LIÊN TỤC:
  *          gộp các ô thẳng liên tiếp thành 1 đoạn đi thẳng, chỉ hãm và dừng ở tâm ô cần rẽ trái / rẽ phải /
  *          quay đầu (hoặc ô đích). Trong lúc chạy vẫn đọc IR để vạch tường từng ô mà không làm xe dừng.
  *
  * Chia 2 lớp (trong solving_maze.c):
  *   - Lớp thuật toán: bản đồ tường, flood fill, chọn hướng, gộp ô thẳng, máy trạng thái. Không gọi phần cứng.
  *   - Lớp phần cứng (HW_*): bọc các hàm có sẵn của dự án (pid.c, adc.c, mpu6500.c). Đổi phần cứng chỉ sửa lớp này.
  *
  * Cách dùng: do mode_manager.c điều khiển (xe ở TÂM ô xuất phát, mũi hướng Bắc = ra lối mở, đứng yên):
  *     MODE 3 (dò mới):   Maze_Init()          -> Maze_Step() lặp tới FINISHED (khám phá xong, về (0,0), nhìn Bắc)
  *     MODE 1 (dò tiếp):  Maze_ImportMap() + Maze_StartContinue() -> như trên nhưng giữ các vách đã biết
  *     Chạm đích lúc dò: MAZE_STATE_CHECKPOINT (xe đứng yên) -> người gọi lưu Flash -> Maze_CheckpointDone()
  *     MODE 2 (chạy nhanh): Maze_StartSpeedRun() -> Maze_Step() lặp tới FINISHED (tới đích)
  *     Quay về:          Maze_StartReturn()   -> Maze_Step() lặp tới FINISHED (quay 180, về (0,0), nhìn Bắc)
  * Maze_Step() KHÔNG CHẶN ở mọi trạng thái (đi thẳng, quay, căn tường đều bắt đầu rồi theo dõi): gọi càng dày càng tốt.
  * Bị nhấc xe -> MAZE_STATE_PAUSED (giữ bản đồ + ô hiện tại); Maze_Resume() -> chạy tiếp từ ô đó.
  * Gyro: Maze_Init / Maze_StartSpeedRun chỉ lấy hướng hiện tại làm gốc (không đo lại gyro) - người gọi đo trước.
  ******************************************************************************
  */
#ifndef SOLVING_MAZE_H
#define SOLVING_MAZE_H

#include <stdint.h>
#include <stdbool.h>

/* ===== MÊ CUNG ===== */
#define MAZE_SIZE                 16U        /* Số ô mỗi cạnh */
#define MAZE_CELLS                (MAZE_SIZE * MAZE_SIZE)
#define MAZE_CELL_MM              180.0f     /* Khoảng cách tâm 2 ô kề nhau */
#define MAZE_GOAL_LO              7U         /* Đích = 4 ô tâm: x, y trong [MAZE_GOAL_LO, MAZE_GOAL_HI] */
#define MAZE_GOAL_HI              8U
#define MAZE_START_EAST_WALL      1          /* 1: luật thi - ô xuất phát (0,0) có tường phía Đông (3 phía kín) */

/* ===== TỐC ĐỘ / VÒNG KHÁM PHÁ ===== */
#ifndef MAZE_SEARCH_SPEED_MMS
#define MAZE_SEARCH_SPEED_MMS     400.0f     /* Tốc độ khi khám phá (người dùng tăng 250 -> 300, 14/10/2026) */
#endif
#ifndef MAZE_RUN_SPEED_MMS
#define MAZE_RUN_SPEED_MMS        500.0f     /* Tốc độ chạy nhanh. Đi thẳng không còn chặn 350 (chỉ sức động cơ
                                                ~500), nhưng muốn hãm kịp ở ô rẽ: v^2/(2*MOVE_DECEL) < MAZE_SENSE_BEFORE_MM
                                                (90mm -> <= ~420 mm/s) */
#endif
#ifndef MAZE_MAX_ROUNDS
#define MAZE_MAX_ROUNDS           3U         /* Số vòng khám phá tối đa (đi tâm + về) trước khi chạy nhanh */
#endif
#ifndef MAZE_RETURN_SPEED_MMS
#define MAZE_RETURN_SPEED_MMS     400.0f     /* Tốc độ quay về ô xuất phát sau khi chạy nhanh */
#endif
#ifndef MAZE_TURN_TIMEOUT_MS
#define MAZE_TURN_TIMEOUT_MS      2500U      /* Mỗi lần quay tại chỗ tối đa (kẹt bánh -> lỗi) */
#endif
#ifndef MAZE_STRAIGHT_TIMEOUT_MS
#define MAZE_STRAIGHT_TIMEOUT_MS  3000U      /* Mỗi đoạn thẳng tối đa: chừng này + 2 lần thời gian chạy ở tốc độ tối đa */
#endif

/* ===== ĐỌC TƯỜNG BẰNG IR (giá trị IR_*_Filt, thang đã quy đổi; IR lớn = gần tường) =====
 * Đọc tường của ô k khi TRỤC BÁNH còn cách tâm ô k đúng MAZE_SENSE_BEFORE_MM (90 = đúng biên giữa 2 ô):
 *   - 2 mắt bên (gắn xiên ~45 độ, nhìn trước trục ~90mm) thấy giữa vách bên của ô k
 *   - 2 mắt trước cách mặt tường trước của ô k ~114mm
 * Biết ô k phải rẽ thì còn MAZE_SENSE_BEFORE_MM để hãm: cần v^2 / (2 * MOVE_DECEL_MMS2) < MAZE_SENSE_BEFORE_MM
 * (MOVE_DECEL 1000: 90mm cho phép tới ~420 mm/s).
 * HIỆU CHỈNH (print_LED): đặt trục bánh đúng trên biên ô, có / không có vách bên và tường trước của ô kế tiếp, ghi
 * giá trị rồi đặt ngưỡng ở giữa. Mắt bên nhìn ngang (không xiên) thì giảm MAZE_SENSE_BEFORE_MM (~40). */
#ifndef MAZE_SENSE_BEFORE_MM
#define MAZE_SENSE_BEFORE_MM      90.0f
#endif
#ifndef MAZE_SIDE_WALL_IR
#define MAZE_SIDE_WALL_IR         WALL_SIDE_PRESENT_IR   /* IRL thô / IRR (print_LED) >= mức này = có vách bên, dưới =
                                                            không có (170, pid.h - chung với cân giữa ô) */
#endif
#ifndef MAZE_FRONT_FAR_IR
#define MAZE_FRONT_FAR_IR         120.0f     /* TB 2 mắt trước khi cách tường ~114mm: trên mức này = có tường */
#endif
#ifndef MAZE_FRONT_NEAR_IR
#define MAZE_FRONT_NEAR_IR        400.0f     /* TB 2 mắt trước khi đứng ở tâm ô (tường cách ~24mm) */
#endif
#ifndef MAZE_ALIGN_ON_FRONT_WALL
#define MAZE_ALIGN_ON_FRONT_WALL  1          /* 1: dừng ở ô có tường trước thì Wall_Align (sửa hướng gyro + khoảng
                                                cách tới tường) trước khi rẽ; đặt WALL_TARGET_IR = IR ở tâm ô */
#endif
#ifndef MAZE_ALIGN_MIN_INTERVAL_MS
#define MAZE_ALIGN_MIN_INTERVAL_MS 0U        /* Căn tường cách nhau ít nhất chừng này (mỗi lần căn ~1-1.5s gồm 0.5s đo
                                                lại gyro). 0 = lần nào có tường trước cũng căn (hướng chuẩn nhất) */
#endif

/* ===== MÁY TRẠNG THÁI ===== */
typedef enum
{
    MAZE_STATE_IDLE = 0,       /* Chưa Maze_Init */
    MAZE_STATE_DECIDE_PATH,    /* Đứng ở tâm ô: đọc tường trước, flood fill, chọn hướng / số ô đi thẳng */
    MAZE_STATE_MOVE_STRAIGHT,  /* Đang chạy thẳng nhiều ô (không chặn), đọc tường + kéo dài / rút ngắn đoạn */
    MAZE_STATE_TURN_LEFT,      /* Quay trái 90 độ tại tâm ô */
    MAZE_STATE_TURN_RIGHT,     /* Quay phải 90 độ tại tâm ô */
    MAZE_STATE_U_TURN,         /* Quay đầu 180 độ tại tâm ô */
    MAZE_STATE_FINISHED,       /* Xong lượt (khám phá xong / tới đích / về tới xuất phát) */
    MAZE_STATE_ERROR,          /* Không có đường / chuyển động hỏng (lệch hướng, kẹt, quá giờ) */
    MAZE_STATE_PAUSED,         /* Bị nhấc xe: dừng, giữ bản đồ + ô hiện tại, chờ Maze_Resume() */
    MAZE_STATE_ALIGN,          /* Căn tường trước (không chặn) trước khi rẽ */
    MAZE_STATE_CHECKPOINT      /* Đang dò map, vừa chạm đích: xe đứng yên chờ lưu Flash, Maze_CheckpointDone() để đi tiếp */
} MazeState;

typedef enum
{
    MAZE_PHASE_SEARCH_GOAL = 0,  /* Khám phá: xuất phát -> tâm (tường chưa biết coi như mở) */
    MAZE_PHASE_SEARCH_START,     /* Khám phá: tâm -> xuất phát */
    MAZE_PHASE_SPEED_RUN,        /* Chạy nhanh: xuất phát -> tâm, chỉ đi qua tường đã biết */
    MAZE_PHASE_DONE,
    MAZE_PHASE_RETURN            /* Quay về: đích -> xuất phát theo đường ngắn nhất đã biết */
} MazePhase;

/* Thống kê (xem / in sau khi chạy) */
typedef struct
{
    uint16_t straights;        /* Số đoạn đi thẳng đã chạy */
    uint16_t cells;            /* Tổng số ô đã đi */
    uint16_t turns90;          /* Số lần rẽ 90 độ */
    uint16_t uturns;           /* Số lần quay đầu */
    uint16_t extends;          /* Số lần kéo dài đoạn đang chạy */
    uint16_t shortens;         /* Số lần rút ngắn đoạn đang chạy (thấy tường mới) */
    uint16_t wall_stops;       /* Số lần IR trước dừng xe sớm (tường không thấy kịp từ xa) */
    uint16_t run_cells;        /* Số ô của lượt chạy nhanh */
    uint8_t  rounds;           /* Số vòng khám phá đã chạy */
} MazeStats;

/* ===== API ===== */
void      Maze_Init(void);             /* Dò map mới (MODE 3): xóa bản đồ, xe ở (0,0) hướng Bắc, khám phá */
void      Maze_StartContinue(uint8_t x, uint8_t y, uint8_t h);  /* Dò tiếp (MODE 1): GIỮ bản đồ, xe ở ô (x,y) hướng h */
bool      Maze_IsExplored(void);       /* Đường ngắn nhất đã chắc chắn (chỉ qua cạnh đã biết = lạc quan): hết cần dò */
void      Maze_CheckpointDone(void);   /* Đã lưu xong ở MAZE_STATE_CHECKPOINT: đi tiếp */
void      Maze_ExportMap(uint8_t out[MAZE_CELLS]);       /* Chép bản đồ RAM ra (byte ô x*MAZE_SIZE+y) */
bool      Maze_ImportMap(const uint8_t in[MAZE_CELLS]);  /* Nạp bản đồ vào RAM. false: dữ liệu sai (giữ bản đồ cũ) */
bool      Maze_HasPath(void);          /* Đã có đường ngắn nhất xuất phát -> đích CHỈ qua cạnh đã biết */
bool      Maze_StartSpeedRun(void);    /* MODE 2: xe ở (0,0) hướng Bắc, chạy nhanh tới đích. false: chưa có đường */
void      Maze_StartReturn(void);      /* Đang ở đích: quay 180 rồi chạy đường ngắn nhất về (0,0) */
void      Maze_Pause(void);            /* Dừng lượt đang chạy (bị nhấc): chốt ô hiện tại, giữ bản đồ */
void      Maze_Resume(void);           /* Chạy tiếp từ ô đã chốt (xe phải được đặt lại đúng ô, đúng hướng) */
void      Maze_GetPose(uint8_t *x, uint8_t *y, uint8_t *h);   /* Ô + hướng hiện tại (0 Bắc 1 Đông 2 Nam 3 Tây) */
MazeState Maze_Step(void);             /* 1 bước máy trạng thái - gọi lặp trong while(1) */
MazeState Maze_GetState(void);
MazePhase Maze_GetPhase(void);
const MazeStats *Maze_GetStats(void);
void      Maze_PrintMap(void);         /* In bản đồ + vị trí qua UART (chặn ~2s ở 9600 baud): chỉ gọi khi xe đứng */
/* Gửi bản đồ dạng máy đọc cho maze/maze_viewer.py qua UART (chặn ~1s ở 9600 baud): chỉ gọi khi xe đứng yên.
 *   #MAZE_BEGIN <size>
 *   #STATE <MazeState> <MazePhase> <rounds>
 *   #ROBOT <x> <y> <hướng 0 Bắc 1 Đông 2 Nam 3 Tây>
 *   #STATS <straights> <cells> <turns90> <uturns> <extends> <shortens> <wall_stops> <run_cells>
 *   #ROW <y> <size byte hex, x = 0..size-1>   (y từ size-1 xuống 0; byte: bit 0..3 tường N E S W,
 *                                              bit 4..7 cạnh N E S W đã biết)
 *   #MAZE_END <tổng các byte ô, mod 65536>    (kiểm tra mất ký tự qua Bluetooth) */
void      Maze_SendMap(void);

#endif /* SOLVING_MAZE_H */
