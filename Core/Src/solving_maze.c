/**
  ******************************************************************************
  * @file    solving_maze.c
  * @brief   Giải mê cung flood fill + chạy liên tục (xem solving_maze.h)
  *
  * Bố cục:
  *   1. Hằng số, dữ liệu
  *   2. Lớp thuật toán: bản đồ tường, flood fill, chọn hướng, GỘP Ô THẲNG, chuyển giai đoạn
  *   3. Lớp phần cứng HW_*: chỗ DUY NHẤT gọi hàm của pid.c / adc.c / mpu6500.c / HAL (đều không chặn)
  *   4. Máy trạng thái (DECIDE_PATH, MOVE_STRAIGHT, TURN_LEFT, TURN_RIGHT, U_TURN, ALIGN, PAUSED)
  *   5. API + in bản đồ
  *
  * Tọa độ: (0,0) = ô xuất phát (góc Tây-Nam), x tăng về Đông, y tăng về Bắc. Hướng: 0 Bắc, 1 Đông, 2 Nam, 3 Tây.
  * Quãng đường đoạn thẳng tính từ TÂM ô bắt đầu: tâm ô thứ k cách k x MAZE_CELL_MM.
  * Phong cách MISRA C:2012 (có chủ ý lệch: không có). Không cấp phát động, không đệ quy, mảng tĩnh, mọi switch có
  * default, kiểu có độ rộng cố định, ép kiểu tường minh, không dùng stdio.
  ******************************************************************************
  */
#include "solving_maze.h"
#include "main.h"           /* HAL_GetTick, UART_Print */
#include "pid.h"            /* Move_Straight_Begin..., Turn_Angle_Begin, Wall_Align_Begin/Poll, Motion_SetOrigin */
#include "adc.h"            /* IR_FL_Filt, IR_FR_Filt, IR_L_Filt, IR_R_Filt */
#include "mpu6500.h"        /* robot_lifted */

/* =====================================================================================================
 * 1. HẰNG SỐ + DỮ LIỆU
 * =================================================================================================== */
#define DIR_N           0U
#define DIR_E           1U
#define DIR_S           2U
#define DIR_W           3U
#define DIR_NONE        0xFFU
#define REL_STRAIGHT    0U          /* Hướng cần đi so với hướng xe: (d - h) & 3 */
#define REL_RIGHT       1U
#define REL_BACK        2U
#define REL_LEFT        3U
#define DIST_INF        0xFFFFU
#define MAP_LINE_LEN    (4U * MAZE_SIZE + 4U)   /* 1 dòng in bản đồ: "+---" x SIZE + "+\r\n" + '\0' */

static const int8_t dir_dx[4] = { 0, 1, 0, -1 };
static const int8_t dir_dy[4] = { 1, 0, -1, 0 };

/* Bản đồ: mỗi ô 1 byte. Bit 0..3: có tường N E S W. Bit 4..7: cạnh N E S W đã biết */
static uint8_t  maze_map[MAZE_SIZE][MAZE_SIZE];
static uint16_t maze_dist[MAZE_SIZE][MAZE_SIZE];   /* Flood fill: số ô tới đích gần nhất, DIST_INF = không tới */
static uint16_t bfs_queue[MAZE_CELLS];             /* Mỗi ô vào hàng đợi tối đa 1 lần */

/* Vị trí + hướng xe theo ô */
static uint8_t   rob_x = 0U;
static uint8_t   rob_y = 0U;
static uint8_t   rob_h = DIR_N;
static MazeState maze_state = MAZE_STATE_IDLE;
static MazePhase maze_phase = MAZE_PHASE_SEARCH_GOAL;
static MazeStats maze_stats;

/* Đoạn thẳng đang chạy */
static uint8_t  st_x0 = 0U;         /* Ô bắt đầu (xe ở tâm ô này lúc bắt đầu) */
static uint8_t  st_y0 = 0U;
static uint8_t  st_h = DIR_N;
static uint8_t  st_cells = 0U;      /* Đích hiện tại = tâm ô thứ st_cells kể từ ô bắt đầu */
static uint8_t  st_sensed = 0U;     /* Đã đọc tường tới ô thứ mấy */
static float    st_speed = MAZE_SEARCH_SPEED_MMS;
static uint32_t st_t0 = 0U;
/* Lệnh quay / căn tường đang theo dõi (không chặn) */
static bool     act_started = false;  /* Đã bắt đầu lệnh của trạng thái hiện tại */
static uint32_t act_t0 = 0U;          /* Lúc bắt đầu quay (giới hạn MAZE_TURN_TIMEOUT_MS) */
#if (MAZE_ALIGN_ON_FRONT_WALL != 0)
static uint32_t  align_t = 0U;        /* Lúc căn tường gần nhất */
static bool      align_done = false;  /* Đã căn lần nào chưa */
static MazeState align_next = MAZE_STATE_DECIDE_PATH;  /* Lệnh quay sau khi căn xong */
#endif

/* =====================================================================================================
 * 2. LỚP THUẬT TOÁN (không gọi phần cứng)
 * =================================================================================================== */
static uint8_t Dir_Left(uint8_t h)  { return (uint8_t)((h + 3U) & 3U); }
static uint8_t Dir_Right(uint8_t h) { return (uint8_t)((h + 1U) & 3U); }
static uint8_t Dir_Back(uint8_t h)  { return (uint8_t)((h + 2U) & 3U); }
static uint8_t Wall_Bit(uint8_t d)  { return (uint8_t)(1U << d); }
static uint8_t Known_Bit(uint8_t d) { return (uint8_t)(0x10U << d); }

static bool Cell_Inside(int16_t x, int16_t y)
{
    return (x >= 0) && (y >= 0) && (x < (int16_t)MAZE_SIZE) && (y < (int16_t)MAZE_SIZE);
}
/* Ô thứ k theo hướng h kể từ (x0, y0) - người gọi bảo đảm còn trong mê cung */
static uint8_t Cell_StepX(uint8_t x0, uint8_t h, uint8_t k)
{
    return (uint8_t)((int16_t)x0 + ((int16_t)k * (int16_t)dir_dx[h]));
}
static uint8_t Cell_StepY(uint8_t y0, uint8_t h, uint8_t k)
{
    return (uint8_t)((int16_t)y0 + ((int16_t)k * (int16_t)dir_dy[h]));
}
static bool Cell_IsGoal(uint8_t x, uint8_t y)
{
    return (x >= MAZE_GOAL_LO) && (x <= MAZE_GOAL_HI) && (y >= MAZE_GOAL_LO) && (y <= MAZE_GOAL_HI);
}
/* Ô đích của giai đoạn: về xuất phát (khám phá về / quay về) -> (0,0), còn lại -> 4 ô tâm */
static bool Cell_IsTarget(uint8_t x, uint8_t y, MazePhase ph)
{
    bool t;
    if ((ph == MAZE_PHASE_SEARCH_START) || (ph == MAZE_PHASE_RETURN))
    {
        t = (x == 0U) && (y == 0U);
    }
    else
    {
        t = Cell_IsGoal(x, y);
    }
    return t;
}

static bool Map_IsKnown(uint8_t x, uint8_t y, uint8_t d) { return (maze_map[x][y] & Known_Bit(d)) != 0U; }
static bool Map_HasWall(uint8_t x, uint8_t y, uint8_t d) { return (maze_map[x][y] & Wall_Bit(d)) != 0U; }

/* Ghi 1 cạnh, cả 2 phía (ô bên kia ghi hướng ngược lại).
 * force = false: chỉ ghi khi cạnh chưa biết (đọc lúc đang chạy / từ xa kém tin cậy hơn lúc đứng ở tâm ô) */
static void Map_SetEdge(uint8_t x, uint8_t y, uint8_t d, bool wall, bool force)
{
    if (force || (!Map_IsKnown(x, y, d)))
    {
        const int16_t nx = (int16_t)x + (int16_t)dir_dx[d];
        const int16_t ny = (int16_t)y + (int16_t)dir_dy[d];
        maze_map[x][y] |= Known_Bit(d);
        if (wall)
        {
            maze_map[x][y] |= Wall_Bit(d);
        }
        else
        {
            maze_map[x][y] &= (uint8_t)(~Wall_Bit(d));
        }
        if (Cell_Inside(nx, ny))
        {
            const uint8_t od = Dir_Back(d);
            maze_map[(uint8_t)nx][(uint8_t)ny] |= Known_Bit(od);
            if (wall)
            {
                maze_map[(uint8_t)nx][(uint8_t)ny] |= Wall_Bit(od);
            }
            else
            {
                maze_map[(uint8_t)nx][(uint8_t)ny] &= (uint8_t)(~Wall_Bit(od));
            }
        }
    }
}
/* Đi được từ (x,y) sang ô kề theo d: còn trong mê cung và (đã biết không tường, hoặc chưa biết mà optimistic).
 * optimistic = true khi khám phá (tường chưa biết coi như mở), false khi chạy nhanh (chỉ tin cạnh đã biết) */
static bool Map_CanPass(uint8_t x, uint8_t y, uint8_t d, bool optimistic)
{
    bool ok = false;
    const int16_t nx = (int16_t)x + (int16_t)dir_dx[d];
    const int16_t ny = (int16_t)y + (int16_t)dir_dy[d];
    if (Cell_Inside(nx, ny))
    {
        if (Map_IsKnown(x, y, d))
        {
            ok = !Map_HasWall(x, y, d);
        }
        else
        {
            ok = optimistic;
        }
    }
    return ok;
}
/* Bản đồ trống: biên ngoài là tường, ô xuất phát kín phía Đông (luật thi) */
static void Map_Init(void)
{
    uint8_t i;
    uint8_t j;
    for (i = 0U; i < MAZE_SIZE; i++)
    {
        for (j = 0U; j < MAZE_SIZE; j++)
        {
            maze_map[i][j] = 0U;
            maze_dist[i][j] = DIST_INF;
        }
    }
    for (i = 0U; i < MAZE_SIZE; i++)
    {
        Map_SetEdge(i, 0U, DIR_S, true, true);
        Map_SetEdge(i, (uint8_t)(MAZE_SIZE - 1U), DIR_N, true, true);
        Map_SetEdge(0U, i, DIR_W, true, true);
        Map_SetEdge((uint8_t)(MAZE_SIZE - 1U), i, DIR_E, true, true);
    }
#if (MAZE_START_EAST_WALL != 0)
    Map_SetEdge(0U, 0U, DIR_E, true, true);
#endif
}

/* FLOOD FILL: maze_dist = số ô từ mỗi ô tới ô đích gần nhất của giai đoạn ph. BFS nhiều nguồn (mọi ô đích vào hàng
 * đợi với khoảng cách 0), lan qua các cạnh Map_CanPass. Không đệ quy, mỗi ô vào hàng đợi 1 lần: O(số ô) */
static void Flood_Fill(MazePhase ph, bool optimistic)
{
    uint16_t head = 0U;
    uint16_t tail = 0U;
    uint8_t  x;
    uint8_t  y;
    uint8_t  d;
    for (x = 0U; x < MAZE_SIZE; x++)
    {
        for (y = 0U; y < MAZE_SIZE; y++)
        {
            if (Cell_IsTarget(x, y, ph))
            {
                maze_dist[x][y] = 0U;
                bfs_queue[tail] = (uint16_t)(((uint16_t)x * MAZE_SIZE) + (uint16_t)y);
                tail++;
            }
            else
            {
                maze_dist[x][y] = DIST_INF;
            }
        }
    }
    while (head < tail)
    {
        const uint16_t idx = bfs_queue[head];
        head++;
        x = (uint8_t)(idx / MAZE_SIZE);
        y = (uint8_t)(idx % MAZE_SIZE);
        for (d = 0U; d < 4U; d++)
        {
            if (Map_CanPass(x, y, d, optimistic))
            {
                const uint8_t nx = (uint8_t)((int16_t)x + (int16_t)dir_dx[d]);
                const uint8_t ny = (uint8_t)((int16_t)y + (int16_t)dir_dy[d]);
                if (maze_dist[nx][ny] == DIST_INF)
                {
                    maze_dist[nx][ny] = (uint16_t)(maze_dist[x][y] + 1U);
                    bfs_queue[tail] = (uint16_t)(((uint16_t)nx * MAZE_SIZE) + (uint16_t)ny);
                    tail++;
                }
            }
        }
    }
}

/* Hướng đi tiếp từ (x,y) khi xe nhìn hướng h (sau Flood_Fill): ô kề có khoảng cách NHỎ HƠN ô hiện tại và nhỏ nhất.
 * Bằng nhau thì ưu tiên ĐI THẲNG -> phải -> trái -> quay đầu: đi thẳng trước để đoạn thẳng dài, ít phải dừng rẽ */
static uint8_t Plan_BestDir(uint8_t x, uint8_t y, uint8_t h, bool optimistic)
{
    static const uint8_t order[4] = { REL_STRAIGHT, REL_RIGHT, REL_LEFT, REL_BACK };
    uint8_t  best = DIR_NONE;
    uint16_t best_dist = maze_dist[x][y];
    uint8_t  i;
    for (i = 0U; i < 4U; i++)
    {
        const uint8_t d = (uint8_t)((h + order[i]) & 3U);
        if (Map_CanPass(x, y, d, optimistic))
        {
            const uint8_t nx = (uint8_t)((int16_t)x + (int16_t)dir_dx[d]);
            const uint8_t ny = (uint8_t)((int16_t)y + (int16_t)dir_dy[d]);
            if (maze_dist[nx][ny] < best_dist)
            {
                best_dist = maze_dist[nx][ny];
                best = d;
            }
        }
    }
    return best;
}

/* GỘP Ô THẲNG (sau Flood_Fill): từ ô (x,y) đi hướng h, đếm số ô đi thẳng liên tiếp mà đường flood fill vẫn giữ hướng
 * h - dừng đếm ở ô đích, hoặc ô mà hướng tốt nhất không còn là h (ô đó phải rẽ / quay đầu -> xe dừng ở TÂM ô đó).
 * Trả về số ô (0: ngay ô (x,y) đã phải rẽ hoặc là đích). Xe chạy 1 đoạn liền tới tâm ô cuối, không dừng ở các ô giữa */
static uint8_t Plan_StraightCells(uint8_t x, uint8_t y, uint8_t h, MazePhase ph, bool optimistic)
{
    uint8_t n = 0U;
    uint8_t cx = x;
    uint8_t cy = y;
    bool    go = true;
    while (go && (n < (MAZE_SIZE - 1U)))
    {
        if (Cell_IsTarget(cx, cy, ph))
        {
            go = false;
        }
        else if (Plan_BestDir(cx, cy, h, optimistic) != h)
        {
            go = false;
        }
        else
        {
            cx = Cell_StepX(cx, h, 1U);
            cy = Cell_StepY(cy, h, 1U);
            n++;
        }
    }
    return n;
}

/* Độ dài đường ngắn nhất xuất phát -> tâm (số ô). optimistic = false: chỉ qua cạnh đã biết mở */
static uint16_t Plan_PathLength(bool optimistic)
{
    Flood_Fill(MAZE_PHASE_SEARCH_GOAL, optimistic);
    return maze_dist[0][0];
}

/* =====================================================================================================
 * 3. LỚP PHẦN CỨNG - chỉ ở đây gọi hàm của dự án (pid.c, adc.c, mpu6500.c, HAL). Mọi hàm đều KHÔNG CHẶN
 * =================================================================================================== */
static float    HW_FrontIR(void)        { return 0.5f * (IR_FL_Filt() + IR_FR_Filt()); }   /* TB 2 mắt trước */
/* Có vách bên: mắt bên (đã lọc) >= MAZE_SIDE_WALL_IR tính theo số của print_LED (IRL thô quy đổi về thang IR_L_Filt) */
static bool     HW_LeftWall(void)       { return IR_L_Filt() >= (float)IR_L_ToRef((uint16_t)MAZE_SIDE_WALL_IR); }
static bool     HW_RightWall(void)      { return IR_R_Filt() >= MAZE_SIDE_WALL_IR; }
static bool     HW_StraightBegin(float mm, float speed) { return Move_Straight_Begin(mm, speed) != 0U; }
static void     HW_StraightSetTarget(float mm)          { Move_Straight_SetTarget(mm); }
static float    HW_StraightTraveled(void)               { return Move_Straight_Traveled(); }
static uint8_t  HW_MotionStatus(void)   { return Motion_Status(); }
static bool     HW_TurnBegin(float deg) { return Turn_Angle_Begin(deg) != 0U; } /* Dương = trái, theo dõi HW_MotionStatus */
static bool HW_SmoothTurnBegin(int8_t dir, float speed) {
    /* Đi từ tâm ô trước -> qua tâm ô góc -> dừng ở tâm ô sau */
    return Smooth_Turn_Begin(dir, MAZE_CELL_MM, MAZE_CELL_MM, 0.0f, speed) != 0U;
}
#if (MAZE_ALIGN_ON_FRONT_WALL != 0)
static bool     HW_AlignBegin(void)     { return Wall_Align_Begin() != 0U; }      /* Căn vuông góc + khoảng cách */
static uint8_t  HW_AlignPoll(void)      { return Wall_Align_Poll(); }             /* WALL_ALIGN_BUSY / _OK / _FAILED */
#endif
static bool     HW_Lifted(void)         { return robot_lifted != 0U; }            /* Đang bị nhấc (mpu6500.c) */
static void     HW_Stop(void)           { Motor_SetSpeed(0.0f, 0.0f); }           /* Giảm tốc rồi dừng */
static void     HW_Brake(void)          { Motor_Stop(); }                         /* Phanh ngay, tắt PID */
static uint32_t HW_Millis(void)         { return HAL_GetTick(); }
static void     HW_SetOrigin(void)      { Motion_SetOrigin(); }   /* Hướng hiện tại = gốc (không đo lại gyro) */
static void     HW_Print(const char *s) { UART_Print(s); }

/* =====================================================================================================
 * 4. MÁY TRẠNG THÁI (không chặn: mỗi lệnh chuyển động bắt đầu 1 lần rồi theo dõi ở các lần gọi sau)
 * =================================================================================================== */
/* Khám phá: tường chưa biết coi như mở. Chạy nhanh / quay về: chỉ đi qua cạnh đã biết là mở */
static bool Phase_Optimistic(MazePhase ph)
{
    return (ph == MAZE_PHASE_SEARCH_GOAL) || (ph == MAZE_PHASE_SEARCH_START);
}
static float Phase_Speed(MazePhase ph)
{
    float v = MAZE_SEARCH_SPEED_MMS;
    if (ph == MAZE_PHASE_SPEED_RUN)
    {
        v = MAZE_RUN_SPEED_MMS;
    }
    else if (ph == MAZE_PHASE_RETURN)
    {
        v = MAZE_RETURN_SPEED_MMS;
    }
    else
    {
        /* Khám phá */
    }
    return v;
}

/* Tới ô đích của giai đoạn: chuyển giai đoạn.
 * Khám phá về tới xuất phát: so đường ngắn nhất CHỈ qua cạnh đã biết với đường lạc quan (cạnh chưa biết coi như mở).
 * Bằng nhau = đường đã biết chắc chắn ngắn nhất -> xong MODE 1 (DONE); chưa bằng -> khám phá thêm vòng nữa (tối đa
 * MAZE_MAX_ROUNDS). Chạy nhanh tới đích / quay về tới xuất phát -> DONE */
static void Phase_Advance(void)
{
    switch (maze_phase)
    {
    case MAZE_PHASE_SEARCH_GOAL:
        maze_phase = MAZE_PHASE_SEARCH_START;
        break;
    case MAZE_PHASE_SEARCH_START:
    {
        const uint16_t known = Plan_PathLength(false);
        const uint16_t optim = Plan_PathLength(true);
        maze_stats.rounds++;
        if ((known != DIST_INF) && ((known == optim) || (maze_stats.rounds >= MAZE_MAX_ROUNDS)))
        {
            maze_phase = MAZE_PHASE_DONE;
        }
        else
        {
            maze_phase = MAZE_PHASE_SEARCH_GOAL;
        }
        break;
    }
    case MAZE_PHASE_SPEED_RUN:
    case MAZE_PHASE_RETURN:
        maze_phase = MAZE_PHASE_DONE;
        break;
    default:
        break;
    }
}

/* Chuyển động hỏng: đang bị nhấc -> PAUSED (giữ ô hiện tại, chờ Maze_Resume), còn lại -> ERROR */
static void Motion_Failed(void)
{
    HW_Brake();
    act_started = false;
    if (HW_Lifted())
    {
        maze_state = MAZE_STATE_PAUSED;
    }
    else
    {
        maze_state = MAZE_STATE_ERROR;
    }
}

/* Bắt đầu đoạn thẳng n ô từ tâm ô hiện tại */
static void Straight_Start(uint8_t n)
{
    st_x0 = rob_x;
    st_y0 = rob_y;
    st_h = rob_h;
    st_cells = n;
    st_sensed = 0U;
    st_speed = Phase_Speed(maze_phase);
    st_t0 = HW_Millis();
    if (HW_StraightBegin((float)n * MAZE_CELL_MM, st_speed))
    {
        maze_stats.straights++;
        maze_state = MAZE_STATE_MOVE_STRAIGHT;
    }
    else
    {
        Motion_Failed();
    }
}

/* Đọc tường ô thứ k của đoạn thẳng. Gọi khi trục bánh còn cách tâm ô k MAZE_SENSE_BEFORE_MM:
 * mắt bên -> vách trái / phải của ô k, mắt trước (từ xa) -> tường phía trước của ô k */
static void Straight_SenseCell(uint8_t k)
{
    const uint8_t x = Cell_StepX(st_x0, st_h, k);
    const uint8_t y = Cell_StepY(st_y0, st_h, k);
    Map_SetEdge(x, y, Dir_Left(st_h), HW_LeftWall(), false);
    Map_SetEdge(x, y, Dir_Right(st_h), HW_RightWall(), false);
    Map_SetEdge(x, y, st_h, HW_FrontIR() > MAZE_FRONT_FAR_IR, false);
}

/* Đã biết tường ô k: tính lại từ ô k, đổi đích đoạn đang chạy nếu cần - XE KHÔNG DỪNG.
 *   ô k là đích / phải rẽ ở ô k -> đích = tâm ô k (còn MAZE_SENSE_BEFORE_MM để hãm)
 *   từ ô k còn đi thẳng được m ô -> đích = tâm ô k + m (kéo dài: xe chạy tiếp không giảm tốc) */
static void Straight_Replan(uint8_t k)
{
    const uint8_t x = Cell_StepX(st_x0, st_h, k);
    const uint8_t y = Cell_StepY(st_y0, st_h, k);
    const bool    opt = Phase_Optimistic(maze_phase);
    uint8_t       want = k;
    if (!Cell_IsTarget(x, y, maze_phase))
    {
        Flood_Fill(maze_phase, opt);
        want = (uint8_t)(k + Plan_StraightCells(x, y, st_h, maze_phase, opt));
    }
    if (want != st_cells)
    {
        if (want > st_cells)
        {
            maze_stats.extends++;
        }
        else
        {
            maze_stats.shortens++;
        }
        st_cells = want;
        HW_StraightSetTarget((float)want * MAZE_CELL_MM);
    }
}

/* Ô hiện tại theo quãng đã đi của đoạn thẳng (làm tròn tới tâm ô gần nhất) */
static void Straight_SetPosition(float traveled)
{
    float   cells_f = (traveled / MAZE_CELL_MM) + 0.5f;
    uint8_t k;
    if (cells_f < 0.0f)
    {
        cells_f = 0.0f;
    }
    k = (uint8_t)cells_f;
    if (k > st_cells)
    {
        k = st_cells;
    }
    rob_x = Cell_StepX(st_x0, st_h, k);
    rob_y = Cell_StepY(st_y0, st_h, k);
    rob_h = st_h;
    maze_stats.cells = (uint16_t)(maze_stats.cells + k);
    if (maze_phase == MAZE_PHASE_SPEED_RUN)
    {
        maze_stats.run_cells = (uint16_t)(maze_stats.run_cells + k);
    }
}

/* Đoạn thẳng kết thúc (xe đứng yên): cập nhật ô hiện tại rồi quyết định tiếp */
static void Straight_Finish(float traveled, uint8_t status)
{
    Straight_SetPosition(traveled);
    if (status == MOTION_STATUS_FAIL)
    {
        Motion_Failed();
    }
    else
    {
        if (status == MOTION_STATUS_WALL)
        {
            /* IR trước dừng xe sớm: có tường mà từ xa không thấy -> ghi chắc chắn */
            maze_stats.wall_stops++;
            Map_SetEdge(rob_x, rob_y, rob_h, true, true);
        }
        maze_state = MAZE_STATE_DECIDE_PATH;
    }
}

/* Phải quay sang hướng d tại tâm ô hiện tại. Có tường trước -> căn tường trước trước (sửa hướng gyro + khoảng cách
 * tới tường, tức vị trí dọc trong ô) rồi mới quay */
static void Turn_Toward(uint8_t d)
{
    const uint8_t rel = (uint8_t)((d + 4U - rob_h) & 3U);
    MazeState     turn;
    if (rel == REL_RIGHT)
    {
        turn = MAZE_STATE_TURN_RIGHT;
    }
    else if (rel == REL_LEFT)
    {
        turn = MAZE_STATE_TURN_LEFT;
    }
    else
    {
        turn = MAZE_STATE_U_TURN;
    }
    act_started = false;
    maze_state = turn;
#if (MAZE_ALIGN_ON_FRONT_WALL != 0)
    if (Map_HasWall(rob_x, rob_y, rob_h) &&
        ((!align_done) || ((HW_Millis() - align_t) >= MAZE_ALIGN_MIN_INTERVAL_MS)))
    {
        align_next = turn;
        maze_state = MAZE_STATE_ALIGN;
    }
#endif
}

/* STATE_DECIDE_PATH: xe đứng ở tâm ô (rob_x, rob_y) nhìn rob_h */
static void State_DecidePath(void)
{
    bool    opt;
    uint8_t d;
    if (HW_Lifted())
    {
        /* 0. Đang bị nhấc: không đọc cảm biến (đọc toàn rác), dừng chờ đặt lại */
        Motion_Failed();
        return;
    }
    /* 1. Tường trước ô hiện tại: đứng yên, tường gần (~24mm) -> tin cậy, ghi đè */
    Map_SetEdge(rob_x, rob_y, rob_h, HW_FrontIR() > MAZE_FRONT_NEAR_IR, true);
    /* 2. Tới ô đích của giai đoạn -> sang giai đoạn sau */
    if (Cell_IsTarget(rob_x, rob_y, maze_phase))
    {
        const bool explore_goal = (maze_phase == MAZE_PHASE_SEARCH_GOAL);
        Phase_Advance();
        if (explore_goal)
        {
            /* Đang dò map, chạm đích: phanh, chờ người gọi lưu Flash rồi Maze_CheckpointDone() mới đi tiếp */
            HW_Brake();
            maze_state = MAZE_STATE_CHECKPOINT;
            return;
        }
    }
    if (maze_phase == MAZE_PHASE_DONE)
    {
        /* 3a. Xong lượt. Đang ở ô xuất phát thì quay mũi về hướng Bắc (sẵn sàng cho lượt sau) rồi mới xong */
        if ((rob_x == 0U) && (rob_y == 0U) && (rob_h != DIR_N))
        {
            Turn_Toward(DIR_N);
        }
        else
        {
            HW_Stop();
            maze_state = MAZE_STATE_FINISHED;
        }
    }
    else
    {
        /* 3b. Flood fill, chọn hướng */
        opt = Phase_Optimistic(maze_phase);
        Flood_Fill(maze_phase, opt);
        d = Plan_BestDir(rob_x, rob_y, rob_h, opt);
        if (d == DIR_NONE)
        {
            HW_Stop();                                      /* Không còn đường tới đích */
            maze_state = MAZE_STATE_ERROR;
        }
        else if (d == rob_h)
        {
            /* 4a. Đi thẳng: gộp mọi ô thẳng phía trước thành 1 đoạn */
            Straight_Start(Plan_StraightCells(rob_x, rob_y, rob_h, maze_phase, opt));
        }
        else
        {
            /* 4b. Phải rẽ / quay đầu ngay tại tâm ô này */
            Turn_Toward(d);
        }
    }
}

/* STATE_MOVE_STRAIGHT (không chặn): tới điểm đọc của ô kế tiếp thì đọc tường + tính lại, hết đoạn thì về DECIDE */
static void State_MoveStraight(void)
{
    const float   traveled = HW_StraightTraveled();
    const uint8_t status = HW_MotionStatus();
    /* Tính theo tốc độ <= MAX_SPEED_MMS: đặt quá sức động cơ thì xe chạy chậm hơn số đặt (pid.c), không bị cắt ngang */
    const float   t_speed  = (st_speed < MAX_SPEED_MMS) ? st_speed : MAX_SPEED_MMS;
    const float   limit_ms = (float)MAZE_STRAIGHT_TIMEOUT_MS + ((2000.0f * (float)st_cells * MAZE_CELL_MM) / t_speed);
    /* 1. Có thể vượt nhiều điểm đọc 1 lúc nếu vòng lặp bị chậm (vd in UART): đọc lần lượt. Bị nhấc thì không đọc */
    while ((!HW_Lifted()) && (st_sensed < st_cells) &&
           (traveled >= ((((float)st_sensed + 1.0f) * MAZE_CELL_MM) - MAZE_SENSE_BEFORE_MM)))
    {
        st_sensed++;
        Straight_SenseCell(st_sensed);
        Straight_Replan(st_sensed);
    }
    /* 2. Hết đoạn / quá giờ */
    if (status != MOTION_STATUS_RUNNING)
    {
        Straight_Finish(traveled, status);
    }
    else if ((float)(HW_Millis() - st_t0) > limit_ms)
    {
        HW_Stop();
        maze_state = MAZE_STATE_ERROR;
    }
    else
    {
        /* Đang chạy: không làm gì */
    }
}

/* STATE_TURN_* (không chặn): lần đầu bắt đầu quay tại tâm ô, các lần sau theo dõi tới khi xong rồi quyết định tiếp */
static void State_Turn(float deg, uint8_t new_h)
{
    if (!act_started)
    {
        if (HW_TurnBegin(deg))
        {
            act_started = true;
            act_t0 = HW_Millis();
            if ((deg > 135.0f) || (deg < -135.0f))
            {
                maze_stats.uturns++;
            }
            else
            {
                maze_stats.turns90++;
            }
        }
        else
        {
            Motion_Failed();
        }
    }
    else
    {
        const uint8_t status = HW_MotionStatus();
        if (status == MOTION_STATUS_RUNNING)
        {
            if ((HW_Millis() - act_t0) > MAZE_TURN_TIMEOUT_MS)
            {
                act_started = false;
                HW_Brake();
                maze_state = MAZE_STATE_ERROR;
            }
        }
        else if (status == MOTION_STATUS_DONE)
        {
            act_started = false;
            rob_h = new_h;
            maze_state = MAZE_STATE_DECIDE_PATH;
        }
        else
        {
            Motion_Failed();
        }
    }
}

#if (MAZE_ALIGN_ON_FRONT_WALL != 0)
/* STATE_ALIGN (không chặn): bắt đầu căn tường trước, theo dõi tới khi xong rồi sang lệnh quay đã chọn.
 * Căn không được không phải lỗi (hướng giữ như cũ, vẫn quay); bị nhấc -> PAUSED */
static void State_Align(void)
{
    if (!act_started)
    {
        align_t = HW_Millis();
        align_done = true;
        if (HW_AlignBegin())
        {
            act_started = true;
        }
        else if (HW_Lifted())
        {
            Motion_Failed();
        }
        else
        {
            maze_state = align_next;
        }
    }
    else if (HW_AlignPoll() != WALL_ALIGN_BUSY)
    {
        act_started = false;
        if (HW_Lifted())
        {
            Motion_Failed();
        }
        else
        {
            maze_state = align_next;
        }
    }
    else
    {
        /* Đang căn */
    }
}
#endif

/* =====================================================================================================
 * 5. API
 * =================================================================================================== */
void Maze_Init(void)
{
    Map_Init();
    rob_x = 0U;
    rob_y = 0U;
    rob_h = DIR_N;
    maze_phase = MAZE_PHASE_SEARCH_GOAL;
    maze_stats = (MazeStats){ 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U };
#if (MAZE_ALIGN_ON_FRONT_WALL != 0)
    align_done = false;
#endif
    act_started = false;
    HW_SetOrigin();
    maze_state = MAZE_STATE_DECIDE_PATH;
}

bool Maze_HasPath(void)
{
    return Plan_PathLength(false) != DIST_INF;
}

bool Maze_StartSpeedRun(void)
{
    const bool ok = Maze_HasPath();
    if (ok)
    {
        rob_x = 0U;
        rob_y = 0U;
        rob_h = DIR_N;
        maze_phase = MAZE_PHASE_SPEED_RUN;
        maze_stats.run_cells = 0U;
        act_started = false;
        HW_SetOrigin();
        maze_state = MAZE_STATE_DECIDE_PATH;
    }
    return ok;
}

void Maze_StartReturn(void)
{
    maze_phase = MAZE_PHASE_RETURN;
    act_started = false;
    maze_state = MAZE_STATE_U_TURN;     /* Quay 180 độ trước, rồi DECIDE_PATH đi đường ngắn nhất về (0,0) */
}

void Maze_StartContinue(uint8_t x, uint8_t y, uint8_t h)
{
    rob_x = (x < MAZE_SIZE) ? x : 0U;
    rob_y = (y < MAZE_SIZE) ? y : 0U;
    rob_h = (uint8_t)(h & 3U);
    maze_phase = Cell_IsGoal(rob_x, rob_y) ? MAZE_PHASE_SEARCH_START : MAZE_PHASE_SEARCH_GOAL;
    maze_stats = (MazeStats){ 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U };
#if (MAZE_ALIGN_ON_FRONT_WALL != 0)
    align_done = false;
#endif
    act_started = false;
    HW_SetOrigin();
    maze_state = MAZE_STATE_DECIDE_PATH;    /* Bản đồ giữ nguyên: flood fill chỉ còn dò các cạnh chưa biết */
}

bool Maze_IsExplored(void)
{
    const uint16_t known = Plan_PathLength(false);
    return (known != DIST_INF) && (known == Plan_PathLength(true));
}

void Maze_CheckpointDone(void)
{
    if (maze_state == MAZE_STATE_CHECKPOINT)
    {
        maze_state = MAZE_STATE_DECIDE_PATH;
    }
}

void Maze_ExportMap(uint8_t out[MAZE_CELLS])
{
    uint8_t x;
    uint8_t y;
    for (x = 0U; x < MAZE_SIZE; x++)
    {
        for (y = 0U; y < MAZE_SIZE; y++)
        {
            out[((uint16_t)x * MAZE_SIZE) + y] = maze_map[x][y];
        }
    }
}

/* Bản đồ nạp vào phải hợp lệ: biên ngoài là tường đã biết, mỗi cạnh trong giống hệt nhau ở 2 ô kề */
static bool Map_Consistent(const uint8_t in[MAZE_CELLS])
{
    bool    ok = true;
    uint8_t x;
    uint8_t y;
    uint8_t d;
    for (x = 0U; (x < MAZE_SIZE) && ok; x++)
    {
        for (y = 0U; (y < MAZE_SIZE) && ok; y++)
        {
            const uint8_t c = in[((uint16_t)x * MAZE_SIZE) + y];
            for (d = 0U; (d < 4U) && ok; d++)
            {
                const int16_t nx = (int16_t)x + (int16_t)dir_dx[d];
                const int16_t ny = (int16_t)y + (int16_t)dir_dy[d];
                const uint8_t mask = (uint8_t)(Known_Bit(d) | Wall_Bit(d));
                if (!Cell_Inside(nx, ny))
                {
                    ok = ((c & mask) == mask);
                }
                else
                {
                    const uint8_t o  = in[((uint16_t)nx * MAZE_SIZE) + (uint16_t)ny];
                    const uint8_t od = Dir_Back(d);
                    const bool    ka = (c & Known_Bit(d)) != 0U;
                    const bool    kb = (o & Known_Bit(od)) != 0U;
                    const bool    wa = (c & Wall_Bit(d)) != 0U;
                    const bool    wb = (o & Wall_Bit(od)) != 0U;
                    ok = (ka == kb) && (wa == wb);
                }
            }
        }
    }
    return ok;
}

bool Maze_ImportMap(const uint8_t in[MAZE_CELLS])
{
    const bool ok = Map_Consistent(in);
    uint8_t    x;
    uint8_t    y;
    if (ok)
    {
        for (x = 0U; x < MAZE_SIZE; x++)
        {
            for (y = 0U; y < MAZE_SIZE; y++)
            {
                maze_map[x][y] = in[((uint16_t)x * MAZE_SIZE) + y];
            }
        }
    }
    return ok;
}

void Maze_Pause(void)
{
    bool running = true;
    switch (maze_state)
    {
    case MAZE_STATE_MOVE_STRAIGHT:
        Straight_SetPosition(HW_StraightTraveled());    /* Đang chạy thẳng: làm tròn tới tâm ô gần nhất */
        break;
#if (MAZE_ALIGN_ON_FRONT_WALL != 0)
    case MAZE_STATE_ALIGN:
        if (act_started)
        {
            (void)HW_AlignPoll();                       /* Kết thúc lệnh căn đang dở (hướng đích về như cũ) */
        }
        break;
#endif
    case MAZE_STATE_DECIDE_PATH:
    case MAZE_STATE_TURN_LEFT:
    case MAZE_STATE_TURN_RIGHT:
    case MAZE_STATE_U_TURN:
    case MAZE_STATE_PAUSED:
    case MAZE_STATE_CHECKPOINT:
        break;                                          /* Đang đứng / quay tại tâm ô: giữ ô + hướng cũ */
    default:
        running = false;                                /* IDLE / FINISHED / ERROR: không có lượt nào đang chạy */
        break;
    }
    if (running)
    {
        HW_Brake();
        act_started = false;
        maze_state = MAZE_STATE_PAUSED;
    }
}

void Maze_Resume(void)
{
    if (maze_state == MAZE_STATE_PAUSED)
    {
        act_started = false;
        maze_state = MAZE_STATE_DECIDE_PATH;           /* Đọc lại tường trước, tính lại đường từ ô đã chốt */
    }
}

void Maze_GetPose(uint8_t *x, uint8_t *y, uint8_t *h)
{
    *x = rob_x;
    *y = rob_y;
    *h = rob_h;
}

MazeState Maze_Step(void)
{
    switch (maze_state)
    {
    case MAZE_STATE_DECIDE_PATH:
        State_DecidePath();
        break;
    case MAZE_STATE_MOVE_STRAIGHT:
        State_MoveStraight();
        break;
    case MAZE_STATE_TURN_LEFT:
        State_Turn(90.0f, Dir_Left(rob_h));
        break;
    case MAZE_STATE_TURN_RIGHT:
        State_Turn(-90.0f, Dir_Right(rob_h));
        break;
    case MAZE_STATE_U_TURN:
        State_Turn(180.0f, Dir_Back(rob_h));
        break;
#if (MAZE_ALIGN_ON_FRONT_WALL != 0)
    case MAZE_STATE_ALIGN:
        State_Align();
        break;
#endif
    case MAZE_STATE_IDLE:
    case MAZE_STATE_FINISHED:
    case MAZE_STATE_ERROR:
    case MAZE_STATE_PAUSED:
    case MAZE_STATE_CHECKPOINT:                         /* Chờ Maze_CheckpointDone() */
    default:
        break;
    }
    return maze_state;
}

MazeState        Maze_GetState(void) { return maze_state; }
MazePhase        Maze_GetPhase(void) { return maze_phase; }
const MazeStats *Maze_GetStats(void) { return &maze_stats; }

/* ===== Gửi bản đồ cho máy tính (maze/maze_viewer.py), định dạng xem solving_maze.h. Không dùng stdio ===== */
static uint16_t Str_Put(char *buf, uint16_t pos, const char *s)
{
    uint16_t p = pos;
    uint16_t i = 0U;
    while (s[i] != '\0')
    {
        buf[p] = s[i];
        p++;
        i++;
    }
    return p;
}
static uint16_t Str_PutU16(char *buf, uint16_t pos, uint16_t v)
{
    char     tmp[5];
    uint8_t  n = 0U;
    uint16_t val = v;
    uint16_t p = pos;
    do
    {
        tmp[n] = (char)('0' + (char)(val % 10U));
        n++;
        val = (uint16_t)(val / 10U);
    } while (val != 0U);
    while (n > 0U)
    {
        n--;
        buf[p] = tmp[n];
        p++;
    }
    return p;
}
static uint16_t Str_PutHex2(char *buf, uint16_t pos, uint8_t v)
{
    static const char hex[16] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };
    buf[pos] = hex[(v >> 4U) & 0x0FU];
    buf[pos + 1U] = hex[v & 0x0FU];
    return (uint16_t)(pos + 2U);
}
static void Str_SendLine(char *buf, uint16_t pos)
{
    buf[pos] = '\r';
    buf[pos + 1U] = '\n';
    buf[pos + 2U] = '\0';
    HW_Print(buf);
}
void Maze_SendMap(void)
{
    char     line[MAP_LINE_LEN + 16U];
    uint16_t p;
    uint16_t sum = 0U;
    uint8_t  row;
    p = Str_Put(line, 0U, "#MAZE_BEGIN ");
    p = Str_PutU16(line, p, (uint16_t)MAZE_SIZE);
    Str_SendLine(line, p);
    p = Str_Put(line, 0U, "#STATE ");
    p = Str_PutU16(line, p, (uint16_t)maze_state);
    p = Str_Put(line, p, " ");
    p = Str_PutU16(line, p, (uint16_t)maze_phase);
    p = Str_Put(line, p, " ");
    p = Str_PutU16(line, p, (uint16_t)maze_stats.rounds);
    Str_SendLine(line, p);
    p = Str_Put(line, 0U, "#ROBOT ");
    p = Str_PutU16(line, p, (uint16_t)rob_x);
    p = Str_Put(line, p, " ");
    p = Str_PutU16(line, p, (uint16_t)rob_y);
    p = Str_Put(line, p, " ");
    p = Str_PutU16(line, p, (uint16_t)rob_h);
    Str_SendLine(line, p);
    p = Str_Put(line, 0U, "#STATS ");
    p = Str_PutU16(line, p, maze_stats.straights);
    p = Str_Put(line, p, " ");
    p = Str_PutU16(line, p, maze_stats.cells);
    p = Str_Put(line, p, " ");
    p = Str_PutU16(line, p, maze_stats.turns90);
    p = Str_Put(line, p, " ");
    p = Str_PutU16(line, p, maze_stats.uturns);
    p = Str_Put(line, p, " ");
    p = Str_PutU16(line, p, maze_stats.extends);
    p = Str_Put(line, p, " ");
    p = Str_PutU16(line, p, maze_stats.shortens);
    p = Str_Put(line, p, " ");
    p = Str_PutU16(line, p, maze_stats.wall_stops);
    p = Str_Put(line, p, " ");
    p = Str_PutU16(line, p, maze_stats.run_cells);
    Str_SendLine(line, p);
    for (row = 0U; row < MAZE_SIZE; row++)
    {
        const uint8_t y = (uint8_t)(MAZE_SIZE - 1U - row);
        uint8_t x;
        p = Str_Put(line, 0U, "#ROW ");
        p = Str_PutU16(line, p, (uint16_t)y);
        p = Str_Put(line, p, " ");
        for (x = 0U; x < MAZE_SIZE; x++)
        {
            p = Str_PutHex2(line, p, maze_map[x][y]);
            sum = (uint16_t)(sum + maze_map[x][y]);
        }
        Str_SendLine(line, p);
    }
    p = Str_Put(line, 0U, "#MAZE_END ");
    p = Str_PutU16(line, p, sum);
    Str_SendLine(line, p);
}

/* In bản đồ (ASCII, Bắc ở trên): "---" / "|" tường đã biết, " . " / ":" chưa biết, chữ ^ > v < = xe */
static void Map_PrintRowWalls(uint8_t y, uint8_t d)
{
    char    line[MAP_LINE_LEN];
    uint8_t x;
    uint8_t p = 0U;
    for (x = 0U; x < MAZE_SIZE; x++)
    {
        const char c = (!Map_IsKnown(x, y, d)) ? '.' : (Map_HasWall(x, y, d) ? '-' : ' ');
        line[p] = '+';
        line[p + 1U] = (c == '.') ? ' ' : c;
        line[p + 2U] = c;
        line[p + 3U] = (c == '.') ? ' ' : c;
        p = (uint8_t)(p + 4U);
    }
    line[p] = '+';
    line[p + 1U] = '\r';
    line[p + 2U] = '\n';
    line[p + 3U] = '\0';
    HW_Print(line);
}
void Maze_PrintMap(void)
{
    static const char robot_ch[4] = { '^', '>', 'v', '<' };
    char    line[MAP_LINE_LEN];
    uint8_t row;
    for (row = 0U; row < MAZE_SIZE; row++)
    {
        const uint8_t y = (uint8_t)(MAZE_SIZE - 1U - row);
        uint8_t x;
        uint8_t p = 0U;
        Map_PrintRowWalls(y, DIR_N);
        for (x = 0U; x < MAZE_SIZE; x++)
        {
            line[p] = (!Map_IsKnown(x, y, DIR_W)) ? ':' : (Map_HasWall(x, y, DIR_W) ? '|' : ' ');
            line[p + 1U] = ' ';
            line[p + 2U] = ((x == rob_x) && (y == rob_y)) ? robot_ch[rob_h & 3U] : ' ';
            line[p + 3U] = ' ';
            p = (uint8_t)(p + 4U);
        }
        line[p] = '|';
        line[p + 1U] = '\r';
        line[p + 2U] = '\n';
        line[p + 3U] = '\0';
        HW_Print(line);
    }
    Map_PrintRowWalls(0U, DIR_S);
}
