// THUẬT TOÁN FLOOD FILL CƠ BẢN cho simulator mms (https://github.com/mackorone/mms) - bản C
//
// - Chuột xuất phát ô (0,0) góc dưới-trái, hướng Bắc. Đích = các ô giữa mê cung (2x2 nếu cạnh chẵn).
// - Mỗi ô: đọc tường trái / trước / phải -> ghi vào bản đồ -> flood lại khoảng cách từ mọi ô tới đích
//   (tường CHƯA BIẾT coi như không có) -> đi sang ô kề có khoảng cách nhỏ nhất (ưu tiên đi thẳng).
// - Giai đoạn: 1) xuất phát -> đích (khám phá)  2) đích -> xuất phát (khám phá thêm)
//              lặp lại 1-2 tới khi chắc chắn đã biết đường ngắn nhất (tối đa MAX_ROUNDS lượt)
//              3) xuất phát -> đích lần nữa CHỈ đi qua các ô đã khám phá (đường đã kiểm chứng, không bất ngờ)
// - Bấm Reset trên simulator: chuột về ô xuất phát, giữ nguyên bản đồ tường đã biết, chạy lại từ giai đoạn 1.
//
// Giao tiếp với simulator qua stdin/stdout (API.c) -> log CHỈ in ra stderr.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "API.h"

#define MAX_SIZE    32                  // Mê cung tối đa 32x32 (half-size), thi đấu thường 16x16
#define UNREACHED   9999
#define MAX_ROUNDS  5                   // Số lượt khám phá (đi + về) tối đa

enum { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };
static const int DX[4] = { 0, 1, 0, -1 };
static const int DY[4] = { 1, 0, -1, 0 };
static const char DIR_CHAR[4] = { 'n', 'e', 's', 'w' };

typedef struct {
    int x, y;
} Cell;

static int W = 0, H = 0;                                // Kích thước mê cung
static unsigned char walls[MAX_SIZE * MAX_SIZE];        // Bit d = có tường ở phía d của ô (đã biết)
static int dist[MAX_SIZE * MAX_SIZE];                   // Khoảng cách (số ô) tới đích, tính bằng flood fill
static unsigned char visited[MAX_SIZE * MAX_SIZE];      // 1 = chuột đã tới ô này (biết đủ 4 phía tường)

static int mx = 0, my = 0, mdir = NORTH;                // Vị trí + hướng hiện tại của chuột

// Không đặt tên "log": trùng hàm log() có sẵn của C
static void log_msg(const char *text) {
    fprintf(stderr, "%s\n", text);
    fflush(stderr);
}

static int idx(int x, int y) { return y * W + x; }
static int inside(int x, int y) { return x >= 0 && x < W && y >= 0 && y < H; }
static int hasWall(int x, int y, int d) { return (walls[idx(x, y)] >> d) & 1; }

// Ghi tường ở phía d của ô (x,y) + phía ngược lại của ô kề, vẽ lên simulator
static void addWall(int x, int y, int d) {
    if (hasWall(x, y, d)) return;
    walls[idx(x, y)] |= 1 << d;
    int nx = x + DX[d], ny = y + DY[d];
    if (inside(nx, ny)) walls[idx(nx, ny)] |= 1 << ((d + 2) % 4);
    API_setWall(x, y, DIR_CHAR[d]);
}

// FLOOD FILL (BFS): đích = 0, mỗi bước ra xa thêm 1 ô; chỉ chặn bởi tường ĐÃ BIẾT.
// knownOnly: chỉ lan qua các ô đã khám phá -> đường đi chắc chắn, không gặp tường bất ngờ
static void flood(const Cell *goals, int n_goals, int knownOnly) {
    static Cell queue[MAX_SIZE * MAX_SIZE];             // Mỗi ô vào hàng đợi tối đa 1 lần
    int head = 0, tail = 0;
    for (int i = 0; i < W * H; i++) dist[i] = UNREACHED;
    for (int g = 0; g < n_goals; g++) {
        dist[idx(goals[g].x, goals[g].y)] = 0;
        queue[tail++] = goals[g];
    }
    while (head < tail) {
        int x = queue[head].x, y = queue[head].y;
        head++;
        for (int d = 0; d < 4; d++) {
            int nx = x + DX[d], ny = y + DY[d];
            if (hasWall(x, y, d) || !inside(nx, ny) || dist[idx(nx, ny)] != UNREACHED) continue;
            if (knownOnly && !visited[idx(nx, ny)]) continue;
            dist[idx(nx, ny)] = dist[idx(x, y)] + 1;
            queue[tail].x = nx;
            queue[tail].y = ny;
            tail++;
        }
    }
}

// Hiện khoảng cách lên từng ô của simulator
static void showDist(void) {
    char text[12];
    for (int x = 0; x < W; x++) {
        for (int y = 0; y < H; y++) {
            if (dist[idx(x, y)] == UNREACHED) {
                strcpy(text, "-");
            } else {
                snprintf(text, sizeof(text), "%d", dist[idx(x, y)]);
            }
            API_setText(x, y, text);
        }
    }
}

// Đọc 3 cảm biến (trái / trước / phải so với hướng chuột) -> ghi tường. Phía sau là ô vừa đi qua, đã biết.
static void senseWalls(void) {
    visited[idx(mx, my)] = 1;
    if (API_wallFront()) addWall(mx, my, mdir);
    if (API_wallRight()) addWall(mx, my, (mdir + 1) % 4);
    if (API_wallLeft())  addWall(mx, my, (mdir + 3) % 4);
}

// Quay tới hướng tuyệt đối d
static void turnTo(int d) {
    int diff = (d - mdir + 4) % 4;
    if (diff == 1) {
        API_turnRight();
    } else if (diff == 3) {
        API_turnLeft();
    } else if (diff == 2) {
        API_turnRight();
        API_turnRight();
    }
    mdir = d;
}

// Đi tới 1 trong các ô goals. Trả 0 nếu simulator bị Reset giữa chừng, 1 nếu tới đích.
// knownOnly: chỉ đi qua ô đã khám phá (lần chạy cuối)
static int runTo(const Cell *goals, int n_goals, char color, int knownOnly, int *steps) {
    static const int ORDER[4] = { 0, 1, 3, 2 };         // Thẳng, phải, trái, quay lại
    *steps = 0;
    while (1) {
        if (API_wasReset()) return 0;
        senseWalls();
        flood(goals, n_goals, knownOnly);
        showDist();
        API_setColor(mx, my, color);
        int here = dist[idx(mx, my)];
        if (here == 0) return 1;                        // Tới đích
        if (here == UNREACHED) {                        // Không thể xảy ra với mê cung hợp lệ
            log_msg("LOI: khong con duong toi dich");
            return 1;
        }
        // Ô kề có khoảng cách nhỏ nhất; xét theo thứ tự thẳng, phải, trái, quay lại -> bằng nhau thì đi thẳng
        int best = -1, bestDist = UNREACHED;
        for (int k = 0; k < 4; k++) {
            int d = (mdir + ORDER[k]) % 4;
            int nx = mx + DX[d], ny = my + DY[d];
            if (hasWall(mx, my, d) || !inside(nx, ny)) continue;
            if (dist[idx(nx, ny)] < bestDist) {
                bestDist = dist[idx(nx, ny)];
                best = d;
            }
        }
        turnTo(best);
        if (!API_moveForward()) {                       // Đâm tường: lỗi thuật toán, dừng hẳn
            log_msg("LOI: dam tuong - dung chuong trinh");
            exit(1);
        }
        mx += DX[best];
        my += DY[best];
        (*steps)++;
    }
}

int main(int argc, char *argv[]) {
    (void)argc;
    (void)argv;
    char msg[128];

    W = API_mazeWidth();
    H = API_mazeHeight();
    if (W <= 0 || H <= 0 || W > MAX_SIZE || H > MAX_SIZE) {
        snprintf(msg, sizeof(msg), "LOI: me cung %dx%d ngoai gioi han %dx%d", W, H, MAX_SIZE, MAX_SIZE);
        log_msg(msg);
        return 1;
    }
    memset(walls, 0, sizeof(walls));
    memset(visited, 0, sizeof(visited));

    // Tường bao ngoài luôn có
    for (int x = 0; x < W; x++) {
        walls[idx(x, 0)]     |= 1 << SOUTH;
        walls[idx(x, H - 1)] |= 1 << NORTH;
    }
    for (int y = 0; y < H; y++) {
        walls[idx(0, y)]     |= 1 << WEST;
        walls[idx(W - 1, y)] |= 1 << EAST;
    }

    // Đích: các ô giữa (2x2 nếu cạnh chẵn, 1 ô nếu lẻ)
    Cell center[4], start[1] = { { 0, 0 } };
    int n_center = 0;
    for (int x = (W - 1) / 2; x <= W / 2; x++) {
        for (int y = (H - 1) / 2; y <= H / 2; y++) {
            center[n_center].x = x;
            center[n_center].y = y;
            n_center++;
        }
    }
    snprintf(msg, sizeof(msg), "Flood fill: me cung %dx%d", W, H);
    log_msg(msg);

    // Giai đoạn: 0 khám phá tới đích (vàng), 1 về xuất phát (xanh lơ), 2 chạy lại tới đích (xanh lá).
    // Sau mỗi lượt đi + về: nếu đường ngắn nhất qua ô ĐÃ BIẾT còn dài hơn đường ngắn nhất LẠC QUAN (coi ô chưa biết
    // là trống) thì có thể còn đường tắt chưa khám phá -> khám phá thêm 1 lượt (tối đa MAX_ROUNDS).
    // Bằng nhau = chắc chắn đã có đường ngắn nhất thật -> chạy lại.
    static const char *names[3] = { "Kham pha -> dich", "Ve xuat phat", "Chay lai -> dich" };
    static const char colors[3] = { 'y', 'c', 'g' };
    int phase = 0, round = 1;
    while (phase < 3) {
        int steps = 0;
        int ok = (phase == 1) ? runTo(start, 1, colors[phase], 0, &steps)
                              : runTo(center, n_center, colors[phase], phase == 2, &steps);
        if (!ok) {                                      // Reset: về ô xuất phát, giữ bản đồ tường
            API_ackReset();
            mx = 0;
            my = 0;
            mdir = NORTH;
            phase = 0;
            API_clearAllColor();
            log_msg("Reset -> chay lai tu dau (giu ban do tuong)");
            continue;
        }
        snprintf(msg, sizeof(msg), "%s (luot %d): %d o", names[phase], round, steps);
        log_msg(msg);
        if (phase == 1) {                               // Vừa về tới xuất phát: đã đủ để chạy lại chưa?
            flood(center, n_center, 0);
            int optimistic = dist[idx(0, 0)];
            flood(center, n_center, 1);
            int known = dist[idx(0, 0)];
            snprintf(msg, sizeof(msg), "  Duong ngan nhat: da biet %d o, lac quan %d o", known, optimistic);
            log_msg(msg);
            if (known > optimistic && round < MAX_ROUNDS) {
                round++;
                phase = 0;                              // Khám phá thêm
                continue;
            }
        }
        phase++;
    }
    log_msg("Xong");
    return 0;
}
