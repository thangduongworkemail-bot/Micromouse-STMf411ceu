
#include <queue>
#include <cstdint>
#include <utility>
#include "API.h"

#define MAZE_SIZE 16
#define INF 9999

// ==================================================
// 1. KHAI BÁO DỮ LIỆU
// ==================================================

// Quy ước hướng: 0 = Bắc, 1 = Đông, 2 = Nam, 3 = Tây.
int dir = 0;
int x = 0, y = 0;

// Mỗi bit biểu diễn trạng thái tường theo một hướng:
// 1 = có tường, 0 = không có tường.
uint8_t walls[MAZE_SIZE][MAZE_SIZE] = {};

// Đánh dấu cạnh đã biết trạng thái hay chưa.
uint8_t known[MAZE_SIZE][MAZE_SIZE] = {};

// Đánh dấu các ô chuột đã đi qua.
bool visited[MAZE_SIZE][MAZE_SIZE] = {};

// Lưu khoảng cách đến các ô cần khám phá.
int dist[MAZE_SIZE][MAZE_SIZE];

// Độ thay đổi tọa độ theo từng hướng.
const int dx[4] = {0, 1, 0, -1};
const int dy[4] = {1, 0, -1, 0};

// Bit tường và ký hiệu hướng tương ứng với từng hướng.
const int bits[4] = {1, 2, 4, 8};
const char wallNames[4] = {'n', 'e', 's', 'w'};


// ==================================================
// 2. CÁC HÀM KIỂM TRA CƠ BẢN
// ==================================================

// Kiểm tra tọa độ có nằm trong mê cung hay không.
bool inside(int cx, int cy) {
    return cx >= 0 && cx < MAZE_SIZE &&
           cy >= 0 && cy < MAZE_SIZE;
}

// Kiểm tra ô có thuộc một trong bốn ô trung tâm hay không.
bool isGoal(int cx, int cy) {
    return (cx == 7 || cx == 8) &&
           (cy == 7 || cy == 8);
}

// Trả về hướng đối diện với hướng d.
int opposite(int d) {
    return (d + 2) % 4;
}


// ==================================================
// 3. CẬP NHẬT BẢN ĐỒ TƯỜNG
// ==================================================

// Cập nhật trạng thái tường của một cạnh và cạnh đối diện
// ở ô bên cạnh, đồng thời cập nhật giao diện MMS.
void setEdge(int cx, int cy, int d, bool hasWall) {
    int nx = cx + dx[d];
    int ny = cy + dy[d];

    known[cx][cy] |= bits[d];

    if (hasWall) {
        walls[cx][cy] |= bits[d];
        API::setWall(cx, cy, wallNames[d]);
    } else {
        walls[cx][cy] &= (uint8_t)(~bits[d]);
        API::clearWall(cx, cy, wallNames[d]);
    }

    // Nếu nằm ngoài mê cung thì không có ô đối diện.
    if (!inside(nx, ny)) {
        return;
    }

    int od = opposite(d);
    known[nx][ny] |= bits[od];

    // Đồng bộ trạng thái cạnh của ô bên cạnh.
    if (hasWall) {
        walls[nx][ny] |= bits[od];
        API::setWall(nx, ny, wallNames[od]);
    } else {
        walls[nx][ny] &= (uint8_t)(~bits[od]);
        API::clearWall(nx, ny, wallNames[od]);
    }
}

// Khởi tạo các cạnh biên của mê cung là tường.
void initBoundary() {
    for (int cx = 0; cx < MAZE_SIZE; cx++) {
        setEdge(cx, 0, 2, true);
        setEdge(cx, MAZE_SIZE - 1, 0, true);
    }

    for (int cy = 0; cy < MAZE_SIZE; cy++) {
        setEdge(0, cy, 3, true);
        setEdge(MAZE_SIZE - 1, cy, 1, true);
    }
}

// Đọc cảm biến và cập nhật tường phía trước, bên trái,
// bên phải theo hướng hiện tại của chuột.
void updateWalls() {
    bool front = API::wallFront();
    bool left = API::wallLeft();
    bool right = API::wallRight();

    setEdge(x, y, dir, front);
    setEdge(x, y, (dir + 3) % 4, left);
    setEdge(x, y, (dir + 1) % 4, right);
}


// ==================================================
// 4. KIỂM TRA ĐƯỜNG ĐI
// ==================================================

// Chỉ cho phép đi qua cạnh đã biết là thông
// và dẫn đến một ô nằm trong mê cung.
bool canMove(int cx, int cy, int d) {
    int nx = cx + dx[d];
    int ny = cy + dy[d];

    if (!inside(nx, ny)) {
        return false;
    }

    if (!(known[cx][cy] & bits[d])) {
        return false;
    }

    if (walls[cx][cy] & bits[d]) {
        return false;
    }

    return true;
}

// Kiểm tra ô còn cạnh chưa được khám phá hay không.
bool hasUnknownEdge(int cx, int cy) {
    for (int d = 0; d < 4; d++) {
        int nx = cx + dx[d];
        int ny = cy + dy[d];

        if (!inside(nx, ny)) {
            continue;
        }

        if (!(known[cx][cy] & bits[d])) {
            return true;
        }
    }

    return false;
}


// ==================================================
// 5. ĐIỀU KHIỂN VÀ DI CHUYỂN
// ==================================================

// Xoay chuột đến hướng đích và cập nhật hướng hiện tại.
void turnTo(int targetDir) {
    int turn = (targetDir - dir + 4) % 4;

    if (turn == 1) {
        API::turnRight();
    } else if (turn == 2) {
        API::turnRight();
        API::turnRight();
    } else if (turn == 3) {
        API::turnLeft();
    }

    dir = targetDir;
}

// Kiểm tra đường đi, xoay chuột và di chuyển một ô.
bool moveOneStep(int d) {
    if (!canMove(x, y, d)) {
        API::setText(x, y, "ERR");
        return false;
    }

    turnTo(d);
    API::moveForward();

    x += dx[d];
    y += dy[d];

    return true;
}


// ==================================================
// 6. FLOOD FILL PHỤC VỤ KHÁM PHÁ
// ==================================================

// Tính khoảng cách từ mỗi ô đến ô còn cạnh chưa khám phá.
// BFS bắt đầu đồng thời từ tất cả các ô này và chỉ lan
// qua những cạnh đã biết là thông.
void computeExploreFloodFill() {
    std::queue<std::pair<int, int>> q;

    // Khởi tạo khoảng cách của tất cả các ô.
    for (int i = 0; i < MAZE_SIZE; i++) {
        for (int j = 0; j < MAZE_SIZE; j++) {
            dist[i][j] = INF;
        }
    }

    // Đưa các ô còn cạnh chưa biết vào hàng đợi.
    for (int i = 0; i < MAZE_SIZE; i++) {
        for (int j = 0; j < MAZE_SIZE; j++) {
            if (hasUnknownEdge(i, j)) {
                dist[i][j] = 0;
                q.push({i, j});
            }
        }
    }

    // Lan truyền khoảng cách qua các cạnh có thể đi qua.
    while (!q.empty()) {
        int cx = q.front().first;
        int cy = q.front().second;
        q.pop();

        for (int d = 0; d < 4; d++) {
            if (!canMove(cx, cy, d)) {
                continue;
            }

            int nx = cx + dx[d];
            int ny = cy + dy[d];

            if (dist[nx][ny] > dist[cx][cy] + 1) {
                dist[nx][ny] = dist[cx][cy] + 1;
                q.push({nx, ny});
            }
        }
    }
}


// ==================================================
// 7. CHỌN HƯỚNG THEO FLOOD FILL
// ==================================================

// Chọn ô kề có khoảng cách nhỏ nhất, ưu tiên ô chưa thăm
// nếu các ô có cùng khoảng cách. Chỉ chọn khi khoảng cách
// giảm; nếu không, chuyển sang tìm đường bằng BFS.
bool chooseFloodFillDirection(int &nextDir) {
    int bestDist = INF;
    int bestVisited = 2;
    bool found = false;

    for (int d = 0; d < 4; d++) {
        if (!canMove(x, y, d)) {
            continue;
        }

        int nx = x + dx[d];
        int ny = y + dy[d];
        int nd = dist[nx][ny];

        if (nd == INF) {
            continue;
        }

        int visitRank = visited[nx][ny] ? 1 : 0;

        if (!found ||
            nd < bestDist ||
            (nd == bestDist && visitRank < bestVisited)) {
            found = true;
            bestDist = nd;
            bestVisited = visitRank;
            nextDir = d;
        }
    }

    return found && bestDist < dist[x][y];
}


// ==================================================
// 8. BFS TÌM Ô CẦN KHÁM PHÁ
// ==================================================

// Tìm frontier gần nhất có thể đến qua các cạnh đã biết
// là thông, rồi trả về hướng đi đầu tiên trên đường đó.
bool findNearestFrontier(int &nextDir) {
    int firstDir[MAZE_SIZE][MAZE_SIZE];
    bool seen[MAZE_SIZE][MAZE_SIZE] = {};

    for (int i = 0; i < MAZE_SIZE; i++) {
        for (int j = 0; j < MAZE_SIZE; j++) {
            firstDir[i][j] = -1;
        }
    }

    std::queue<std::pair<int, int>> q;
    q.push({x, y});
    seen[x][y] = true;

    while (!q.empty()) {
        int cx = q.front().first;
        int cy = q.front().second;
        q.pop();

        // Tìm thấy ô còn cạnh chưa khám phá.
        if (hasUnknownEdge(cx, cy) &&
            !(cx == x && cy == y)) {
            nextDir = firstDir[cx][cy];
            return nextDir != -1;
        }

        for (int d = 0; d < 4; d++) {
            if (!canMove(cx, cy, d)) {
                continue;
            }

            int nx = cx + dx[d];
            int ny = cy + dy[d];

            if (seen[nx][ny]) {
                continue;
            }

            seen[nx][ny] = true;

            // Lưu hướng đầu tiên từ vị trí xuất phát.
            if (cx == x && cy == y) {
                firstDir[nx][ny] = d;
            } else {
                firstDir[nx][ny] = firstDir[cx][cy];
            }

            q.push({nx, ny});
        }
    }

    return false;
}


// ==================================================
// 9. BFS TÌM ĐƯỜNG ĐẾN ĐÍCH
// ==================================================

// Tìm bước đi đầu tiên đến tâm hoặc một tọa độ cụ thể.
// BFS tìm đường ngắn nhất trên bản đồ đã biết.
bool findNextStepToGoal(bool targetCenter,
                        int targetX,
                        int targetY,
                        int &nextDir) {
    int firstDir[MAZE_SIZE][MAZE_SIZE];
    bool seen[MAZE_SIZE][MAZE_SIZE] = {};

    for (int i = 0; i < MAZE_SIZE; i++) {
        for (int j = 0; j < MAZE_SIZE; j++) {
            firstDir[i][j] = -1;
        }
    }

    std::queue<std::pair<int, int>> q;
    q.push({x, y});
    seen[x][y] = true;

    while (!q.empty()) {
        int cx = q.front().first;
        int cy = q.front().second;
        q.pop();

        bool goal;

        if (targetCenter) {
            goal = isGoal(cx, cy);
        } else {
            goal = (cx == targetX && cy == targetY);
        }

        if (goal) {
            if (cx == x && cy == y) {
                return false;
            }

            nextDir = firstDir[cx][cy];
            return nextDir != -1;
        }

        for (int d = 0; d < 4; d++) {
            if (!canMove(cx, cy, d)) {
                continue;
            }

            int nx = cx + dx[d];
            int ny = cy + dy[d];

            if (seen[nx][ny]) {
                continue;
            }

            seen[nx][ny] = true;

            if (cx == x && cy == y) {
                firstDir[nx][ny] = d;
            } else {
                firstDir[nx][ny] = firstDir[cx][cy];
            }

            q.push({nx, ny});
        }
    }

    return false;
}


// ==================================================
// 10. GIAI ĐOẠN 1: KHÁM PHÁ MÊ CUNG
// ==================================================

// Cập nhật tường, tính lại Flood Fill và chọn hướng đi.
// Nếu không giảm được khoảng cách, dùng BFS tìm frontier.
// Dừng khi không tìm thấy frontier nào khác có thể tiếp cận.
bool exploreMaze() {
    while (true) {
        updateWalls();

        visited[x][y] = true;
        API::setText(x, y, "V");

        computeExploreFloodFill();

        int nextDir = -1;

        if (chooseFloodFillDirection(nextDir)) {
            if (!moveOneStep(nextDir)) {
                return false;
            }

            continue;
        }

        if (findNearestFrontier(nextDir)) {
            if (!moveOneStep(nextDir)) {
                return false;
            }

            continue;
        }

        return true;
    }
}


// ==================================================
// 11. GIAI ĐOẠN 2: ĐI ĐẾN TRUNG TÂM
// ==================================================

// Liên tục dùng BFS tìm bước đi đến một trong bốn ô tâm.
bool goToCenter() {
    while (!isGoal(x, y)) {
        int nextDir;

        if (!findNextStepToGoal(true, 0, 0, nextDir)) {
            API::setText(x, y, "NO PATH");
            return false;
        }

        if (!moveOneStep(nextDir)) {
            return false;
        }
    }

    return true;
}


// ==================================================
// 12. GIAI ĐOẠN 3: QUAY VỀ ĐIỂM XUẤT PHÁT
// ==================================================

// Dùng BFS tìm đường về ô (0,0) và di chuyển từng bước.
bool returnToStart() {
    while (x != 0 || y != 0) {
        int nextDir;

        if (!findNextStepToGoal(false, 0, 0, nextDir)) {
            API::setText(x, y, "NO PATH");
            return false;
        }

        if (!moveOneStep(nextDir)) {
            return false;
        }
    }

    return true;
}


// ==================================================
// 13. GIAI ĐOẠN 4: SPEED RUN
// ==================================================

// Từ điểm xuất phát, dùng BFS tìm đường đến tâm trên
// bản đồ đã khám phá, không chủ động khám phá thêm.
bool speedRun() {
    API::clearAllText();

    while (!isGoal(x, y)) {
        int nextDir;

        if (!findNextStepToGoal(true, 0, 0, nextDir)) {
            API::setText(x, y, "NO PATH");
            return false;
        }

        if (!moveOneStep(nextDir)) {
            return false;
        }
    }

    return true;
}


// ==================================================
// 14. CHƯƠNG TRÌNH CHÍNH
// ==================================================

// Thực hiện lần lượt: khởi tạo tường, khám phá,
// đến tâm, quay về Start và chạy nhanh đến tâm.
int main() {
    API::clearAllText();
    initBoundary();

    if (!exploreMaze()) {
        return 1;
    }

    if (!goToCenter()) {
        return 1;
    }

    if (!returnToStart()) {
        return 1;
    }

    if (!speedRun()) {
        return 1;
    }

    return 0;
}
