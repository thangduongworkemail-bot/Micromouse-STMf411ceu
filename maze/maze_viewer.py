"""
maze_viewer.py - Vẽ mê cung mà xe Micromouse đã vạch sau khi giải (solving_maze.c, Maze_SendMap).

Xe giải xong gửi bản đồ qua Bluetooth HC-05 (USART1, 9600 baud), rồi cứ nhận 1 ký tự là gửi lại. Chương trình
này mở cổng COM, gửi 'M' xin bản đồ, nhận khối #MAZE_BEGIN ... #MAZE_END (có tổng kiểm tra) rồi vẽ lên màn hình.
Mỗi bản đồ nhận được tự lưu vào maze/logs/ để vẽ lại sau.

Mở lúc nào cũng được, KHÔNG cần xe / COM8 kết nối trước. Trên cửa sổ:
    Cổng + Baud      chọn cổng (mặc định COM8, ⟳ = quét lại) và tốc độ (mặc định 9600 như HC-05)
    Kết nối lại      đóng (nếu đang mở) rồi mở lại cổng NGAY
    Ngắt kết nối     đóng cổng, thôi tự nối lại
    Xin bản đồ       gửi 'M' cho xe gửi lại bản đồ
    Tự nối lại       mất kết nối / chưa mở được thì tự thử lại mỗi 3s
    Mở file...       vẽ bản đồ đã lưu (logs/*.txt, log terminal, file mê cung mẫu)
Mọi việc với cổng COM chạy ở LUỒNG RIÊNG: mất Bluetooth (ghi / đóng cổng bị chờ) hay mở cổng khi xe tắt (Windows chờ
~20s) đều không làm treo cửa sổ.

Dòng lệnh:
    python maze_viewer.py                              mở cửa sổ, tự kết nối DEFAULT_PORT (COM8) - hoặc bấm đúp run_viewer.bat
    python maze_viewer.py --no-connect                 mở cửa sổ, chưa kết nối
    python maze_viewer.py --port COM5 --baud 115200    cổng / tốc độ khác (nếu đã đổi HC-05 + CubeMX sang 115200)
    python maze_viewer.py --file logs/maze_xxx.txt     vẽ lại từ file đã lưu / log terminal
    python maze_viewer.py --mazefile ../Simulator/mazefiles/classic/50.txt     vẽ mê cung mẫu (thử chương trình)
    python maze_viewer.py --list                       liệt kê cổng COM
    Thêm --ps hinh.ps để lưu hình (PostScript), --close-after N để tự đóng sau N giây.

Cần: Python 3 (tkinter có sẵn), pyserial (pip install pyserial) để nhận qua cổng COM.

Hình vẽ:
    tường đã biết = đỏ đậm | cạnh CHƯA biết = gạch xám mảnh | ô chưa khám phá = nền xám
    ô xuất phát = xanh lá, 4 ô đích = vàng, đường ngắn nhất qua các cạnh đã biết = xanh dương, tam giác = vị trí xe
"""
import argparse
import os
import queue
import re
import sys
import threading
import time
from collections import deque
import tkinter as tk
from tkinter import ttk, filedialog

N, E, S, W = 0, 1, 2, 3
DX = (0, 1, 0, -1)
DY = (1, 0, -1, 0)
STATE_NAMES = ["IDLE", "DECIDE_PATH", "MOVE_STRAIGHT", "TURN_LEFT", "TURN_RIGHT", "U_TURN", "FINISHED", "ERROR", "PAUSED (bị nhấc)", "ALIGN (căn tường)", "CHECKPOINT (lưu Flash)"]
PHASE_NAMES = ["SEARCH_GOAL (khám phá tới tâm)", "SEARCH_START (khám phá về)", "SPEED_RUN (chạy nhanh)", "DONE (xong)", "RETURN (quay về xuất phát)"]
STATS_NAMES = ["đoạn thẳng", "ô đã đi", "rẽ 90", "quay đầu", "kéo dài", "rút ngắn", "IR dừng sớm", "ô chạy nhanh"]
HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_PORT = "COM8"           # Cổng Bluetooth HC-05 (Outgoing) trên máy này
DEFAULT_BAUD = 9600             # Tốc độ HC-05 hiện tại
BAUDS = (9600, 19200, 38400, 57600, 115200)
RETRY_S = 3.0                   # Tự nối lại: thử mở lại sau chừng này giây
REQUEST_EVERY_S = 3.0           # Chưa có bản đồ: gửi 'M' xin lại mỗi chừng này giây


# ===================================================================================== dữ liệu + đọc
class MazeData:
    """Bản đồ n x n. cells[x][y] = byte như firmware: bit 0..3 tường N E S W, bit 4..7 cạnh N E S W đã biết."""

    def __init__(self, n):
        self.n = n
        self.cells = [[0] * n for _ in range(n)]
        self.robot = None          # (x, y, hướng)
        self.state = None          # (state, phase, rounds)
        self.stats = None          # list 8 số
        self.checksum_ok = None    # None = không có tổng kiểm tra (file chữ)
        self.source = ""

    def known(self, x, y, d):
        return bool(self.cells[x][y] & (0x10 << d))

    def wall(self, x, y, d):
        return bool(self.cells[x][y] & (1 << d))

    def set_edge(self, x, y, d, state):
        """state: True tường, False mở, None chưa biết. Ghi cả 2 phía"""
        for cx, cy, cd in ((x, y, d), (x + DX[d], y + DY[d], (d + 2) % 4)):
            if 0 <= cx < self.n and 0 <= cy < self.n:
                c = self.cells[cx][cy] & ~((1 << cd) | (0x10 << cd))
                if state is not None:
                    c |= 0x10 << cd
                    if state:
                        c |= 1 << cd
                self.cells[cx][cy] = c

    def goals(self):
        a, b = (self.n - 1) // 2, self.n // 2
        return {(a, a), (a, b), (b, a), (b, b)}

    def can_pass(self, x, y, d, optimistic):
        nx, ny = x + DX[d], y + DY[d]
        if not (0 <= nx < self.n and 0 <= ny < self.n):
            return False
        if self.known(x, y, d):
            return not self.wall(x, y, d)
        return optimistic

    def shortest_path(self, optimistic=False):
        """BFS (0,0) -> ô đích gần nhất. Trả về danh sách ô, hoặc None nếu không có đường"""
        prev = {(0, 0): None}
        q = deque([(0, 0)])
        goals = self.goals()
        while q:
            c = q.popleft()
            if c in goals:
                path = []
                while c is not None:
                    path.append(c)
                    c = prev[c]
                return path[::-1]
            for d in range(4):
                if self.can_pass(c[0], c[1], d, optimistic):
                    nc = (c[0] + DX[d], c[1] + DY[d])
                    if nc not in prev:
                        prev[nc] = c
                        q.append(nc)
        return None

    def known_ratio(self):
        tot = kn = 0
        for x in range(self.n):
            for y in range(self.n):
                for d in (N, E):
                    if 0 <= x + DX[d] < self.n and 0 <= y + DY[d] < self.n:
                        tot += 1
                        kn += self.known(x, y, d)
        return kn / tot if tot else 1.0


def parse_dump(lines, verbose=True):
    """Khối #MAZE_BEGIN ... #MAZE_END (Maze_SendMap). Lấy khối hoàn chỉnh CUỐI CÙNG. None nếu không có / hỏng"""
    block = None
    cur = None
    for raw in lines:
        line = raw.strip()
        if "#MAZE_BEGIN" in line:
            cur = [line[line.index("#MAZE_BEGIN"):]]
        elif cur is not None and line.startswith("#"):
            cur.append(line)
            if line.startswith("#MAZE_END"):
                block = cur
                cur = None
    if block is None:
        return None
    try:
        n = int(block[0].split()[1])
        m = MazeData(n)
        rows = 0
        total = 0
        for line in block[1:]:
            f = line.split()
            if f[0] == "#STATE" and len(f) >= 4:
                m.state = (int(f[1]), int(f[2]), int(f[3]))
            elif f[0] == "#ROBOT" and len(f) >= 4:
                m.robot = (int(f[1]), int(f[2]), int(f[3]) % 4)
            elif f[0] == "#STATS":
                m.stats = [int(v) for v in f[1:]]
            elif f[0] == "#ROW" and len(f) >= 3:
                y, hexs = int(f[1]), f[2]
                if len(hexs) != 2 * n or not 0 <= y < n:
                    raise ValueError(f"dòng #ROW {y} hỏng: {hexs!r}")
                for x in range(n):
                    v = int(hexs[2 * x:2 * x + 2], 16)
                    m.cells[x][y] = v
                    total += v
                rows += 1
            elif f[0] == "#MAZE_END":
                m.checksum_ok = (rows == n) and len(f) >= 2 and int(f[1]) == total % 65536
        if rows != n:
            m.checksum_ok = False
        return m
    except (ValueError, IndexError) as e:
        if verbose:
            print(f"[!] Khối bản đồ hỏng: {e}")
        return None


def parse_ascii(lines):
    """Bản đồ chữ: Maze_PrintMap ('+' cột, '---' tường, ' . ' / ':' chưa biết, ^ > v < = xe) hoặc file mazefiles
    ('o' cột). Lấy khối CUỐI CÙNG gồm 2n+1 dòng liên tiếp. None nếu không có"""
    rows = [l.rstrip("\r\n") for l in lines]
    best = None
    i = 0
    while i < len(rows):
        if rows[i][:1] in ("+", "o"):
            j = i
            while j < len(rows) and rows[j][:1] in ("+", "o", "|", ":"):
                j += 1
            blk = rows[i:j]
            n = (len(blk[0].rstrip()) - 1) // 4
            if n >= 2 and len(blk) >= 2 * n + 1:
                best = blk[:2 * n + 1]
            i = j
        else:
            i += 1
    if best is None:
        return None
    n = (len(best[0].rstrip()) - 1) // 4
    m = MazeData(n)
    ch = lambda r, c: best[r][c] if c < len(best[r]) else " "
    for r in range(n):
        y = n - 1 - r
        for x in range(n):
            for d, (rr, cc) in ((N, (2 * r, 4 * x + 2)), (S, (2 * r + 2, 4 * x + 2)),
                                (W, (2 * r + 1, 4 * x)), (E, (2 * r + 1, 4 * x + 4))):
                c = ch(rr, cc)
                state = None if c in ".:" else (c in "-|")
                if not m.known(x, y, d):
                    m.set_edge(x, y, d, state)
            rc = ch(2 * r + 1, 4 * x + 2)
            if rc in "^>v<":
                m.robot = (x, y, "^>v<".index(rc))
    return m


def parse_any(lines, source):
    m = parse_dump(lines) or parse_ascii(lines)
    if m is not None:
        m.source = source
    return m


def save_log(lines, _last=[None]):
    """Lưu khối bản đồ vào maze/logs/ (bỏ qua nếu trùng khối vừa lưu). Trả về đường dẫn hoặc None"""
    if lines == _last[0]:
        return None
    _last[0] = list(lines)
    d = os.path.join(HERE, "logs")
    os.makedirs(d, exist_ok=True)
    path = os.path.join(d, time.strftime("maze_%Y%m%d_%H%M%S.txt"))
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    return path


# ===================================================================================== cổng COM (luồng riêng)
def list_ports():
    """[(cổng, mô tả)]. Cổng Bluetooth: ghi Outgoing (dùng được) / Incoming (không dùng) theo MAC trong hwid"""
    try:
        from serial.tools import list_ports as lp
    except ImportError:
        return []
    out = []
    for p in sorted(lp.comports(), key=lambda p: int(re.sub(r"\D", "", p.device) or 0)):
        desc = p.description
        m = re.search(r"&([0-9A-F]{12})_", p.hwid or "", re.I)
        if m:
            desc = "Bluetooth Incoming (không dùng)" if m.group(1) == "000000000000" else f"Bluetooth Outgoing {m.group(1)}"
        out.append((p.device, desc))
    return out


def friendly_error(e):
    s = str(e)
    low = s.lower()
    if "semaphore" in low or "121" in s:
        return "HC-05 không trả lời (xe chưa bật / ngoài tầm / đang nối với máy khác)"
    if "access" in low or "denied" in low or "permission" in low:
        return "cổng đang bị chương trình khác dùng (đóng terminal / app khác)"
    if "filenotfound" in low or "cannot find" in low or "could not find" in low:
        return "không có cổng này"
    return s


class SerialWorker:
    """TOÀN BỘ việc mở / ghi / đọc / đóng cổng COM chạy ở 1 luồng riêng - cửa sổ không bao giờ phải chờ cổng.
    Luồng giao diện chỉ bỏ lệnh vào cmd_q (trả về ngay); luồng cổng báo lại qua evt_q:
        ("state", "disconnected" | "connecting" | "connected" | "error", thông báo)
        ("log", dòng chữ)          ("map", MazeData, các dòng khối)
    (Không kế thừa threading.Thread: Python 3.13 dùng tên nội bộ như _handle, trùng tên hàm là hỏng luồng)"""

    def __init__(self, evt_q):
        self.thread = threading.Thread(target=self.run, daemon=True, name="serial-worker")
        self.cmd_q = queue.Queue()
        self.evt = evt_q
        self.ser = None
        self.port = DEFAULT_PORT
        self.baud = DEFAULT_BAUD
        self.want = False          # Người dùng muốn đang kết nối
        self.auto = True           # Tự nối lại
        self.need_map = False      # Chưa có bản đồ -> gửi 'M' định kỳ
        self.next_try = 0.0
        self.last_req = 0.0
        self.buf = b""
        self.block = []

    # ----- gọi từ luồng giao diện (không chặn)
    def start(self):
        self.thread.start()

    def connect(self, port, baud):
        self.cmd_q.put(("connect", port, baud))

    def disconnect(self):
        self.cmd_q.put(("disconnect",))

    def request_map(self):
        self.cmd_q.put(("request",))

    def set_auto(self, on):
        self.cmd_q.put(("auto", bool(on)))

    def quit(self):
        self.cmd_q.put(("quit",))

    # ----- luồng cổng
    def _emit(self, *ev):
        self.evt.put(ev)

    def run(self):
        while True:
            try:
                try:
                    cmd = self.cmd_q.get(timeout=0.02 if self.ser is not None else 0.2)
                except queue.Empty:
                    cmd = None
                if cmd is not None and self._do_cmd(cmd) == "quit":
                    break
                if self.ser is None and self.want and time.time() >= self.next_try:
                    self._open()
                if self.ser is not None:
                    self._io()
            except Exception as e:      # noqa: BLE001 - luồng cổng không bao giờ được chết
                self._lost(f"Lỗi cổng: {friendly_error(e)}")
        self._close()

    def _do_cmd(self, cmd):
        kind = cmd[0]
        if kind == "connect":
            self.port, self.baud = cmd[1], cmd[2]
            self.want = True
            self.need_map = True
            if self.ser is not None:            # Kết nối lại ngay: đóng rồi mở lại
                self._emit("log", f"Kết nối lại {self.port}...")
                self._close()
            self.next_try = 0.0
        elif kind == "disconnect":
            self.want = False
            self._close()
            self._emit("state", "disconnected", "Đã ngắt kết nối")
        elif kind == "request":
            if self.ser is None:
                self._emit("log", "Chưa kết nối - bấm 'Kết nối' trước")
            else:
                self.need_map = True
                self.last_req = 0.0             # Gửi ngay ở _io
        elif kind == "auto":
            self.auto = cmd[1]
        elif kind == "quit":
            return "quit"
        return None

    def _open(self):
        import serial
        self._emit("state", "connecting", f"Đang mở {self.port} ({self.baud} baud)... bật xe để HC-05 kết nối")
        try:
            # write_timeout: ghi khi mất Bluetooth không chờ vô hạn. Mở cổng Bluetooth khi xe tắt: Windows chờ ~20s
            s = serial.serial_for_url(self.port, baudrate=self.baud, timeout=0.05, write_timeout=1.0)
        except Exception as e:          # noqa: BLE001
            self._fail(f"Không mở được {self.port}: {friendly_error(e)}")
            return
        self.ser = s
        self.buf = b""
        self.block = []
        self.last_req = 0.0
        self._emit("state", "connected", f"Đã kết nối {self.port} ({self.baud} baud)")
        self._emit("log", f"Đã kết nối {self.port} - xin bản đồ (xe phải giải xong mê cung)")

    def _fail(self, msg):
        """Mở không được / mất kết nối: tự thử lại (nếu bật) hoặc dừng"""
        if self.want and self.auto:
            self.next_try = time.time() + RETRY_S
            self._emit("state", "error", f"{msg} - tự thử lại sau {RETRY_S:.0f}s")
        else:
            self.want = False
            self._emit("state", "error", msg)
        self._emit("log", msg)

    def _lost(self, msg):
        self._close()
        self._fail(msg)

    def _close(self):
        s, self.ser = self.ser, None
        if s is not None:
            try:
                s.close()                       # Có thể chờ khi Bluetooth đã rớt - chỉ luồng này chờ
            except Exception:                   # noqa: BLE001
                pass

    def _io(self):
        import serial
        s = self.ser
        if self.need_map and time.time() - self.last_req >= REQUEST_EVERY_S:
            self.last_req = time.time()
            try:
                s.write(b"M")
            except serial.SerialTimeoutException:
                self._lost("Gửi không được trong 1s (mất Bluetooth?)")
                return
        data = s.read(4096)
        if not data:
            return
        self.buf += data
        while b"\n" in self.buf:
            raw, self.buf = self.buf.split(b"\n", 1)
            line = raw.decode("ascii", errors="replace").rstrip("\r")
            if not line.startswith("#"):
                if line.strip():
                    self._emit("log", f"xe> {line}")
                continue
            if line.startswith("#MAZE_BEGIN"):
                self.block = []
            self.block.append(line)
            if line.startswith("#MAZE_END"):
                m = parse_dump(self.block, verbose=False)
                if m is not None and m.checksum_ok:
                    m.source = f"{self.port} lúc {time.strftime('%H:%M:%S')}"
                    self.need_map = False
                    self._emit("map", m, list(self.block))
                else:
                    self._emit("log", "[!] Bản đồ nhận được bị lỗi (mất ký tự) -> xin lại")
                    self.need_map = True
                    self.last_req = 0.0
                self.block = []


# ===================================================================================== vẽ
def make_root():
    """Cửa sổ Tk nét trên màn hình phóng to (Windows DPI scaling): báo Windows chương trình tự lo DPI"""
    if sys.platform == "win32":
        try:
            import ctypes
            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except (AttributeError, OSError):
            pass
    return tk.Tk()


class Viewer:
    """Khung vẽ mê cung + bảng thông tin (đặt trong parent)"""
    CELL = 38
    MARGIN = 34

    def __init__(self, parent):
        scale = max(1.0, parent.winfo_fpixels("1i") / 96.0)     # 96 dpi = 100%
        self.CELL = int(Viewer.CELL * scale)
        self.MARGIN = int(Viewer.MARGIN * scale)
        self.data = None
        self.info = tk.StringVar(value="Chưa có bản đồ.\n\nKết nối xe (COM) rồi bấm 'Xin bản đồ',\nhoặc 'Mở file...'.")
        self.canvas = tk.Canvas(parent, width=16 * self.CELL + 2 * self.MARGIN,
                                height=16 * self.CELL + 2 * self.MARGIN, bg="white", highlightthickness=0)
        self.canvas.pack(side=tk.LEFT, padx=6, pady=6)
        ttk.Label(parent, textvariable=self.info, justify=tk.LEFT, font=("Consolas", 10)).pack(
            side=tk.LEFT, anchor=tk.N, padx=8, pady=10)

    def px(self, x, y):
        """Góc Tây-Nam ô (x,y) -> tọa độ canvas (Bắc ở trên)"""
        n = self.data.n
        return self.MARGIN + x * self.CELL, self.MARGIN + (n - y) * self.CELL

    def center(self, x, y):
        X, Y = self.px(x, y)
        return X + self.CELL / 2, Y - self.CELL / 2

    def draw(self, m):
        self.data = m
        n, cs, cv = m.n, self.CELL, self.canvas
        cv.delete("all")
        cv.config(width=n * cs + 2 * self.MARGIN, height=n * cs + 2 * self.MARGIN)
        goals = m.goals()
        # 1. Nền ô: chưa khám phá (4 cạnh chưa biết) xám, xuất phát xanh lá, đích vàng
        for x in range(n):
            for y in range(n):
                X, Y = self.px(x, y)
                fill = None
                if all(not m.known(x, y, d) for d in range(4)):
                    fill = "#e4e4e4"
                if (x, y) in goals:
                    fill = "#fff0a0"
                if (x, y) == (0, 0):
                    fill = "#c6f2c0"
                if fill:
                    cv.create_rectangle(X, Y - cs, X + cs, Y, fill=fill, outline="")
        # 2. Đường ngắn nhất qua cạnh đã biết (nếu chưa có thì vẽ đường lạc quan, nét đứt)
        path = m.shortest_path(optimistic=False)
        opt_path = m.shortest_path(optimistic=True)
        show, dash = (path, None) if path else (opt_path, (6, 4))
        if show and len(show) > 1:
            pts = [c for cell in show for c in self.center(*cell)]
            cv.create_line(*pts, fill="#1f6fd1", width=4, dash=dash, arrow=tk.LAST, arrowshape=(12, 14, 5),
                           capstyle=tk.ROUND, joinstyle=tk.ROUND)
        # 3. Cạnh: tường đã biết đỏ đậm, chưa biết gạch xám (mỗi cạnh vẽ 1 lần: N + E của mọi ô, thêm biên S / W)
        for x in range(n):
            for y in range(n):
                X, Y = self.px(x, y)
                edges = [(N, (X, Y - cs, X + cs, Y - cs)), (E, (X + cs, Y - cs, X + cs, Y))]
                if y == 0:
                    edges.append((S, (X, Y, X + cs, Y)))
                if x == 0:
                    edges.append((W, (X, Y - cs, X, Y)))
                for d, xy in edges:
                    if not m.known(x, y, d):
                        cv.create_line(*xy, fill="#a8a8a8", width=1, dash=(3, 3))
                    elif m.wall(x, y, d):
                        cv.create_line(*xy, fill="#b00000", width=4, capstyle=tk.PROJECTING)
        # 4. Cột tường
        for i in range(n + 1):
            for j in range(n + 1):
                X, Y = self.MARGIN + i * cs, self.MARGIN + j * cs
                cv.create_rectangle(X - 2, Y - 2, X + 2, Y + 2, fill="#202020", outline="")
        # 5. Nhãn tọa độ
        for i in range(n):
            X, _ = self.center(i, 0)
            cv.create_text(X, self.MARGIN + n * cs + 14, text=str(i), font=("Arial", 8), fill="#555")
            _, Y = self.center(0, i)
            cv.create_text(self.MARGIN - 14, Y, text=str(i), font=("Arial", 8), fill="#555")
        cv.create_text(self.MARGIN + n * cs / 2, 12, text="Bắc", font=("Arial", 9, "bold"), fill="#555")
        # 6. Xe: tam giác chỉ hướng
        if m.robot:
            x, y, h = m.robot
            cx, cy = self.center(x, y)
            r = cs * 0.32
            tip = (cx + DX[h] * r, cy - DY[h] * r)
            lx, ly = DX[(h + 3) % 4], DY[(h + 3) % 4]
            back = (cx - DX[h] * r * 0.7, cy + DY[h] * r * 0.7)
            pts = [tip[0], tip[1], back[0] + lx * r * 0.7, back[1] - ly * r * 0.7,
                   back[0] - lx * r * 0.7, back[1] + ly * r * 0.7]
            cv.create_polygon(*pts, fill="#ff8c00", outline="#7a3e00", width=1.5)
        self.info.set(self.describe(m, path, opt_path))

    @staticmethod
    def describe(m, path, opt_path):
        L = [f"Nguồn: {m.source}"]
        if m.checksum_ok is True:
            L.append("Tổng kiểm tra: ĐÚNG")
        elif m.checksum_ok is False:
            L.append("Tổng kiểm tra: SAI (mất ký tự?)")
        L.append(f"Mê cung {m.n} x {m.n}, đã biết {m.known_ratio() * 100:.0f}% cạnh")
        if m.state:
            s, p, rounds = m.state
            L.append(f"Trạng thái: {STATE_NAMES[s] if 0 <= s < len(STATE_NAMES) else s}")
            L.append(f"Giai đoạn : {PHASE_NAMES[p] if 0 <= p < len(PHASE_NAMES) else p}")
            L.append(f"Số vòng khám phá: {rounds}")
        if m.robot:
            L.append(f"Xe ở ô ({m.robot[0]}, {m.robot[1]}) hướng {['Bắc', 'Đông', 'Nam', 'Tây'][m.robot[2]]}")
        L.append("")
        kp = (len(path) - 1) if path else None
        op = (len(opt_path) - 1) if opt_path else None
        L.append(f"Đường ngắn nhất (cạnh đã biết): {kp if kp is not None else 'CHƯA CÓ'} ô")
        L.append(f"Đường lạc quan (chưa biết = mở): {op if op is not None else '-'} ô")
        if kp is not None and op is not None:
            L.append("=> đường đã biết CHẮC CHẮN ngắn nhất" if kp == op else "=> có thể còn đường ngắn hơn")
        if m.stats:
            L.append("")
            L.append("Thống kê xe:")
            for name, v in zip(STATS_NAMES, m.stats):
                L.append(f"  {name:14s} {v}")
        L.append("")
        L.append("Đỏ = tường đã biết")
        L.append("Gạch xám = cạnh chưa biết")
        L.append("Xanh dương = đường ngắn nhất")
        L.append("Cam = xe")
        return "\n".join(L)

    def save_ps(self, path):
        self.canvas.update()
        self.canvas.postscript(file=path, colormode="color")


# ===================================================================================== cửa sổ chính
class App:
    STATE_COLORS = {"disconnected": "#9a9a9a", "connecting": "#e69500", "connected": "#1a9e2a", "error": "#c62828"}

    def __init__(self, root, port, baud, auto_connect):
        self.root = root
        root.title("Micromouse - bản đồ mê cung")
        self.evt_q = queue.Queue()
        self.worker = SerialWorker(self.evt_q)
        self.worker.start()

        # Thanh 1: cổng + nút kết nối
        bar = ttk.Frame(root)
        bar.pack(side=tk.TOP, fill=tk.X, padx=6, pady=(6, 2))
        ttk.Label(bar, text="Cổng").pack(side=tk.LEFT)
        self.port_var = tk.StringVar(value=port)
        self.port_box = ttk.Combobox(bar, textvariable=self.port_var, width=34)
        self.port_box.pack(side=tk.LEFT, padx=(4, 0))
        ttk.Button(bar, text="⟳", width=3, command=self.refresh_ports).pack(side=tk.LEFT, padx=(2, 8))
        ttk.Label(bar, text="Baud").pack(side=tk.LEFT)
        self.baud_var = tk.StringVar(value=str(baud))
        ttk.Combobox(bar, textvariable=self.baud_var, values=[str(b) for b in BAUDS], width=8).pack(
            side=tk.LEFT, padx=(4, 8))
        self.btn_connect = ttk.Button(bar, text="Kết nối", command=self.on_connect)
        self.btn_connect.pack(side=tk.LEFT, padx=2)
        ttk.Button(bar, text="Ngắt kết nối", command=self.on_disconnect).pack(side=tk.LEFT, padx=2)
        ttk.Button(bar, text="Xin bản đồ", command=self.worker.request_map).pack(side=tk.LEFT, padx=2)
        self.auto_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(bar, text="Tự nối lại", variable=self.auto_var,
                        command=lambda: self.worker.set_auto(self.auto_var.get())).pack(side=tk.LEFT, padx=8)
        ttk.Button(bar, text="Lưu hình (.ps)", command=self.on_save_ps).pack(side=tk.RIGHT, padx=2)
        ttk.Button(bar, text="Mở file...", command=self.on_open_file).pack(side=tk.RIGHT, padx=2)

        # Thanh 2: đèn + trạng thái
        st = ttk.Frame(root)
        st.pack(side=tk.TOP, fill=tk.X, padx=8, pady=(0, 2))
        self.dot = tk.Label(st, text="●", font=("Arial", 14), fg=self.STATE_COLORS["disconnected"])
        self.dot.pack(side=tk.LEFT)
        self.status = tk.StringVar(value="Chưa kết nối")
        ttk.Label(st, textvariable=self.status).pack(side=tk.LEFT, padx=4)

        # Giữa: mê cung + thông tin
        body = ttk.Frame(root)
        body.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        self.view = Viewer(body)

        # Dưới: nhật ký
        logf = ttk.Frame(root)
        logf.pack(side=tk.TOP, fill=tk.X, padx=6, pady=(0, 6))
        self.log = tk.Text(logf, height=6, font=("Consolas", 9), state=tk.DISABLED, bg="#fafafa")
        sb = ttk.Scrollbar(logf, command=self.log.yview)
        self.log.config(yscrollcommand=sb.set)
        self.log.pack(side=tk.LEFT, fill=tk.X, expand=True)
        sb.pack(side=tk.LEFT, fill=tk.Y)

        self.refresh_ports()
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        root.after(50, self.pump)
        if auto_connect:
            self.on_connect()

    # ----- tiện ích
    def add_log(self, text):
        self.log.config(state=tk.NORMAL)
        self.log.insert(tk.END, time.strftime("%H:%M:%S ") + text + "\n")
        lines = int(self.log.index("end-1c").split(".")[0])
        if lines > 500:
            self.log.delete("1.0", f"{lines - 500}.0")
        self.log.see(tk.END)
        self.log.config(state=tk.DISABLED)

    def set_state(self, state, msg):
        self.dot.config(fg=self.STATE_COLORS.get(state, "#9a9a9a"))
        self.status.set(msg)
        self.btn_connect.config(text="Kết nối lại" if state in ("connected", "connecting", "error") else "Kết nối")

    def selected_port(self):
        return self.port_var.get().strip().split()[0] if self.port_var.get().strip() else DEFAULT_PORT

    def refresh_ports(self):
        cur = self.selected_port()
        ports = list_ports()
        self.port_box.config(values=[f"{p}  -  {d}" for p, d in ports])
        for p, d in ports:
            if p == cur:
                self.port_var.set(f"{p}  -  {d}")
                break

    # ----- nút
    def on_connect(self):
        try:
            baud = int(self.baud_var.get())
        except ValueError:
            baud = DEFAULT_BAUD
            self.baud_var.set(str(baud))
        port = self.selected_port()
        self.worker.set_auto(self.auto_var.get())
        self.worker.connect(port, baud)
        self.set_state("connecting", f"Đang kết nối {port} ({baud} baud)...")

    def on_disconnect(self):
        self.worker.disconnect()
        self.set_state("disconnected", "Đang ngắt kết nối...")

    def on_open_file(self):
        path = filedialog.askopenfilename(title="Mở bản đồ", initialdir=os.path.join(HERE, "logs")
                                          if os.path.isdir(os.path.join(HERE, "logs")) else HERE,
                                          filetypes=[("Bản đồ / log", "*.txt *.log"), ("Tất cả", "*.*")])
        if path:
            self.load_file(path)

    def load_file(self, path):
        try:
            with open(path, encoding="utf-8", errors="replace") as f:
                m = parse_any(f.readlines(), os.path.basename(path))
        except OSError as e:
            self.add_log(f"Không đọc được {path}: {e}")
            return False
        if m is None:
            self.add_log(f"Không tìm thấy bản đồ trong {path}")
            return False
        self.view.draw(m)
        self.add_log(f"Đã vẽ bản đồ từ file {path}")
        return True

    def on_save_ps(self, path=None):
        if self.view.data is None:
            self.add_log("Chưa có bản đồ để lưu")
            return
        path = path or os.path.join(HERE, time.strftime("maze_%Y%m%d_%H%M%S.ps"))
        self.view.save_ps(path)
        self.add_log(f"Đã lưu hình: {path}")

    def on_close(self):
        self.worker.quit()          # Luồng cổng tự đóng cổng; không chờ (có thể đang kẹt chờ Bluetooth)
        self.root.destroy()

    # ----- nhận sự kiện từ luồng cổng (không bao giờ chờ)
    def pump(self):
        try:
            while True:
                ev = self.evt_q.get_nowait()
                if ev[0] == "state":
                    self.set_state(ev[1], ev[2])
                elif ev[0] == "log":
                    self.add_log(ev[1])
                elif ev[0] == "map":
                    m, lines = ev[1], ev[2]
                    self.view.draw(m)
                    path = save_log(lines)
                    self.add_log("Đã nhận bản đồ" + (f", lưu: {path}" if path else " (trùng bản trước, không lưu lại)"))
                    self.status.set(f"Đã nhận bản đồ lúc {time.strftime('%H:%M:%S')} - "
                                    "bấm 'Xin bản đồ' để cập nhật")
        except queue.Empty:
            pass
        self.root.after(50, self.pump)


# ===================================================================================== main
def main():
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except AttributeError:
        pass
    ap = argparse.ArgumentParser(description="Vẽ mê cung xe Micromouse đã vạch (Maze_SendMap)")
    ap.add_argument("--port", help=f"cổng COM của HC-05 (mặc định {DEFAULT_PORT}) hoặc URL pyserial (socket://host:port)")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--no-connect", action="store_true", help="mở cửa sổ nhưng chưa kết nối")
    ap.add_argument("--file", help="file log / dữ liệu đã lưu")
    ap.add_argument("--mazefile", help="file mê cung mẫu (micromouseonline/mazefiles)")
    ap.add_argument("--list", action="store_true", help="liệt kê cổng COM")
    ap.add_argument("--ps", help="lưu hình ra file PostScript (khi có bản đồ)")
    ap.add_argument("--close-after", type=float, default=0, help="tự đóng sau N giây")
    a = ap.parse_args()

    if a.list:
        for p, d in list_ports():
            print(f"{p:8s} {d}")
        return 0

    root = make_root()
    path = a.file or a.mazefile
    # Có file thì chỉ vẽ file (trừ khi chỉ định --port); không thì tự kết nối cổng mặc định - KHÔNG chặn cửa sổ
    auto = (not a.no_connect) and (a.port is not None or path is None)
    app = App(root, a.port or DEFAULT_PORT, a.baud, auto)
    if path and app.load_file(path) and a.ps:
        app.on_save_ps(a.ps)
    if a.ps and not path:
        def save_when_ready():
            if app.view.data is not None:
                app.on_save_ps(a.ps)
            else:
                root.after(200, save_when_ready)
        root.after(200, save_when_ready)
    if a.close_after > 0:
        root.after(int(a.close_after * 1000), app.on_close)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
