import pygame
import math
import json
import os
import sys
from heapq import heappush, heappop
from PIL import Image

def resource_path(relative_path):
    if hasattr(sys, '_MEIPASS'):
        return os.path.join(sys._MEIPASS, relative_path)
    return os.path.join(os.path.abspath("."), relative_path)

script_dir = os.path.dirname(os.path.abspath(__file__))
os.chdir(script_dir)
print(f"작업 디렉토리가 {script_dir}로 변경되었습니다.")

# PyGame 초기화 및 전역 변수 선언
pygame.init()
BASE_WIDTH, BASE_HEIGHT = 1280, 720
GRID_SIZE = 20
screen = pygame.display.set_mode((BASE_WIDTH, BASE_HEIGHT), pygame.RESIZABLE)
pygame.display.set_caption("6륜 UGV 시뮬레이션")
clock = pygame.time.Clock()

font = pygame.font.SysFont("malgungothic", 16)
modal_font = pygame.font.SysFont("malgungothic", 20)

try:
    ugv_img_orig = pygame.image.load(resource_path("ugv.png")).convert_alpha()
    ugv_img = pygame.transform.scale(ugv_img_orig, (80, 50))
    print("UGV PNG 이미지가 성공적으로 로드되었습니다.")
except Exception as e:
    ugv_img_orig = None
    ugv_img = None
    print(f"UGV 이미지 로드 실패: {e}")

try:
    terrain_img_orig = pygame.image.load(resource_path("terrain.jpg")).convert()
    print("지형도 이미지가 성공적으로 로드되었습니다.")
except Exception as e:
    terrain_img_orig = None
    print(f"지형도 이미지 로드 실패: {e}")

try:
    rock_img_orig = pygame.image.load(resource_path("rock.png")).convert_alpha()
    rock_img = pygame.transform.scale(rock_img_orig, (50, 50))
    print("바위 이미지가 성공적으로 로드되었습니다.")
except Exception as e:
    rock_img_orig = None
    rock_img = None
    print(f"바위 이미지 로드 실패: {e}")

# 시뮬레이션 전역 변수
waypoints = []
obstacles = []
scenario_events = []
full_path = []
ugv = None
path_index = 0
input_mode = True
max_speed = 20.0
max_lat_acc = 1.0
max_acc = 1.0
fov_angle = 60.0
fov_range = 30.0
min_turn_radius = 5.0
cog_height = 0.8
use_safe_speed = True
lat0 = 37.0
lon0 = 127.0
sim_time = 0.0
simulation_ended = False
overturn_modal = False
path_error_modal = False
debug_modal = False
update_modal = False
update_modal_start_time = 0
update_modal_duration = 2000
modal_close_button_rect = None
temp_max_speed = max_speed
temp_max_lat_acc = max_lat_acc
temp_max_acc = max_acc
temp_fov_angle = fov_angle
temp_fov_range = fov_range
temp_min_turn_radius = min_turn_radius
temp_cog_height = cog_height
temp_use_safe_speed = use_safe_speed
modal_text = [
    "1) 마우스 Right-클릭으로 맵 상에 장애물들을 생성하세요",
    "2) 오른쪽 패널에서 UGV 관련 파라미터를 조정하세요",
    "3) 마우스 Left-클릭으로 경로점들을 생성하세요",
    "4) 엔터 키를 누르면 시뮬레이션이 시작됩니다",
    "5) ESC 키를 누르면 시뮬레이션이 초기화됩니다",
    "6) scenario.json으로 시나리오 설정 가능"
]
modal_start_time = pygame.time.get_ticks()
modal_duration = 10000
show_modal = True

# PID 제어 변수
pid_error_integral = 0.0
pid_prev_error = 0.0
pid_kp = 0.5  # 비례 게인
pid_ki = 0.01  # 적분 게인
pid_kd = 0.1  # 미분 게인

# UGV 클래스
class UGV:
    def __init__(self, x, y, theta, max_speed_kmh=20.0, max_lat_acc=1.0, max_acc=1.0, fov_angle=60.0, fov_range=30.0, min_turn_radius=5.0, cog_height=0.8):
        self.x = x
        self.y = y
        self.theta = theta
        self.v = 0
        self.target_speed = max_speed_kmh / 3.6
        self.omega = 0
        self.size = 40
        self.max_speed = max_speed_kmh / 3.6
        self.max_lat_acc = max_lat_acc
        self.max_acc = max_acc
        self.fov_angle = math.radians(fov_angle)
        self.fov_range = fov_range
        self.min_turn_radius = min_turn_radius
        self.cog_height = cog_height
        self.vehicle_width = 1.8  # 고정
        self.is_overturned = False

    def reset(self):
        self.v = 0
        self.target_speed = self.max_speed
        self.omega = 0
        self.is_overturned = False

    def update(self, dt, scale_factor=1.0, use_safe_speed=True, use_omega_limit=True, sim_time=0.0, scenario_events=None, path_index=0):
        # 위치 유효성 검사
        if not (math.isfinite(self.x) and math.isfinite(self.y) and math.isfinite(self.theta)):
            print(f"위치 비정상: x={self.x}, y={self.y}, theta={self.theta}")
            self.x, self.y, self.theta = BASE_WIDTH // 2, BASE_HEIGHT // 2, 0
            self.v = 0

        # JSON 이벤트 처리
        if scenario_events:
            for event in scenario_events:
                if abs(sim_time - event["time"]) < dt:
                    print(f"시나리오 이벤트 적용: 시간={sim_time:.1f}s")
                    if "use_safe_speed" in event:
                        use_safe_speed = event["use_safe_speed"]
                        print(f"  use_safe_speed={use_safe_speed}")
                    if "use_omega_limit" in event:
                        use_omega_limit = event["use_omega_limit"]
                        print(f"  use_omega_limit={use_omega_limit}")
                    if "target_speed_kmh" in event:
                        self.target_speed = event["target_speed_kmh"] / 3.6
                        self.max_speed = max(20.0 / 3.6, min(40.0 / 3.6, self.target_speed))
                        self.target_speed = min(self.target_speed, self.max_speed)
                        print(f"  target_speed={event['target_speed_kmh']:.1f} km/h, max_speed={self.max_speed * 3.6:.1f} km/h")
                    if "min_turn_radius" in event:
                        self.min_turn_radius = event["min_turn_radius"]
                        print(f"  min_turn_radius={self.min_turn_radius:.1f} m")
                    if "cog_height" in event or "vehicle_width" in event:
                        print("  cog_height, vehicle_width는 고정값 사용: cog_height=0.8 m, vehicle_width=1.8 m")

        # 속도 제한 강화
        self.max_speed = min(self.max_speed, 40.0 / 3.6)
        self.target_speed = min(self.target_speed, self.max_speed)
        if self.v < self.target_speed:
            self.v = min(self.v + self.max_acc * dt, self.target_speed)
        elif self.v > self.target_speed:
            self.v = max(self.v - self.max_acc * dt, self.target_speed)
        self.v = min(self.v, self.max_speed)

        # 전복 체크 로직
        lateral_acc = 0.0
        current_radius = float('inf')
        self.cog_height = 0.8  # 고정
        self.vehicle_width = 1.8  # 고정
        overturn_threshold = 9.81 * (self.vehicle_width / (2 * self.cog_height)) * 0.8  # 약 4.41 m/s²
        if self.v > 0 and abs(self.omega) > 1e-6:
            current_radius = self.v / abs(self.omega)
            lateral_acc = (self.v ** 2) / current_radius
            if lateral_acc > overturn_threshold:
                self.is_overturned = True
                print(f"차량 전복 발생! 시간={sim_time:.1f}s, 위치=({self.x:.1f}, {self.y:.1f}), "
                      f"v={self.v * 3.6:.1f} km/h, omega={self.omega:.3f} rad/s, "
                      f"current_radius={current_radius:.2f} m, lateral_acc={lateral_acc:.2f} m/s², "
                      f"overturn_threshold={overturn_threshold:.2f} m/s²")
                return
        else:
            print(f"직선 주행 또는 omega 너무 작음: omega={self.omega:.3f} rad/s, path_index={path_index}")

        # 디버깅 로그
        print(f"시간={sim_time:.1f}s, x={self.x:.1f}, y={self.y:.1f}, v={self.v * 3.6:.1f} km/h, "
              f"target_speed={self.target_speed * 3.6:.1f} km/h, max_speed={self.max_speed * 3.6:.1f} km/h, "
              f"omega={self.omega:.3f} rad/s, current_radius={current_radius:.2f} m, "
              f"lateral_acc={lateral_acc:.2f} m/s², overturn_threshold={overturn_threshold:.2f} m/s²")

        # 안전 속도
        if use_safe_speed and self.v > 0 and abs(self.omega) > 1e-6:
            v_safe = math.sqrt(9.81 * (self.vehicle_width / (2 * self.cog_height)) * self.min_turn_radius)
            self.target_speed = min(self.target_speed, v_safe)
            print(f"v_safe={v_safe:.2f} m/s, target_speed={self.target_speed:.2f} m/s")

        # 오메가 제한
        if use_omega_limit and self.v > 0:
            max_omega = self.v / self.min_turn_radius
            prev_omega = self.omega
            self.omega = max(min(self.omega, max_omega), -max_omega)
            print(f"max_omega={max_omega:.3f} rad/s, prev_omega={prev_omega:.3f} rad/s, applied_omega={self.omega:.3f} rad/s")

        # 위치 업데이트
        dx = self.v * math.cos(self.theta) * dt * (GRID_SIZE / 10) * scale_factor
        dy = self.v * math.sin(self.theta) * dt * (GRID_SIZE / 10) * scale_factor
        self.x += dx
        self.y += dy
        self.theta += self.omega * dt

        # 위치 재검사
        if not (math.isfinite(self.x) and math.isfinite(self.y)):
            print(f"위치 업데이트 후 비정상: x={self.x}, y={self.y}")
            self.x, self.y = BASE_WIDTH // 2, BASE_HEIGHT // 2

    def draw(self, screen, scale_factor=1.0):
        print(f"UGV 렌더링: x={self.x:.1f}, y={self.y:.1f}")
        if ugv_img:
            scaled_size = (int(80 * scale_factor), int(50 * scale_factor))
            scaled_img = pygame.transform.scale(ugv_img_orig, scaled_size)
            rotated_img = pygame.transform.rotate(scaled_img, -math.degrees(self.theta))
            rect = rotated_img.get_rect(center=(int(self.x), int(self.y)))
            screen.blit(rotated_img, rect)
        else:
            size = int(self.size * scale_factor)
            points = [
                (self.x + size * math.cos(self.theta), self.y + size * math.sin(self.theta)),
                (self.x + size * math.cos(self.theta + 2.094), self.y + size * math.sin(self.theta + 2.094)),
                (self.x + size * math.cos(self.theta + 4.189), self.y + size * math.sin(self.theta + 4.189))
            ]
            pygame.draw.polygon(screen, (100, 100, 100), points)
            for i in range(6):
                wheel_x = self.x + size * 0.8 * math.cos(self.theta + i * 1.047 - 0.524)
                wheel_y = self.y + size * 0.8 * math.sin(self.theta + i * 1.047 - 0.524)
                pygame.draw.circle(screen, (50, 50, 50), (int(wheel_x), int(wheel_y)), int(8 * scale_factor))

        fov_surface = pygame.Surface((screen.get_width(), screen.get_height()), pygame.SRCALPHA)
        for direction in [1, -1]:
            start_angle = self.theta - direction * self.fov_angle / 2
            end_angle = self.theta + direction * self.fov_angle / 2
            points = [(self.x, self.y)]
            step = -1 if start_angle > end_angle else 1
            for angle in range(int(math.degrees(start_angle)), int(math.degrees(end_angle)) + step, step):
                rad = math.radians(angle)
                px = self.x + (self.fov_range * (GRID_SIZE / 10) * scale_factor) * math.cos(rad)
                py = self.y + (self.fov_range * (GRID_SIZE / 10) * scale_factor) * math.sin(rad)
                points.append((px, py))
            if len(points) > 2:
                pygame.draw.polygon(fov_surface, (0, 255, 0, 50), points)
        screen.blit(fov_surface, (0, 0))

# A* 알고리즘
def heuristic(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])

def a_star(start, goal, obstacles, grid_size, map_width, map_height, fov_range=30.0):
    grid_width = map_width // grid_size
    grid_height = map_height // grid_size
    open_list = []
    heappush(open_list, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_list:
        current = heappop(open_list)[1]
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]

        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]:
            neighbor = (current[0] + dx, current[1] + dy)
            if not (0 <= neighbor[0] < grid_width and 0 <= neighbor[1] < grid_height):
                continue

            nx, ny = neighbor[0] * grid_size + grid_size // 2, neighbor[1] * grid_size + grid_size // 2
            avoid_margin = fov_range * (grid_size / 10) * 0.8
            if any(
                math.hypot(nx - (obs["x"] + obs["width"] / 2), ny - (obs["y"] + obs["height"] / 2)) < avoid_margin
                for obs in obstacles
            ):
                continue

            tentative_g_score = g_score[current] + heuristic(current, neighbor)
            if tentative_g_score < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal)
                heappush(open_list, (f_score[neighbor], neighbor))
    return None

# 경로 스무딩 함수
def smooth_path(path, weight_data=0.5, weight_smooth=0.2, tolerance=0.00001):
    if not path or len(path) < 2:
        return path
    smoothed_path = [list(p) for p in path]
    change = tolerance
    max_iterations = 100  # 반복 횟수 제한
    iteration = 0
    while change >= tolerance and iteration < max_iterations:
        change = 0.0
        for i in range(1, len(path) - 1):
            for j in range(2):
                aux = smoothed_path[i][j]
                smoothed_path[i][j] += (
                    weight_data * (path[i][j] - smoothed_path[i][j]) +
                    weight_smooth * (smoothed_path[i-1][j] + smoothed_path[i+1][j] - 2 * smoothed_path[i][j])
                )
                change += abs(aux - smoothed_path[i][j])
        iteration += 1
    print(f"경로 스무딩 완료: 반복={iteration}, 최종 change={change:.6f}")
    return smoothed_path

# 위경도 변환
def px_to_latlon(x, y, lat0, lon0, grid_size, base_width, base_height):
    scale_x = screen.get_width() / base_width
    scale_y = screen.get_height() / base_height
    lat = lat0 - (y / (grid_size * scale_y)) * 0.002
    lon = lon0 + (x / (grid_size * scale_x)) * 0.002
    return lat, lon

# JSON 로드
def load_or_generate_mission():
    try:
        with open(resource_path("mission.json"), "r") as f:
            return json.load(f)["waypoints"]
    except FileNotFoundError:
        return []

def load_or_generate_obstacles():
    try:
        with open(resource_path("obstacles.json"), "r") as f:
            return json.load(f)["obstacles"]
    except FileNotFoundError:
        return []

def load_or_generate_scenario():
    try:
        with open(resource_path("scenario.json"), "r") as f:
            return json.load(f)["events"]
    except FileNotFoundError:
        return [
            {"time": 5.0, "use_safe_speed": False, "use_omega_limit": False, "target_speed_kmh": 40.0, "min_turn_radius": 3.0},
            {"time": 15.0, "use_safe_speed": True, "use_omega_limit": True, "target_speed_kmh": 20.0, "min_turn_radius": 5.0}
        ]

# 전역 변수 초기화
waypoints = load_or_generate_mission()
obstacles = load_or_generate_obstacles()
scenario_events = load_or_generate_scenario()

# 경로 재계산 함수
def recalculate_path(waypoints, obstacles, grid_size, map_width, map_height, fov_range):
    global full_path
    full_path = []
    if len(waypoints) >= 2:
        last_valid_point = waypoints[0]
        for i in range(len(waypoints) - 1):
            start = (int(last_valid_point["x"] // grid_size), int(last_valid_point["y"] // grid_size))
            goal = (int(waypoints[i + 1]["x"] // grid_size), int(waypoints[i + 1]["y"] // grid_size))
            path = a_star(start, goal, obstacles, grid_size, map_width, map_height, fov_range)
            if path:
                smoothed_path = smooth_path([(x * grid_size + grid_size // 2, y * grid_size + grid_size // 2) for x, y in path])
                # 중복 점 제거
                cleaned_path = [smoothed_path[0]]
                for point in smoothed_path[1:]:
                    if math.hypot(point[0] - cleaned_path[-1][0], point[1] - cleaned_path[-1][1]) > 1.0:
                        cleaned_path.append(point)
                full_path.extend(cleaned_path)
                last_valid_point = waypoints[i + 1]
            else:
                print(f"경로 생성 실패: {i} -> {i+1}, 시작: {start}, 목표: {goal}")
                return None
    print(f"전체 경로 길이: {len(full_path)}")
    return full_path

# 파라미터 업데이트 함수
def update_vehicle_params():
    global max_speed, max_lat_acc, max_acc, fov_angle, fov_range, min_turn_radius, cog_height, use_safe_speed
    global debug_modal, update_modal, update_modal_start_time, waypoints
    max_speed = temp_max_speed
    max_lat_acc = temp_max_lat_acc
    max_acc = temp_max_acc
    fov_angle = temp_fov_angle
    fov_range = temp_fov_range
    min_turn_radius = temp_min_turn_radius
    cog_height = temp_cog_height
    use_safe_speed = temp_use_safe_speed
    if ugv:
        ugv.max_speed = max_speed / 3.6
        ugv.target_speed = ugv.max_speed
        ugv.max_lat_acc = max_lat_acc
        ugv.max_acc = max_acc
        ugv.fov_angle = math.radians(fov_angle)
        ugv.fov_range = fov_range
        ugv.min_turn_radius = min_turn_radius
        ugv.cog_height = cog_height
    for wp in waypoints:
        wp["speed"] = max_speed / 3.6
    debug_modal = True
    update_modal = True
    update_modal_start_time = pygame.time.get_ticks()
    print(f"업데이트 완료: max_speed={max_speed:.1f} km/h, max_lat_acc={max_lat_acc:.1f} m/s², "
          f"max_acc={max_acc:.1f} m/s², fov_angle={fov_angle:.1f}°, fov_range={fov_range:.1f} m, "
          f"min_turn_radius={min_turn_radius:.1f} m, cog_height={cog_height:.1f} m")

# 리셋 함수
def reset_simulation():
    global waypoints, obstacles, scenario_events, full_path, ugv, path_index, input_mode
    global sim_time, simulation_ended, overturn_modal, path_error_modal, debug_modal, update_modal
    global pid_error_integral, pid_prev_error
    waypoints = []
    obstacles = []
    scenario_events = load_or_generate_scenario()
    full_path = []
    ugv = None
    path_index = 0
    input_mode = True
    sim_time = 0.0
    simulation_ended = False
    overturn_modal = False
    path_error_modal = False
    debug_modal = False
    update_modal = False
    pid_error_integral = 0.0
    pid_prev_error = 0.0
    print("시뮬레이션 리셋 완료")

# 시뮬레이션 루프
running = True
while running:
    screen_width, screen_height = screen.get_width(), screen.get_height()
    map_width = int(screen_width * 0.76)
    panel_width = screen_width - map_width
    grid_size = int(GRID_SIZE * (screen_width / BASE_WIDTH))
    scale_factor = screen_width / BASE_WIDTH

    mouse_pos = pygame.mouse.get_pos()
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_RETURN and ugv and len(waypoints) > 1 and not simulation_ended:
                new_path = recalculate_path(waypoints, obstacles, grid_size, map_width, screen_height, fov_range)
                if new_path is not None:
                    full_path = new_path
                    input_mode = False
                    path_index = 0
                    ugv.reset()
                    # 초기 경로 검증
                    if len(full_path) > 1:
                        next_x, next_y = full_path[1]
                        dx = next_x - ugv.x
                        dy = next_y - ugv.y
                        distance = math.hypot(dx, dy)
                        if distance > 5 * scale_factor:  # 최소 거리 강제
                            ugv.theta = math.atan2(dy, dx)
                        else:
                            print("초기 경로점이 너무 가까움, 다음 경로점 탐색")
                            for i in range(1, len(full_path)):
                                next_x, next_y = full_path[i]
                                distance = math.hypot(next_x - ugv.x, next_y - ugv.y)
                                if distance > 5 * scale_factor:
                                    ugv.theta = math.atan2(next_y - ugv.y, next_x - ugv.x)
                                    path_index = i
                                    break
                            else:
                                print("유효한 초기 경로점 없음, 시뮬레이션 중단")
                                path_error_modal = True
                                input_mode = True
                                continue
                    print("시뮬레이션 시작")
                else:
                    path_error_modal = True
            elif event.key == pygame.K_ESCAPE:
                reset_simulation()
            elif event.key == pygame.K_F11:
                if screen.get_flags() & pygame.FULLSCREEN:
                    screen = pygame.display.set_mode((BASE_WIDTH, BASE_HEIGHT), pygame.RESIZABLE)
                else:
                    screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
        elif event.type == pygame.MOUSEBUTTONDOWN:
            x, y = event.pos
            if show_modal and modal_close_button_rect and modal_close_button_rect.collidepoint(x, y):
                show_modal = False
            if overturn_modal and modal_close_button_rect and modal_close_button_rect.collidepoint(x, y):
                running = False
            if path_error_modal and modal_close_button_rect and modal_close_button_rect.collidepoint(x, y):
                path_error_modal = False
            if debug_modal and modal_close_button_rect and modal_close_button_rect.collidepoint(x, y):
                debug_modal = False
            if update_modal and modal_close_button_rect and modal_close_button_rect.collidepoint(x, y):
                update_modal = False
            if input_mode and x < map_width and not simulation_ended:
                if event.button == 1:
                    temp_waypoints = waypoints.copy()
                    temp_waypoints.append({"x": x, "y": y, "speed": max_speed / 3.6})
                    new_path = recalculate_path(temp_waypoints, obstacles, grid_size, map_width, screen_height, fov_range)
                    if new_path is not None:
                        waypoints = temp_waypoints
                        full_path = new_path
                        if ugv is None and len(waypoints) == 1:
                            ugv = UGV(x, y, 0, max_speed, max_lat_acc, max_acc, fov_angle, fov_range, min_turn_radius, cog_height)
                        path_index = 0
                        print(f"경로점 추가: ({x}, {y}), 총 경로점: {len(waypoints)}")
                    else:
                        path_error_modal = True
                        print(f"경로점 추가 실패: ({x}, {y})")
                elif event.button == 3:
                    obstacles.append({"x": x - 25 * scale_factor, "y": y - 25 * scale_factor, "width": 50 * scale_factor, "height": 50 * scale_factor})
                    new_path = recalculate_path(waypoints, obstacles, grid_size, map_width, screen_height, fov_range)
                    if new_path is not None:
                        full_path = new_path
                        path_index = 0
                        print(f"장애물 추가: ({x}, {y}), 총 장애물: {len(obstacles)}")
                    else:
                        obstacles.pop()
                        path_error_modal = True
                        print(f"장애물 추가로 경로 생성 실패, 롤백")
            elif x >= map_width:
                button_width = int(20 * scale_factor)
                button_height = int(20 * scale_factor)
                value_width = int(80 * scale_factor)
                panel_x = map_width + 10
                value_x = panel_x + 160

                if event.button == 1:
                    if panel_x + 130 <= x <= panel_x + 130 + button_width and 30 <= y <= 30 + button_height:
                        lat0 -= 0.1
                    elif value_x + value_width <= x <= value_x + value_width + button_width and 30 <= y <= 30 + button_height:
                        lat0 += 0.1
                    elif panel_x + 130 <= x <= panel_x + 130 + button_width and 80 <= y <= 80 + button_height:
                        lon0 -= 0.1
                    elif value_x + value_width <= x <= value_x + value_width + button_width and 80 <= y <= 80 + button_height:
                        lon0 += 0.1
                    elif panel_x + 130 <= x <= panel_x + 130 + button_width and 130 <= y <= 130 + button_height:
                        temp_max_speed = max(0, temp_max_speed - 1.0)
                    elif value_x + value_width <= x <= value_x + value_width + button_width and 130 <= y <= 130 + button_height:
                        temp_max_speed += 1.0
                    elif panel_x + 130 <= x <= panel_x + 130 + button_width and 180 <= y <= 180 + button_height:
                        temp_max_lat_acc = max(0, temp_max_lat_acc - 0.1)
                    elif value_x + value_width <= x <= value_x + value_width + button_width and 180 <= y <= 180 + button_height:
                        temp_max_lat_acc += 0.1
                    elif panel_x + 130 <= x <= panel_x + 130 + button_width and 230 <= y <= 230 + button_height:
                        temp_max_acc = max(0, temp_max_acc - 0.1)
                    elif value_x + value_width <= x <= value_x + value_width + button_width and 230 <= y <= 230 + button_height:
                        temp_max_acc += 1.0
                    elif panel_x + 130 <= x <= panel_x + 130 + button_width and 280 <= y <= 280 + button_height:
                        temp_fov_angle = max(10.0, temp_fov_angle - 5.0)
                    elif value_x + value_width <= x <= value_x + value_width + button_width and 280 <= y <= 280 + button_height:
                        temp_fov_angle = min(180.0, temp_fov_angle + 5.0)
                    elif panel_x + 130 <= x <= panel_x + 130 + button_width and 330 <= y <= 330 + button_height:
                        temp_fov_range = max(10.0, temp_fov_range - 10.0)
                    elif value_x + value_width <= x <= value_x + value_width + button_width and 330 <= y <= 330 + button_height:
                        temp_fov_range += 10.0
                    elif panel_x + 130 <= x <= panel_x + 130 + button_width and 380 <= y <= 380 + button_height:
                        temp_min_turn_radius = max(1.0, temp_min_turn_radius - 1.0)
                    elif value_x + value_width <= x <= value_x + value_width + button_width and 380 <= y <= 380 + button_height:
                        temp_min_turn_radius += 1.0
                    elif panel_x + 130 <= x <= panel_x + 130 + button_width and 430 <= y <= 430 + button_height:
                        temp_cog_height = max(0.1, temp_cog_height - 0.1)
                    elif value_x + value_width <= x <= value_x + value_width + button_width and 430 <= y <= 430 + button_height:
                        temp_cog_height += 0.1
                    elif panel_x <= x <= panel_x + 150 and 480 <= y <= 480 + button_height:
                        temp_use_safe_speed = not temp_use_safe_speed
                    else:
                        update_button_rect = pygame.Rect(panel_x + 130, 530, button_width * 3, button_height)
                        if update_button_rect.collidepoint(x, y):
                            update_vehicle_params()

    dt = clock.tick(60) / 1000.0
    if not input_mode and not simulation_ended:
        sim_time += dt

    if terrain_img_orig:
        terrain_img = pygame.transform.scale(terrain_img_orig, (map_width, screen_height))
        screen.blit(terrain_img, (0, 0))
    else:
        screen.fill((210, 180, 140), (0, 0, map_width, screen_height))
    screen.fill((220, 220, 220), (map_width, 0, panel_width, screen_height))

    for x in range(0, map_width, grid_size):
        pygame.draw.line(screen, (200, 200, 200), (x, 0), (x, screen_height))
    for y in range(0, screen_height, grid_size):
        pygame.draw.line(screen, (200, 200, 200), (0, y), (map_width, y))

    for obs in obstacles:
        if rock_img_orig:
            scaled_rock = pygame.transform.scale(rock_img_orig, (int(obs["width"]), int(obs["height"])))
            screen.blit(scaled_rock, (obs["x"], obs["y"]))
        else:
            pygame.draw.rect(screen, (139, 69, 19), (obs["x"], obs["y"], obs["width"], obs["height"]))

    for wp in waypoints:
        pygame.draw.circle(screen, (0, 255, 0), (int(wp["x"]), int(wp["y"])), int(5 * scale_factor))

    for i, point in enumerate(full_path):
        color = (255, 0, 0) if i == path_index else (0, 255, 0)
        pygame.draw.circle(screen, color, (int(point[0]), int(point[1])), int(3 * scale_factor))

    if ugv and not input_mode and path_index < len(full_path) and not simulation_ended:
        if not full_path:
            print("경고: full_path가 비어 있음")
            simulation_ended = True
        else:
            # Lookahead 경로점 선택
            lookahead_distance = 10 * scale_factor  # 10m 전방
            target_x, target_y = full_path[path_index]
            distance = math.hypot(target_x - ugv.x, target_y - ugv.y)
            while distance < lookahead_distance and path_index + 1 < len(full_path):
                path_index += 1
                target_x, target_y = full_path[path_index]
                distance = math.hypot(target_x - ugv.x, target_y - ugv.y)

            # PID 제어
            target_theta = math.atan2(target_y - ugv.y, target_x - ugv.x)
            angle_diff = (target_theta - ugv.theta + math.pi) % (2 * math.pi) - math.pi
            #global pid_error_integral, pid_prev_error
            pid_error_integral += angle_diff * dt
            pid_error_integral = max(min(pid_error_integral, 1.0), -1.0)  # 적분 항 제한
            error_derivative = (angle_diff - pid_prev_error) / dt if dt > 0 else 0
            target_omega = (
                pid_kp * angle_diff +
                pid_ki * pid_error_integral +
                pid_kd * error_derivative
            )
            pid_prev_error = angle_diff

            # 오메가 제한
            max_omega = min(ugv.max_lat_acc, ugv.v / ugv.min_turn_radius if ugv.v > 0 else float('inf'))
            ugv.omega = max(min(target_omega, max_omega), -max_omega)
            ugv.target_speed = max(ugv.max_speed * (1 - abs(angle_diff) / math.pi), 0.5)

            print(f"경로 추적: path_index={path_index}, distance={distance:.2f}, angle_diff={angle_diff:.3f} rad, "
                  f"target=({target_x:.1f}, {target_y:.1f}), ugv=({ugv.x:.1f}, {ugv.y:.1f}), "
                  f"target_theta={target_theta:.3f}, ugv.theta={ugv.theta:.3f}, "
                  f"target_omega={target_omega:.3f}, applied_omega={ugv.omega:.3f} rad/s, "
                  f"pid_terms=[P:{pid_kp*angle_diff:.3f}, I:{pid_ki*pid_error_integral:.3f}, D:{pid_kd*error_derivative:.3f}]")

            # 경로점 전환
            if distance < 3 * scale_factor:
                path_index += 1
                if path_index >= len(full_path):
                    simulation_ended = True
                    print("시뮬레이션 완료")
                else:
                    print(f"path_index 증가: {path_index}, 새 목표=({full_path[path_index][0]:.1f}, {full_path[path_index][1]:.1f})")

    if ugv:
        if not input_mode and not simulation_ended:
            ugv.update(dt, scale_factor, use_safe_speed, True, sim_time, scenario_events, path_index)
            if ugv.is_overturned:
                simulation_ended = True
                overturn_modal = True
        ugv.draw(screen, scale_factor)

    if ugv:
        speed_text = font.render(f"Current Speed: {ugv.v * 3.6:.1f} km/h", True, (0, 0, 0))
        screen.blit(speed_text, (10, 30))

    panel_x = map_width + 10
    value_x = panel_x + 160
    button_width = int(20 * scale_factor)
    button_height = int(20 * scale_factor)
    value_width = int(80 * scale_factor)

    screen.blit(font.render("Base Lat:", True, (0, 0, 0)), (panel_x, 30))
    pygame.draw.rect(screen, (200, 200, 200), (panel_x + 130, 30, button_width, button_height))
    screen.blit(font.render("▼", True, (0, 0, 0)), (panel_x + 135, 32))
    pygame.draw.rect(screen, (255, 255, 255), (value_x, 30, value_width, button_height))
    screen.blit(font.render(f"{lat0:.1f}", True, (0, 0, 0)), (value_x + 5, 32))
    pygame.draw.rect(screen, (200, 200, 200), (value_x + value_width, 30, button_width, button_height))
    screen.blit(font.render("▲", True, (0, 0, 0)), (value_x + value_width + 5, 32))

    screen.blit(font.render("Base Lon:", True, (0, 0, 0)), (panel_x, 80))
    pygame.draw.rect(screen, (200, 200, 200), (panel_x + 130, 80, button_width, button_height))
    screen.blit(font.render("▼", True, (0, 0, 0)), (panel_x + 135, 82))
    pygame.draw.rect(screen, (255, 255, 255), (value_x, 80, value_width, button_height))
    screen.blit(font.render(f"{lon0:.1f}", True, (0, 0, 0)), (value_x + 5, 82))
    pygame.draw.rect(screen, (200, 200, 200), (value_x + value_width, 80, button_width, button_height))
    screen.blit(font.render("▲", True, (0, 0, 0)), (value_x + value_width + 5, 82))

    screen.blit(font.render("Max Speed (km/h):", True, (0, 0, 0)), (panel_x, 130))
    pygame.draw.rect(screen, (200, 200, 200), (panel_x + 130, 130, button_width, button_height))
    screen.blit(font.render("▼", True, (0, 0, 0)), (panel_x + 135, 132))
    pygame.draw.rect(screen, (255, 255, 255), (value_x, 130, value_width, button_height))
    screen.blit(font.render(f"{temp_max_speed:.1f}", True, (0, 0, 0)), (value_x + 5, 132))
    pygame.draw.rect(screen, (200, 200, 200), (value_x + value_width, 130, button_width, button_height))
    screen.blit(font.render("▲", True, (0, 0, 0)), (value_x + value_width + 5, 132))

    screen.blit(font.render("Max Lat Acc (m/s²):", True, (0, 0, 0)), (panel_x, 180))
    pygame.draw.rect(screen, (200, 200, 200), (panel_x + 130, 180, button_width, button_height))
    screen.blit(font.render("▼", True, (0, 0, 0)), (panel_x + 135, 182))
    pygame.draw.rect(screen, (255, 255, 255), (value_x, 180, value_width, button_height))
    screen.blit(font.render(f"{temp_max_lat_acc:.1f}", True, (0, 0, 0)), (value_x + 5, 182))
    pygame.draw.rect(screen, (200, 200, 200), (value_x + value_width, 180, button_width, button_height))
    screen.blit(font.render("▲", True, (0, 0, 0)), (value_x + value_width + 5, 182))

    screen.blit(font.render("Max Acc (m/s²):", True, (0, 0, 0)), (panel_x, 230))
    pygame.draw.rect(screen, (200, 200, 200), (panel_x + 130, 230, button_width, button_height))
    screen.blit(font.render("▼", True, (0, 0, 0)), (panel_x + 135, 232))
    pygame.draw.rect(screen, (255, 255, 255), (value_x, 230, value_width, button_height))
    screen.blit(font.render(f"{temp_max_acc:.1f}", True, (0, 0, 0)), (value_x + 5, 232))
    pygame.draw.rect(screen, (200, 200, 200), (value_x + value_width, 230, button_width, button_height))
    screen.blit(font.render("▲", True, (0, 0, 0)), (value_x + value_width + 5, 232))

    screen.blit(font.render("FOV Angle (deg):", True, (0, 0, 0)), (panel_x, 280))
    pygame.draw.rect(screen, (200, 200, 200), (panel_x + 130, 280, button_width, button_height))
    screen.blit(font.render("▼", True, (0, 0, 0)), (panel_x + 135, 282))
    pygame.draw.rect(screen, (255, 255, 255), (value_x, 280, value_width, button_height))
    screen.blit(font.render(f"{temp_fov_angle:.1f}", True, (0, 0, 0)), (value_x + 5, 282))
    pygame.draw.rect(screen, (200, 200, 200), (value_x + value_width, 280, button_width, button_height))
    screen.blit(font.render("▲", True, (0, 0, 0)), (value_x + value_width + 5, 282))

    screen.blit(font.render("FOV Range (m):", True, (0, 0, 0)), (panel_x, 330))
    pygame.draw.rect(screen, (200, 200, 200), (panel_x + 130, 330, button_width, button_height))
    screen.blit(font.render("▼", True, (0, 0, 0)), (panel_x + 135, 332))
    pygame.draw.rect(screen, (255, 255, 255), (value_x, 330, value_width, button_height))
    screen.blit(font.render(f"{temp_fov_range:.1f}", True, (0, 0, 0)), (value_x + 5, 332))
    pygame.draw.rect(screen, (200, 200, 200), (value_x + value_width, 330, button_width, button_height))
    screen.blit(font.render("▲", True, (0, 0, 0)), (value_x + value_width + 5, 332))

    screen.blit(font.render("Min Turn Radius (m):", True, (0, 0, 0)), (panel_x, 380))
    pygame.draw.rect(screen, (200, 200, 200), (panel_x + 130, 380, button_width, button_height))
    screen.blit(font.render("▼", True, (0, 0, 0)), (panel_x + 135, 382))
    pygame.draw.rect(screen, (255, 255, 255), (value_x, 380, value_width, button_height))
    screen.blit(font.render(f"{temp_min_turn_radius:.1f}", True, (0, 0, 0)), (value_x + 5, 382))
    pygame.draw.rect(screen, (200, 200, 200), (value_x + value_width, 380, button_width, button_height))
    screen.blit(font.render("▲", True, (0, 0, 0)), (value_x + value_width + 5, 382))

    screen.blit(font.render("COG Height (m):", True, (0, 0, 0)), (panel_x, 430))
    pygame.draw.rect(screen, (200, 200, 200), (panel_x + 130, 430, button_width, button_height))
    screen.blit(font.render("▼", True, (0, 0, 0)), (panel_x + 135, 432))
    pygame.draw.rect(screen, (255, 255, 255), (value_x, 430, value_width, button_height))
    screen.blit(font.render(f"{temp_cog_height:.1f}", True, (0, 0, 0)), (value_x + 5, 432))
    pygame.draw.rect(screen, (200, 200, 200), (value_x + value_width, 430, button_width, button_height))
    screen.blit(font.render("▲", True, (0, 0, 0)), (value_x + value_width + 5, 432))

    screen.blit(font.render("Use Safe Speed:", True, (0, 0, 0)), (panel_x, 480))
    checkbox_rect_safe = pygame.Rect(panel_x + 130, 480, button_width, button_height)
    pygame.draw.rect(screen, (255, 255, 255), checkbox_rect_safe)
    if temp_use_safe_speed:
        pygame.draw.line(screen, (0, 0, 0), (panel_x + 130, 480), (panel_x + 130 + button_width, 480 + button_height), 2)
        pygame.draw.line(screen, (0, 0, 0), (panel_x + 130 + button_width, 480), (panel_x + 130, 480 + button_height), 2)

    update_button_rect = pygame.Rect(panel_x + 130, 530, button_width * 3, button_height)
    pygame.draw.rect(screen, (0, 200, 0), update_button_rect)
    screen.blit(font.render("Update", True, (255, 255, 255)), (panel_x + 135, 532))

    screen.blit(font.render("Waypoints:", True, (0, 0, 0)), (panel_x, 580))
    for i, wp in enumerate(waypoints):
        lat, lon = px_to_latlon(wp["x"], wp["y"], lat0, lon0, grid_size, BASE_WIDTH, BASE_HEIGHT)
        wp_text = font.render(f"{i+1}: ({lat:.4f}, {lon:.4f})", True, (0, 0, 0))
        screen.blit(wp_text, (panel_x, 600 + i * 20))

    if mouse_pos[0] < map_width:
        lat, lon = px_to_latlon(mouse_pos[0], mouse_pos[1], lat0, lon0, grid_size, BASE_WIDTH, BASE_HEIGHT)
        mouse_text = font.render(f"Mouse: ({lat:.4f}, {lon:.4f})", True, (0, 0, 0))
        screen.blit(mouse_text, (10, 10))

    scale_text = font.render("1 grid = 10m", True, (0, 0, 0))
    screen.blit(scale_text, (10, screen_height - 20))

    if show_modal:
        current_time = pygame.time.get_ticks()
        if current_time - modal_start_time >= modal_duration:
            show_modal = False
        else:
            modal_surface = pygame.Surface((screen_width * 0.8, 200), pygame.SRCALPHA)
            modal_surface.fill((0, 0, 0, 150))
            for i, line in enumerate(modal_text):
                text_surface = modal_font.render(line, True, (255, 255, 255))
                text_rect = text_surface.get_rect(center=(modal_surface.get_width() // 2, 30 + i * 35))
                modal_surface.blit(text_surface, text_rect)
            
            close_button_size = 30
            close_button_x = int(screen_width * 0.9 - close_button_size - 10)
            close_button_y = screen_height // 2 - 100 + 10
            modal_close_button_rect = pygame.Rect(close_button_x, close_button_y, close_button_size, close_button_size)
            pygame.draw.rect(screen, (255, 0, 0), modal_close_button_rect)
            close_text = modal_font.render("X", True, (255, 255, 255))
            screen.blit(close_text, (close_button_x + 8, close_button_y + 5))
            
            screen.blit(modal_surface, (screen_width * 0.1, screen_height // 2 - 100))

    if overturn_modal:
        modal_surface = pygame.Surface((screen_width * 0.5, 100), pygame.SRCALPHA)
        modal_surface.fill((0, 0, 0, 200))
        overturn_text = modal_font.render("차량이 전복되었습니다", True, (255, 255, 255))
        text_rect = overturn_text.get_rect(center=(modal_surface.get_width() // 2, modal_surface.get_height() // 2 - 15))
        modal_surface.blit(overturn_text, text_rect)
        instruction_text = modal_font.render("ESC: 리셋, X: 종료", True, (255, 255, 255))
        instruction_rect = instruction_text.get_rect(center=(modal_surface.get_width() // 2, modal_surface.get_height() // 2 + 15))
        modal_surface.blit(instruction_text, instruction_rect)

        close_button_size = 30
        close_button_x = int(screen_width * 0.75 - close_button_size - 10)
        close_button_y = screen_height // 2 - 50 + 10
        modal_close_button_rect = pygame.Rect(close_button_x, close_button_y, close_button_size, close_button_size)
        pygame.draw.rect(screen, (255, 0, 0), modal_close_button_rect)
        close_text = modal_font.render("X", True, (255, 255, 255))
        screen.blit(close_text, (close_button_x + 8, close_button_y + 5))

        screen.blit(modal_surface, (screen_width * 0.25, screen_height // 2 - 50))

    if path_error_modal:
        modal_surface = pygame.Surface((screen_width * 0.5, 100), pygame.SRCALPHA)
        modal_surface.fill((0, 0, 0, 200))
        error_text = modal_font.render("경로를 생성할 수 없습니다", True, (255, 255, 255))
        text_rect = error_text.get_rect(center=(modal_surface.get_width() // 2, modal_surface.get_height() // 2 - 15))
        modal_surface.blit(error_text, text_rect)
        instruction_text = modal_font.render("장애물 또는 FOV를 조정하세요", True, (255, 255, 255))
        instruction_rect = instruction_text.get_rect(center=(modal_surface.get_width() // 2, modal_surface.get_height() // 2 + 15))
        modal_surface.blit(instruction_text, instruction_rect)

        close_button_size = 30
        close_button_x = int(screen_width * 0.75 - close_button_size - 10)
        close_button_y = screen_height // 2 - 50 + 10
        modal_close_button_rect = pygame.Rect(close_button_x, close_button_y, close_button_size, close_button_size)
        pygame.draw.rect(screen, (255, 0, 0), modal_close_button_rect)
        close_text = modal_font.render("X", True, (255, 255, 255))
        screen.blit(close_text, (close_button_x + 8, close_button_y + 5))

        screen.blit(modal_surface, (screen_width * 0.25, screen_height // 2 - 50))

    if update_modal:
        current_time = pygame.time.get_ticks()
        if current_time - update_modal_start_time >= update_modal_duration:
            update_modal = False
        else:
            modal_surface = pygame.Surface((screen_width * 0.4, 50), pygame.SRCALPHA)
            modal_surface.fill((0, 0, 0, 200))
            update_text = modal_font.render("파라미터 업데이트 완료", True, (255, 255, 255))
            text_rect = update_text.get_rect(center=(modal_surface.get_width() // 2, modal_surface.get_height() // 2))
            modal_surface.blit(update_text, text_rect)
            screen.blit(modal_surface, (screen_width * 0.3, screen_height * 0.1))

    if debug_modal:
        modal_surface = pygame.Surface((screen_width * 0.5, 200), pygame.SRCALPHA)
        modal_surface.fill((0, 0, 0, 200))
        debug_texts = [
            f"Max Speed: {max_speed:.1f} km/h",
            f"Max Lat Acc: {max_lat_acc:.1f} m/s²",
            f"Max Acc: {max_acc:.1f} m/s²",
            f"FOV Angle: {fov_angle:.1f}°",
            f"FOV Range: {fov_range:.1f} m",
            f"Min Turn Radius: {min_turn_radius:.1f} m",
            f"COG Height: {cog_height:.1f} m",
            f"Use Safe Speed: {use_safe_speed}"
        ]
        for i, line in enumerate(debug_texts):
            text_surface = modal_font.render(line, True, (255, 255, 255))
            text_rect = text_surface.get_rect(center=(modal_surface.get_width() // 2, 30 + i * 20))
            modal_surface.blit(text_surface, text_rect)

        close_button_size = 30
        close_button_x = int(screen_width * 0.75 - close_button_size - 10)
        close_button_y = screen_height // 2 - 100 + 10
        modal_close_button_rect = pygame.Rect(close_button_x, close_button_y, close_button_size, close_button_size)
        pygame.draw.rect(screen, (255, 0, 0), modal_close_button_rect)
        close_text = modal_font.render("X", True, (255, 255, 255))
        screen.blit(close_text, (close_button_x + 8, close_button_y + 5))

        screen.blit(modal_surface, (screen_width * 0.25, screen_height // 2 - 100))

    pygame.display.flip()

pygame.quit()
