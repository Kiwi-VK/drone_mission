"""RPi5 + ArduCopter; surveyed local NED metres. No invented hardware calibration."""
from __future__ import annotations
import json, math, os
from pathlib import Path
PROJECT_DIR = Path(__file__).resolve().parent
LOG_DIR = Path(os.getenv('MISSION_LOG_DIR', str(PROJECT_DIR/'logs')))
MISSION_POINTS_FILE = Path(os.getenv('MISSION_POINTS_FILE', str(PROJECT_DIR/'mission_points.json')))
MISSION_DATA = json.loads(MISSION_POINTS_FILE.read_text(encoding='utf-8-sig'))
DROP_POINTS = MISSION_DATA.get('drop_points', [])
DROP_SEQUENCE = MISSION_DATA.get('drop_sequence', [])
LANDING_POINT = MISSION_DATA.get('landing_point', {})
PACKAGE_SLOTS = MISSION_DATA.get('package_slots', [])
FIELD = MISSION_DATA.get('field', {})
COMMISSIONING = MISSION_DATA.get('commissioning', {})
def number(v): return isinstance(v,(float,int)) and not isinstance(v,bool) and math.isfinite(v)
def envf(k,d): return float(os.getenv(k,d))
FLIGHT_CONTROLLER_PORT = os.getenv('FC_PORT','/dev/serial/by-id/SET_ME')
FLIGHT_CONTROLLER_BAUD = int(os.getenv('FC_BAUD','115200'))
FC_HEARTBEAT_TIMEOUT = 8.0
TELEMETRY_MAX_AGE_S = 0.6
HEARTBEAT_MAX_AGE_S = 2.0
BATTERY_LOW_VOLTAGE = envf('BATTERY_LOW_VOLTAGE',0.0)
MAX_FLIGHT_TIME = envf('MAX_FLIGHT_TIME',180.0)
TAKEOFF_ALTITUDE = envf('TAKEOFF_ALTITUDE',1.3)
DROP_ALTITUDE = TAKEOFF_ALTITUDE
TARGET_DESCENT_CM = 20
DESCENT_TOLERANCE_M = 0.035
ALTITUDE_TOLERANCE_M = 0.06
NAVIGATION_TOLERANCE_M = 0.12
MIN_ALLOWED_ALTITUDE = 0.35
MAX_ALTITUDE = envf('MAX_ALTITUDE',1.6)
AIRFRAME_HEIGHT_ABOVE_REFERENCE_M = envf('AIRFRAME_HEIGHT_ABOVE_REFERENCE_M',0.20)
FRAME_SPAN_M = envf('FRAME_SPAN_M',0.45)
LANDING_GEAR_SPAN_M = envf('LANDING_GEAR_SPAN_M',0.45)
LANDING_ZONE_DIAMETER_M = 0.60
POSITION_MARGIN_M = envf('POSITION_MARGIN_M',0.08)
LANDING_MARGIN_M = 0.025
MAX_FORWARD_SPEED_MPS = 0.25
MAX_AVOID_SPEED_MPS = 0.20
MAX_VERTICAL_SPEED_MPS = 0.12
CONTROL_PERIOD_S = 0.10
MISSION_NAV_TIMEOUT_S = 45.0
VISUAL_ALIGN_TIMEOUT_S = 20.0
LANDING_ALIGN_TIMEOUT_S = 40.0
VISUAL_STABLE_FRAMES = 8
RING_CENTER_TOLERANCE_PX = 8.0
LANDING_CENTER_TOLERANCE_PX = 10.0
VISUAL_KP, VISUAL_KI, VISUAL_KD = 0.0018,0.00008,0.00035
VISUAL_MAX_SPEED_MPS = 0.15
LAND_HANDOVER_ALT_M = 0.28
CAMERA_WIDTH,CAMERA_HEIGHT,CAMERA_FPS = 640,480,20
BOTTOM_CAMERA_PORT = os.getenv('BOTTOM_CAMERA_PORT','')
FRONT_CAMERA_PORT = os.getenv('FRONT_CAMERA_PORT','')
COLLECTION_TEMPLATE_DIR = PROJECT_DIR/'assets'/'collection_points'
TARGET_TEMPLATE_DIR = PROJECT_DIR/'assets'/'target_obstacles'
LANDING_TEMPLATE_PATH = PROJECT_DIR/'assets'/'landing_zone.png'
TARGET_MATCH_THRESHOLD = 0.78
TARGET_CONFIRM_FRAMES = 4
FRONT_HFOV_DEG = envf('FRONT_HFOV_DEG',0.0)
CAMERA_TO_BODY = MISSION_DATA.get('camera_to_body')
CAMERA_CALIBRATION = MISSION_DATA.get('camera_calibration',{})
GRID_BIND_PORT = int(os.getenv('GRID_BIND_PORT','14660'))
GRID_MAX_AGE_S = 0.4
GRID_MAX_CELLS = 6400
OBSTACLE_CLEARANCE_M = 0.10
OBSTACLE_EMERGENCY_DISTANCE_M = FRAME_SPAN_M/2+0.12
SERVO_ACTION_DURATION = envf('SERVO_ACTION_DURATION',0.6)
PAYLOAD_FEEDBACK_FILE = os.getenv('PAYLOAD_FEEDBACK_FILE','')

def inside_field(x,y,margin=None):
    if margin is None: margin=FRAME_SPAN_M/2+POSITION_MARGIN_M
    b=[FIELD.get(k) for k in ('north_min','north_max','east_min','east_max')]
    return all(number(v) for v in b) and b[0]+margin<=x<=b[1]-margin and b[2]+margin<=y<=b[3]-margin

def validate_config(*,simulation=False):
    errors=[]
    if [p.get('id') for p in DROP_POINTS]!=[1,2,3,4]: errors.append('投放点必须按 1→2→3→4 配置')
    if sorted(DROP_SEQUENCE)!=sorted(['black','white','red','yellow']): errors.append('投放顺序必须是黑白红黄各一次，按裁判公布顺序填写')
    if len(PACKAGE_SLOTS)!=4 or [s.get('color') for s in PACKAGE_SLOTS]!=DROP_SEQUENCE: errors.append('四个装载仓位必须与公布颜色顺序一致')
    for s in PACKAGE_SLOTS:
        if not isinstance(s.get('channel'),int) or not 1<=s['channel']<=16: errors.append('每个仓位需配置已核实的舵机输出通道')
        for k in ('open_pwm','close_pwm'):
            if not number(s.get(k)) or not 900<=s[k]<=2100: errors.append('舵机 PWM 超范围')
    if not 0.5<=TAKEOFF_ALTITUDE<=MAX_ALTITUDE or MAX_ALTITUDE+AIRFRAME_HEIGHT_ABOVE_REFERENCE_M+POSITION_MARGIN_M>=2.0: errors.append('高度加机体上方余量必须低于 2m')
    if not 0<LANDING_GEAR_SPAN_M+2*LANDING_MARGIN_M<0.60: errors.append('脚架外廓加余量无法进入60cm降落圈')
    if not 0.1<=FRAME_SPAN_M<=0.8 or not 0<LANDING_GEAR_SPAN_M<=FRAME_SPAN_M: errors.append('需填写含桨扫掠外廓及脚架实测尺寸')
    if simulation: return errors
    for key in ('firmware_target_verified','uart_mapping_verified','servo_mapping_verified','indoor_position_verified','camera_capture_verified','camera_axes_verified','range_grid_verified','payload_feedback_verified','loaded_colors_verified','fc_failsafes_verified','field_survey_verified'):
        if COMMISSIONING.get(key) is not True: errors.append('未完成台架/现场核验: commissioning.'+key)
    for p in DROP_POINTS+[LANDING_POINT]:
        if not all(number(p.get(k)) for k in ('north_m','east_m')): errors.append(f"任务点 {p.get('id','landing')} 缺少现场NED坐标")
        elif not inside_field(p['north_m'],p['east_m']): errors.append('任务点不在安全边界内')
    if not inside_field(0,0): errors.append('起飞原点必须在测量安全边界内')
    for lo,hi in (('north_min','north_max'),('east_min','east_max')):
        if all(number(FIELD.get(k)) for k in (lo,hi)) and not 0<FIELD[hi]-FIELD[lo]<=5: errors.append('场地边长必须在0~5m内')
    if not BOTTOM_CAMERA_PORT or not FRONT_CAMERA_PORT or BOTTOM_CAMERA_PORT==FRONT_CAMERA_PORT: errors.append('需两个不同的前视/下视采集源，FPV发射机不是采集卡')
    if not 20<=FRONT_HFOV_DEG<=160: errors.append('需标定 FRONT_HFOV_DEG')
    m=CAMERA_TO_BODY
    if not isinstance(m,list) or len(m)!=2 or any(not isinstance(r,list) or len(r)!=2 or not all(number(v) for v in r) for r in m): errors.append('缺少2×2下视图像到机体系标定矩阵')
    elif abs(m[0][0]*m[1][1]-m[0][1]*m[1][0])<0.5: errors.append('相机坐标矩阵退化')
    for k in ('fx','fy'):
        if not number(CAMERA_CALIBRATION.get(k)) or CAMERA_CALIBRATION[k]<=0: errors.append('缺少相机内参 '+k)
    if CAMERA_CALIBRATION.get('undistorted') is not True: errors.append('下视输入需已去畸变并完成尺寸标定')
    for f in [LANDING_TEMPLATE_PATH]+[COLLECTION_TEMPLATE_DIR/f'point_{i}.png' for i in range(1,5)]+[TARGET_TEMPLATE_DIR/f'obstacle_{i}.png' for i in (1,2)]:
        if not f.is_file(): errors.append('缺少现场视觉模板: '+str(f))
    if BATTERY_LOW_VOLTAGE<=0: errors.append('必须按电池串数及负载配置 BATTERY_LOW_VOLTAGE')
    if not PAYLOAD_FEEDBACK_FILE: errors.append('必须配置投放反馈文件 PAYLOAD_FEEDBACK_FILE')
    for oid in ('1','2'):
        ob=MISSION_DATA.get('fixed_obstacles',{}).get(oid,{})
        if not all(number(ob.get(k)) for k in ('north_m','east_m')): errors.append('固定障碍物'+oid+'缺少现场位置，不能绑定目标身份')
    end=FIELD.get('end_zone')
    if not isinstance(end,list) or len(end)!=4 or not all(number(v) for v in end): errors.append('需测量终点禁入区 [north_min,north_max,east_min,east_max]')
    elif not end[0]<end[1] or not end[2]<end[3]: errors.append('终点区边界顺序错误')
    if not all(number(MISSION_DATA.get(k)) for k in ('local_origin_north_m','local_origin_east_m','local_origin_down_m')): errors.append('需记录静止起飞时 LOCAL_POSITION_NED 的 x/y/z 原点读数')
    return errors
