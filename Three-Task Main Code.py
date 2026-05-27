import time, os, sys,math
import _thread
from media.sensor import *
from media.display import *
from media.media import *
from machine import Pin, UART, FPIOA
import image
from image import SEARCH_EX

############# 全局变量 ###############

cam_lock = _thread.allocate_lock() #线程锁
sensor = None
uart1 = None

road_stop = False
light_stop = False
sign_stop = False

main_img_color = None
main_img_gray = None

road_angle = 0       #偏转角度
led_color = 0        # 0=无, 1=红, 2=黄, 3=绿
show_txt = "NONE"
sign_result = None
main_img = None
sign_cnt = 0
sign_flag = 0
sign_cmd = 0           # 当前锁存的标志动作
sign_lock = False      # 是否正在锁存
sign_lock_start = 0    # 锁存开始时间
SIGN_HOLD_MS = 1500    # 左转/右转保持时间
STOP_HOLD_MS = 2000    # 停车保持时间

############# 串口通信 ###############

# 实例化 FPIOA
fpioa = FPIOA()

# UART1 引脚映射
fpioa.set_function(40, FPIOA.UART1_TXD)
fpioa.set_function(41, FPIOA.UART1_RXD)

# 初始化 UART1
uart1 = UART(
    UART.UART1,
    baudrate=9600,
    bits=UART.EIGHTBITS,
    parity=UART.PARITY_NONE,
    stop=UART.STOPBITS_ONE
)

def send_UART(mode,angle):
    global uart1
    uart1.write("<MODE:{},ANG:{}>\n".format((mode),int(angle * 10)))
############ 函数封装部分 ##############
############ 红绿灯识别 ##############

def find_max_blob(blobs):
    max_blob = None
    max_pixels = 0
    for b in blobs:
        if b.pixels() > max_pixels:
            max_pixels = b.pixels()
            max_blob = b
    return max_blob

def detect_light(img):
    light_thres = [
        (97, 100, -27, 9, -10, 87),    # 红灯
        (67, 100, 2, 11, 27, 67),      # 黄灯
        (84, 92, -88, -44, -13, 86)    # 绿灯
    ]

    blobs = img.find_blobs(
        light_thres,
        False,
        (0, 80, 640, 220),
        x_stride=5,
        y_stride=5,
        pixels_threshold=150
    )

    max_b = find_max_blob(blobs)

    led_color = 0
    show_txt = "NONE"

    if max_b is not None:
        if max_b.code() == 1:
            led_color = 1
            show_txt = "STOP1"
        elif max_b.code() == 2:
            led_color = 2
            show_txt = "STOP2"
        elif max_b.code() == 4:
            led_color = 3
            show_txt = "GO"

        img.draw_rectangle(max_b.x(), max_b.y(), max_b.w(), max_b.h(),
                           color=(0, 255, 0), thickness=4)
        img.draw_string_advanced(max_b.x(), max_b.y() - 10,24, show_txt,
                         color=(0, 0, 255), scale=2)

    return led_color, show_txt

############ 道路识别 ##############
# 黑线道路阈值
road_threshold = [(23, 0, -45, 19, -31, 28)]

# (x, y, w, h)
ROI = (0, 200, 640, 80)

def get_top2_blobs(blobs):
    """获取面积最大的两个色块，按x坐标排序返回左右色块"""
    if len(blobs) < 2:
        return None, None
    sorted_blobs = sorted(blobs, key=lambda b: b.pixels(), reverse=True)
    top2 = sorted_blobs[:2]
    top2.sort(key=lambda b: b.cx())
    return top2[0], top2[1]

def get_direction(left_blob, right_blob):
    """
    根据左右两块黑色区域计算摄像头偏转角度
    ratio < 0 左拐，小车在车道偏右位置
    ratio > 0 右拐，小车在车道偏左位置
    """
    MAX_WIDTH = 640
    theta = 0.01
    b = 3

    x1 = left_blob.x() - int(0.5 * left_blob.w())
    x2 = right_blob.x() + int(0.5 * right_blob.w())

    w_left = x1
    w_center = math.fabs(x2 - x1)
    w_right = math.fabs(MAX_WIDTH - x2)

    direct_ratio = (w_left + b + theta * w_center) / \
                   (w_left + w_right + 2 * b + 2 * theta * w_center) - 0.5

    return direct_ratio

def draw_direct(img, direct_ratio):
    """可视化偏转方向"""
    center_x = 320
    center_y = 240
    end_x = int(center_x + direct_ratio * 200)
    img.draw_line(center_x, center_y, end_x, center_y - 60,
                  color=(0, 255, 0), thickness=2)
############ 交通识别部分 ##############
def detect_sign(img):
    r2 = img.find_template(template2, 0.55, roi=((80, 40, 220, 160)), step=4, search=SEARCH_EX)
    if r2:
        img.draw_rectangle(r2, color=(255, 0, 0), thickness=2)
        print("right")
        return 2

    r3 = img.find_template(template3, 0.70, roi=((80, 40, 220, 160)), step=4, search=SEARCH_EX)
    if r3:
        img.draw_rectangle(r3, color=(255, 0, 0), thickness=2)
        print("left")
        return 3

    r4 = img.find_template(template4, 0.70, roi=((80, 40, 220, 160)), step=4, search=SEARCH_EX)
    if r4:
        img.draw_rectangle(r4, color=(255, 0, 0), thickness=2)
        print("stop")
        return 4

    return 0
############ 线程部分 ##############
###########道路识别线程##############
def road_thread():
    global sensor, road_stop, road_angle, main_img

    while not road_stop:
        try:
            if not cam_lock.acquire():
                time.sleep_ms(5)
                continue

            img = sensor.snapshot(chn=CAM_CHN_ID_0)

            blobs = img.find_blobs(road_threshold, roi=ROI, merge=True)
            road_angle = 0

            if blobs:
                left_blob, right_blob = get_top2_blobs(blobs)

                if left_blob is not None and right_blob is not None:
                    img.draw_rectangle(left_blob.x(), left_blob.y(),
                                       left_blob.w(), left_blob.h(),
                                       color=(0, 255, 0), thickness=2)
                    img.draw_cross(left_blob.cx(), left_blob.cy(),
                                   color=(0, 255, 0))

                    img.draw_rectangle(right_blob.x(), right_blob.y(),
                                       right_blob.w(), right_blob.h(),
                                       color=(0, 255, 0), thickness=2)
                    img.draw_cross(right_blob.cx(), right_blob.cy(),
                                   color=(0, 255, 0))

                    direct_ratio = get_direction(left_blob, right_blob)

                    center_x = 320
                    center_y = 240
                    end_x = int(center_x + direct_ratio * 200)
                    img.draw_line(center_x, center_y, end_x, center_y - 60,
                                  color=(0, 255, 0), thickness=2)

                    road_angle = int(math.degrees(direct_ratio))
                    img.draw_string_advanced(10, 10, 24, "%d" % road_angle,
                                             color=(255, 255, 255), scale=2)
                else:
                    road_angle = 0

            img.draw_rectangle(ROI[0], ROI[1], ROI[2], ROI[3],
                               color=(255, 0, 0), thickness=2)

            # 只把最新图给主线程显示
            main_img = img

        except Exception as e:
            print("road_thread err:", e)

        finally:
            try:
                cam_lock.release()
            except:
                pass

        time.sleep_ms(10)

############ 红绿灯线程 ##############

def light_thread():
    """红绿灯识别线程"""
    global sensor, light_stop, led_color, show_txt, main_img

    while not light_stop:
        try:
            if not cam_lock.acquire():
                time.sleep_ms(5)
                continue

            img = sensor.snapshot(chn=CAM_CHN_ID_0)
            led_color, show_txt = detect_light(img)

            # 如果你希望红绿灯也画在图上，就保留
            main_img = img

        except Exception as e:
            print("light_thread err:", e)

        finally:
            try:
                cam_lock.release()
            except:
                pass

        time.sleep_ms(30)
############ 交通标志线程 ##############
def sign_thread():
    """交通标志识别线程"""
    global sensor, sign_stop, sign_flag, main_img_gray, sign_cnt
    global sign_cmd, sign_lock, sign_lock_start
    while not sign_stop:
        try:
            if not cam_lock.acquire():
                time.sleep_ms(5)
                continue

            # 灰度通道
            img = sensor.snapshot(chn=CAM_CHN_ID_1)
            sign_cnt += 1
            if sign_cnt % 5 == 0:
                res = detect_sign(img)

                if (not sign_lock) and res != 0:
                    sign_cmd = res
                    sign_lock = True
                    sign_lock_start = time.ticks_ms()

            img.draw_string_advanced(10, 80, 24, "sign=%d" % sign_flag,
                                     color=(255, 255, 255), scale=2)

            main_img_gray = img

        except Exception as e:
            print("sign_thread err:", e)

        finally:
            try:
                cam_lock.release()
            except:
                pass

        time.sleep_ms(50)


############ 主线程 ##############
try:
    sensor = Sensor(width=640, height=480)
    sensor.reset()

    sensor.set_framesize(Sensor.VGA, chn=CAM_CHN_ID_0)
    sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)

    sensor.set_framesize(Sensor.VGA, chn=CAM_CHN_ID_1)
    sensor.set_pixformat(Sensor.GRAYSCALE, chn=CAM_CHN_ID_1)
    Display.init(Display.ST7701, sensor.width(), sensor.height(), fps=90, to_ide=False)
    MediaManager.init()
    sensor.run()

    template2 = image.Image("/sdcard/template2.pgm")  # 向右转
    template3 = image.Image("/sdcard/template3.pgm")  # 向左转
    template4 = image.Image("/sdcard/template4.pgm")  # 停车

    last_send_time = 0
    send_interval = 50

    _thread.start_new_thread(road_thread, ())
    _thread.start_new_thread(light_thread, ())
    _thread.start_new_thread(sign_thread, ())

    mode = 1
    last_mode = -1
    #mode = 0：停车
    #mode = 1：正常巡线
    #mode = 2：左转
    #mode = 3：右转

    while True:
        os.exitpoint()

        now = time.ticks_ms()

        if sign_lock:
            if sign_cmd in (2, 3):   # 左转 / 右转
                if time.ticks_diff(now, sign_lock_start) >= SIGN_HOLD_MS:
                    sign_lock = False
                    sign_cmd = 0

            elif sign_cmd == 4:      # 停车
                if time.ticks_diff(now, sign_lock_start) >= STOP_HOLD_MS:
                    sign_lock = False
                    sign_cmd = 0

        if led_color == 1 or led_color == 2:
            mode = 0
        elif sign_cmd == 4:
            mode = 0
        elif sign_cmd == 3:
            mode = 2
        elif sign_cmd == 2:
            mode = 3
        else:
            mode = 1

        if mode == 1:
            angle_to_send = road_angle
        else:
            angle_to_send = 0

        now = time.ticks_ms()

        if mode != last_mode:
            send_UART(mode, angle_to_send)
            last_mode = mode
            last_send_time = now

        elif time.ticks_diff(now, last_send_time) >= send_interval:
            send_UART(mode, angle_to_send)
            last_send_time = now

        if main_img is not None:
            Display.show_image(main_img)

        time.sleep_ms(20)

except KeyboardInterrupt as e:
    print("user stop: ", e)
except BaseException as e:
    print(f"Exception {e}")
finally:
    road_stop = True
    light_stop = True
    sign_stop = True

    time.sleep_ms(100)
    if isinstance(sensor, Sensor):
        sensor.stop()
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    MediaManager.deinit()
