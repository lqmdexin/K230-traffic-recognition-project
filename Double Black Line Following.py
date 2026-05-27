# K230 CanMV 道路识别巡线模块
import time, os, sys, math

from media.sensor import *
from media.display import *
from media.media import *

# 黑线道路阈值 (L Min, L Max, A Min, A Max, B Min, B Max)
road_threshold = [(23, 0, -45, 19, -31, 28)]

# 感兴趣区域 (x, y, w, h) - 根据640x480分辨率调整
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

try:
    sensor = Sensor(width=640, height=480)
    sensor.reset()

    sensor.set_framesize(Sensor.VGA)
    sensor.set_pixformat(Sensor.RGB565)

    Display.init(Display.ST7701, sensor.width(), sensor.height(), fps=90, to_ide=True)
    MediaManager.init()
    sensor.run()

    while True:
        os.exitpoint()
        img = sensor.snapshot(chn=CAM_CHN_ID_0)

        blobs = img.find_blobs(road_threshold, roi=ROI, merge=True)

        ratio = 0

        if blobs:
            left_blob, right_blob = get_top2_blobs(blobs)

            if left_blob is None or right_blob is None:
                print("Out Of Range")
            else:
                # 画出车道左边线
                img.draw_rectangle(left_blob.x(), left_blob.y(),
                                   left_blob.w(), left_blob.h(),
                                   color=(0, 255, 0), thickness=2)
                img.draw_cross(left_blob.cx(), left_blob.cy(),
                               color=(0, 255, 0))

                # 画出车道右边线
                img.draw_rectangle(right_blob.x(), right_blob.y(),
                                   right_blob.w(), right_blob.h(),
                                   color=(0, 255, 0), thickness=2)
                img.draw_cross(right_blob.cx(), right_blob.cy(),
                               color=(0, 255, 0))

                # 计算并显示偏转角度
                direct_ratio = get_direction(left_blob, right_blob)
                draw_direct(img, direct_ratio)

                ratio = int(math.degrees(direct_ratio))
                img.draw_string_advanced(10, 10,24, "%d" % ratio,
                                color=(255, 255, 255), scale=2)
                print(ratio)

        # 画出感兴趣区域
        img.draw_rectangle(ROI[0], ROI[1], ROI[2], ROI[3],
                           color=(255, 0, 0), thickness=2)

        Display.show_image(img)

except KeyboardInterrupt as e:
    print("user stop: ", e)
except BaseException as e:
    print(f"Exception {e}")
finally:
    if isinstance(sensor, Sensor):
        sensor.stop()
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    MediaManager.deinit()
