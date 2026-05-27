import time, os, sys
from media.sensor import *  #导入sensor模块，使用摄像头相关接口
from media.display import * #导入display模块，使用display相关接口
from media.media import *   #导入media模块，使用meida相关接口
from image import SEARCH_EX
from machine import UART, Pin, FPIOA


def detect_sign(img):
    flag = 0

    r2 = img.find_template(template2, 0.65,roi=(80, 60, 160, 120), step=4, search=SEARCH_EX)
    if r2:
        img.draw_rectangle(r2, color=(255, 0, 0), thickness=2)
        print("right")
        return 2

    r3 = img.find_template(template3, 0.75,roi=(80, 60, 160, 120),  step=8, search=SEARCH_EX)
    if r3:
        img.draw_rectangle(r3, color=(255, 0, 0), thickness=2)
        print("left")
        return 3

    r4 = img.find_template(template4, 0.65,roi=(80, 65, 160, 120),  step=8, search=SEARCH_EX)
    if r4:
        img.draw_rectangle(r4, color=(255, 0, 0), thickness=2)
        print("stop")
        return 4

    return 0

try:
    sensor = Sensor(width=640, height=480) #构建摄像头对象
    sensor.reset() #复位和初始化摄像头

    sensor.set_framesize(Sensor.VGA)      #设置帧大小VGA(640x480)，默认通道0
    sensor.set_pixformat(Sensor.GRAYSCALE) #设置输出图像格式，默认通道0

    #使用IDE缓冲区输出图像,显示尺寸和sensor配置一致。
    Display.init(Display.ST7701, sensor.width(), sensor.height(), fps=90, to_ide=True)
    MediaManager.init() #初始化media资源管理器
    sensor.run() #启动sensor

    template2 = image.Image("/sdcard/template2.pgm")  # 向右转弯
    template3 = image.Image("/sdcard/template3.pgm")  # 向左转弯
    template4 = image.Image("/sdcard/template4.pgm")  # 停车

    while True:
        os.exitpoint() #检测IDE中断
        img = sensor.snapshot(chn=CAM_CHN_ID_0)


        flag = detect_sign(img)

        img.draw_string_advanced(10, 40,24, "flag=%d" % flag, color=(255, 255, 255), scale=2)

        Display.show_image(img)

# IDE中断释放资源代码
except KeyboardInterrupt as e:
    print("user stop: ", e)
except BaseException as e:
    print(f"Exception {e}")
finally:
    # sensor stop run
    if isinstance(sensor, Sensor):
        sensor.stop()
    # deinit display
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    # release media buffer
    MediaManager.deinit()
