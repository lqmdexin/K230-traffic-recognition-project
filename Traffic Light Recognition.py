import time, os, sys

from media.sensor import *
from media.display import *
from media.media import *

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
        (0, 0, 640, 480),
        x_stride=5,
        y_stride=5,
        pixels_threshold=150
    )

    max_b = find_max_blob(blobs)

    led_color = 0   # 0=无, 1=红, 2=黄, 3=绿
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
        img.draw_string(max_b.x(), max_b.y() - 10, show_txt,
                         color=(0, 0, 255), scale=2)

    return led_color, show_txt

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

        led_color, show_txt = detect_light(img)

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
