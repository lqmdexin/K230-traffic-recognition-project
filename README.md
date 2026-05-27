# 电赛基于视觉引导的智能车系统K230视觉代码实现 
  这几份代码是我电赛在视觉部分中基于K230弄的代码,其中还有很多部分的代码可以优化,但是本人已经要投入下一阶段的学习了没时间来优化了 欢迎参考该代码的用户在此基础上进行优化或者改良. 
# Double Black Line Following.py:
核心算法是对比左右黑线的宽度是否一样.同时会给单片机返回一个偏航值，可以利用串口发送会stm32然后进行一个视觉环的pid控制 
# Traffic Light Recognition.py:
基于机器视觉来识别红绿灯的，阈值不一定合适，如在识别到过亮的地方会识别出STOP（RED）可以在机器视觉里自己调整 
# Traffic Sign Recognition.py:
采用的是官方例程中的模板匹配,也可以尝试自己去训练模型,因为模板匹配有时会识别不出来图像 
# Three-Task Main Code.py:
三个文件的代码组合一起的多线程代码。我觉得后续线程可以进行优化，可以尝试使用线程一采集RGB图，先对彩色的图进行识别，因为 红灯与停止标志都是红色占比较多的像素,这样就可以直接停止,省去灰度图中模板匹配所占用的时间.然后先识别到左转右转标志的蓝色然后再进入模板匹配的环节 主要目的都是为了节省从开机就时时刻刻处于不断模板匹配所占用的资源(模板匹配太多时会导致严重卡顿),同时也可以避免抢占线程(以上为本人对后续线程改进的观点 尚未进行尝试) 

  These are the K230-based vision codes I developed for the visual part of my electronics competition. There is still plenty of room for optimization in many parts, but I have to move on to the next stage of my studies and no longer have time to work on them. Users who refer to this code are welcome to optimize or improve it further. 
  # Double Black Line Following.py: 
  The core algorithm compares the width of the left and right black lines to determine the vehicle's offset. It returns a yaw value, which can be sent to the STM32 via UART for PID control in the visual loop. 
  # Traffic Light Recognition.py: 
  Recognizes traffic lights using machine vision. The current threshold values may not be suitable for all environments; overexposed areas can sometimes be misidentified as the STOP (RED) state. You can adjust the thresholds in the machine vision tool as needed. 
  # Traffic Sign Recognition.py: 
  Uses template matching from the official example. You can also try training your own model, as template matching may fail to detect images in some cases. 
  # Three-Task Main Code.py: 
  A multi-threaded script combining all three programs. I believe the threads could be further optimized: One idea is to use the first thread to capture RGB images and perform color-based recognition first. Since red traffic lights and stop signs have a high proportion of red pixels, this would allow the system to stop immediately without waiting for template matching on grayscale images. Then, after detecting the blue color of left/right turn signs, the template matching process could be triggered. The main goal is to reduce the constant template matching that consumes significant resources (too many matches cause severe lag) and avoid thread preemption. (These are my suggestions for future improvements and have not yet been tested.)
