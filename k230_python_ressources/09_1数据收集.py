# 搜索学不会电磁场看教程
# 第九课，AI识别数字，电赛题目智能送药小车需要识别数字，现在，我们先收集数据

import time
import os
import sys

from media.sensor import *
from media.display import *
from media.media import *
from time import ticks_ms
from machine import FPIOA
from machine import Pin
from machine import PWM
from machine import Timer
import time


sensor = None

try:

    print("camera_test")
    fpioa = FPIOA()
    fpioa.help()
    fpioa.set_function(53, FPIOA.GPIO53)

    key = Pin(53, Pin.IN, Pin.PULL_DOWN)

    # -------------------------------------------------------------
    # 修改点 1: 添加 id=2 (参考fhd.py)，并将初始化分辨率设为 640x480
    # -------------------------------------------------------------
    sensor = Sensor(id=2, width=640, height=480)
    sensor.reset()

    # -------------------------------------------------------------
    # 修改点 2: 显式设置帧大小为 640x480 (原代码此处被重写为1024x768)
    # -------------------------------------------------------------
    sensor.set_framesize(width=640, height=480)
    sensor.set_pixformat(Sensor.RGB565)

    Display.init(Display.ST7701, width=800, height=480, to_ide=True)
    # 初始化媒体管理器
    MediaManager.init()
    # 启动 sensor
    sensor.run()
    clock = time.clock()

    counter = 0
    save_folder = "/data/data/images/" # 确保SD卡中有此路径，否则请改为 /sdcard/images/ 等
    class_lst = ["1","2","3","4","5"]
    class_id = -1
    prefix = "batch_1_"

    while True:
        clock.tick()
        os.exitpoint()
        img = sensor.snapshot(chn=CAM_CHN_ID_0)

        if key.value() == 1:
            class_id = (class_id + 1) % len(class_lst)
            # 尝试创建目录，如果报错说明父目录不存在
            try:
                os.mkdir(save_folder+class_lst[class_id])
            except:
                pass

            for i in range(3):
                print("will collect {} class in {} s".format(class_lst[class_id], 3-i))
                time.sleep_ms(1000)
            counter = 100

        if not counter == 0:
            time.sleep_ms(50)
            file_name = "{}_{}_{}.jpg".format(prefix, class_lst[class_id], str(counter))
            # 压缩保存
            save_img = img.compress(95)
            file_path = save_folder + class_lst[class_id] + "/" + file_name
            with open(file_path, 'wb') as f:
                f.write(save_img)
            print("img saved to \"{}\"".format(file_path))
            counter -= 1

        img.draw_string_advanced(50, 50, 80, "fps: {}".format(clock.fps()), color=(255, 0, 0))

        # -------------------------------------------------------------
        # 修改点 3: 注释掉 midpoint_pool
        # 原代码用了池化会把图像缩小一半，导致变成 320x240，我们需要保持 640x480
        # -------------------------------------------------------------
        # img.midpoint_pool(2, 2)

        img.compressed_for_ide()

        # -------------------------------------------------------------
        # 修改点 4: 调整显示位置
        # 屏幕宽800，图宽640 -> x偏移 (800-640)/2 = 80
        # 屏幕高480，图高480 -> y偏移 0
        # -------------------------------------------------------------
        Display.show_image(img, x=(800-640)//2, y=0)

except KeyboardInterrupt as e:
    print("用户停止: ", e)
except BaseException as e:
    print(f"异常: {e}")
finally:
    if isinstance(sensor, Sensor):
        sensor.stop()
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    MediaManager.deinit()
