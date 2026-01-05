import os
import ujson
import aicube
from media.sensor import *
from media.display import *
from media.media import *
import network
import socket
import time
import nncase_runtime as nn
import ulab.numpy as np
import image
import random
import gc
import ustruct             # 用于打包二进制数据
from machine import UART
from machine import FPIOA

# ================= 用户配置区域 =================
WIFI_SSID = "1234"
WIFI_PASS = "12345789"
PORT = 8080
DEFAULT_SAVE_PATH = "/sdcard/images/"

# 串口3 配置 (K230 CanMV 一般是 11/12，请根据实际接线修改)
UART3_TX_PIN = 50
UART3_RX_PIN = 51
UART_BAUDRATE = 115200

# 显示模式
display_mode="lcd"
if display_mode=="lcd":
    DISPLAY_WIDTH = ALIGN_UP(640, 16)
    DISPLAY_HEIGHT = 480
else:
    DISPLAY_WIDTH = ALIGN_UP(1920, 16)
    DISPLAY_HEIGHT = 1080

# 640*480
OUT_WIDTH = ALIGN_UP(640, 8)
OUT_HEIGHT = 480

# 颜色盘
color_four = [(255, 220, 20, 60), (255, 119, 11, 32), (255, 0, 0, 142), (255, 0, 0, 230),
        (255, 106, 0, 228), (255, 0, 60, 100), (255, 0, 80, 100), (255, 0, 0, 70),
        (255, 0, 0, 192), (255, 250, 170, 30), (255, 100, 170, 30), (255, 220, 220, 0),
        (255, 175, 116, 175), (255, 250, 0, 30), (255, 165, 42, 42), (255, 255, 77, 255),
        (255, 0, 226, 252), (255, 182, 182, 255), (255, 0, 82, 0), (255, 120, 166, 157),
        (255, 110, 76, 0), (255, 174, 57, 255), (255, 199, 100, 0), (255, 72, 0, 118),
        (255, 255, 179, 240), (255, 0, 125, 92), (255, 209, 0, 151), (255, 188, 208, 182),
        (255, 0, 220, 176), (255, 255, 99, 164), (255, 92, 0, 73), (255, 133, 129, 255),
        (255, 78, 180, 255), (255, 0, 228, 0), (255, 174, 255, 243), (255, 45, 89, 255),
        (255, 134, 134, 103), (255, 145, 148, 174), (255, 255, 208, 186),
        (255, 197, 226, 255), (255, 171, 134, 1), (255, 109, 63, 54), (255, 207, 138, 255),
        (255, 151, 0, 95), (255, 9, 80, 61), (255, 84, 105, 51), (255, 74, 65, 105),
        (255, 166, 196, 102), (255, 208, 195, 210), (255, 255, 109, 65), (255, 0, 143, 149),
        (255, 179, 0, 194), (255, 209, 99, 106), (255, 5, 121, 0), (255, 227, 255, 205),
        (255, 147, 186, 208), (255, 153, 69, 1), (255, 3, 95, 161), (255, 163, 255, 0),
        (255, 119, 0, 170), (255, 0, 182, 199), (255, 0, 165, 120), (255, 183, 130, 88),
        (255, 95, 32, 0), (255, 130, 114, 135), (255, 110, 129, 133), (255, 166, 74, 118),
        (255, 219, 142, 185), (255, 79, 210, 114), (255, 178, 90, 62), (255, 65, 70, 15),
        (255, 127, 167, 115), (255, 59, 105, 106), (255, 142, 108, 45), (255, 196, 172, 0),
        (255, 95, 54, 80), (255, 128, 76, 255), (255, 201, 57, 1), (255, 246, 0, 122),
        (255, 191, 162, 208)]

root_path="/sdcard/mp_deployment_source/"
config_path=root_path+"deploy_config.json"
deploy_conf={}
current_class_name = "default"
should_save_one = False
current_fps = 0

def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print(f"Connecting to: {WIFI_SSID} ...")
        wlan.connect(WIFI_SSID, WIFI_PASS)
        retry = 0
        while not wlan.isconnected():
            time.sleep(1)
            retry += 1
            if retry > 15: 
                print("WiFi connection timed out")
                return None
    ip = wlan.ifconfig()[0]
    print(f"Connected! Access address: http://{ip}")
    return ip

def ensure_dir(path):
    try: os.stat(path)
    except: 
        try: os.mkdir(path)
        except: pass

def save_current_frame(img, class_name):
    full_dir = DEFAULT_SAVE_PATH + class_name
    ensure_dir(DEFAULT_SAVE_PATH)
    ensure_dir(full_dir)
    file_name = f"{class_name}_{time.ticks_ms()}.jpg"
    img.save(full_dir + "/" + file_name, quality=95)

def read_deploy_config(config_path):
    with open(config_path, 'r') as json_file:
        return ujson.load(json_file)

def send_uart_packet(uart, has_obj, class_id, cx, cy, area):
    """
    协议: 帧头(0xAA 0x55) + 是否有物体(1B) + 类别(1B) + CX(2B) + CY(2B) + 面积(4B) + 帧尾(0xED)
    """
    if uart is None: return
    try:
        packet = ustruct.pack(">BBBBHHIB", 0xAA, 0x55, has_obj, class_id, cx, cy, area, 0xED)
        uart.write(packet)
    except Exception as e:
        print("UART Send Error:", e)

def detection():
    global current_class_name, should_save_one, current_fps

    # ================= 1. 初始化串口 3 =================
    try:
        fpioa = FPIOA()
        # 使用顶部定义的变量，方便修改
        fpioa.set_function(UART3_TX_PIN, FPIOA.UART3_TXD)
        fpioa.set_function(UART3_RX_PIN, FPIOA.UART3_RXD)
        
        uart3 = UART(UART.UART3, baudrate=UART_BAUDRATE, bits=UART.EIGHTBITS, parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)
        print(">>> UART3 Initialized Success <<<")
    except Exception as e:
        print(f"UART3 INIT ERROR: {e}")
        uart3 = None

    # ================= 2. 连接 WiFi =================
    ip = connect_wifi()
    if not ip: 
        print("WiFi Fail")
        ip_display_text = "WiFi Disconnected"
    else:
        ip_display_text = f"http://{ip}"
    
    # Socket 初始化
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('0.0.0.0', PORT))
    s.listen(5)
    s.setblocking(False) 
    stream_client = None
    MP_HEADER = "HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n"
    BOUNDARY = "\r\n--frame\r\nContent-Type: image/jpeg\r\n\r\n"

    # ================= 3. AI 模型加载 =================
    print("det_infer start")
    deploy_conf=read_deploy_config(config_path)
    kmodel_name=deploy_conf["kmodel_path"]
    labels=deploy_conf["categories"]
    confidence_threshold= deploy_conf["confidence_threshold"]
    nms_threshold = deploy_conf["nms_threshold"]
    img_size=deploy_conf["img_size"]
    num_classes=deploy_conf["num_classes"]
    nms_option = deploy_conf["nms_option"]
    model_type = deploy_conf["model_type"]
    
    if model_type == "AnchorBaseDet":
        anchors = deploy_conf["anchors"][0] + deploy_conf["anchors"][1] + deploy_conf["anchors"][2]
    
    kmodel_frame_size = img_size
    frame_size = [OUT_WIDTH,OUT_HEIGHT]
    strides = [8,16,32]

    # Padding 计算
    ori_w = OUT_WIDTH; ori_h = OUT_HEIGHT;
    width = kmodel_frame_size[0]; height = kmodel_frame_size[1];
    ratiow = float(width) / ori_w; ratioh = float(height) / ori_h;
    if ratiow < ratioh: ratio = ratiow
    else: ratio = ratioh
    new_w = int(ratio * ori_w); new_h = int(ratio * ori_h);
    dw = float(width - new_w) / 2; dh = float(height - new_h) / 2;
    top = int(round(dh - 0.1)); bottom = int(round(dh + 0.1));
    left = int(round(dw - 0.1)); right = int(round(dw - 0.1));

    kpu = nn.kpu()
    ai2d = nn.ai2d()
    kpu.load_kmodel(root_path+kmodel_name)
    ai2d.set_dtype(nn.ai2d_format.NCHW_FMT, nn.ai2d_format.NCHW_FMT, np.uint8, np.uint8)
    ai2d.set_pad_param(True, [0,0,0,0,top,bottom,left,right], 0, [114,114,114])
    ai2d.set_resize_param(True, nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel )
    ai2d_builder = ai2d.build([1,3,OUT_HEIGHT,OUT_WIDTH], [1,3,height,width])

    # ================= 4. 摄像头与显示初始化 =================
    sensor = Sensor()
    sensor.reset()
    sensor.set_hmirror(False); sensor.set_vflip(False)
    sensor.set_framesize(width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT)
    sensor.set_pixformat(PIXEL_FORMAT_YUV_SEMIPLANAR_420)
    sensor.set_framesize(width=OUT_WIDTH, height=OUT_HEIGHT, chn=CAM_CHN_ID_1)
    sensor.set_pixformat(PIXEL_FORMAT_RGB_565, chn=CAM_CHN_ID_1)
    sensor.set_framesize(width=OUT_WIDTH , height=OUT_HEIGHT, chn=CAM_CHN_ID_2)
    sensor.set_pixformat(PIXEL_FORMAT_RGB_888_PLANAR, chn=CAM_CHN_ID_2)

    sensor_bind_info = sensor.bind_info(x=0, y=0, chn=CAM_CHN_ID_0)
    Display.bind_layer(**sensor_bind_info, layer=Display.LAYER_VIDEO1)
    
    if display_mode=="lcd": Display.init(Display.ST7701, to_ide=True)
    else: Display.init(Display.LT9611, to_ide=True)

    # OSD 层用于画框和写字
    osd_img = image.Image(DISPLAY_WIDTH, DISPLAY_HEIGHT, image.ARGB8888)
    
    try:
        MediaManager.init()
        sensor.run()
        rgb888p_img = None
        stream_img = None
        ai2d_input_tensor = None
        data = np.ones((1,3,width,height),dtype=np.uint8)
        ai2d_output_tensor = nn.from_numpy(data)
        
        clock = time.clock()
        
        while True:
            clock.tick()
            rgb888p_img = sensor.snapshot(chn=CAM_CHN_ID_2)
            
            # AI 推理
            if rgb888p_img.format() == image.RGBP888:
                ai2d_input = rgb888p_img.to_numpy_ref()
                ai2d_input_tensor = nn.from_numpy(ai2d_input)
                ai2d_builder.run(ai2d_input_tensor, ai2d_output_tensor)
                kpu.set_input_tensor(0, ai2d_output_tensor)
                kpu.run()
                
                results = []
                for i in range(kpu.outputs_size()):
                    out_data = kpu.get_output_tensor(i)
                    result = out_data.to_numpy()
                    result = result.reshape((result.shape[0]*result.shape[1]*result.shape[2]*result.shape[3]))
                    del out_data
                    results.append(result)
                gc.collect()

                if model_type == "AnchorBaseDet":
                    det_boxes = aicube.anchorbasedet_post_process(results[0], results[1], results[2], kmodel_frame_size, frame_size, strides, num_classes, confidence_threshold, nms_threshold, anchors, nms_option)
                elif model_type == "GFLDet":
                    det_boxes = aicube.gfldet_post_process(results[0], results[1], results[2], kmodel_frame_size, frame_size, strides, num_classes, confidence_threshold, nms_threshold, nms_option)
                else:
                    det_boxes = aicube.anchorfreedet_post_process(results[0], results[1], results[2], kmodel_frame_size, frame_size, strides, num_classes, confidence_threshold, nms_threshold, nms_option)
                
                # --- LCD 屏幕绘制 (OSD层) ---
                osd_img.clear()
              
                if det_boxes:
                    for det_boxe in det_boxes:
                        x1_raw, y1_raw, x2_raw, y2_raw = int(det_boxe[2]), int(det_boxe[3]), int(det_boxe[4]), int(det_boxe[5])
                        cx_raw = int((x1_raw + x2_raw) / 2)
                        cy_raw = int((y1_raw + y2_raw) / 2)
                        area_raw = int((x2_raw - x1_raw) * (y2_raw - y1_raw))
                        
                        # 串口发送 (识别到物体)
                        if uart3:
                            send_uart_packet(uart3, 1, int(det_boxe[0]), cx_raw, cy_raw, area_raw)

                        # 屏幕画框转换
                        x_disp = int(x1_raw * DISPLAY_WIDTH // OUT_WIDTH)
                        y_disp = int(y1_raw * DISPLAY_HEIGHT // OUT_HEIGHT)
                        w_disp = int(float(x2_raw - x1_raw) * DISPLAY_WIDTH // OUT_WIDTH)
                        h_disp = int(float(y2_raw - y1_raw) * DISPLAY_HEIGHT // OUT_HEIGHT)
                        
                        osd_img.draw_rectangle(x_disp, y_disp, w_disp, h_disp, color=color_four[det_boxe[0]][1:])
                        label_str = labels[det_boxe[0]] + " " + str(round(det_boxe[1],2))
                        osd_img.draw_string_advanced(x_disp, y_disp-30, 32, label_str, color=color_four[det_boxe[0]][1:])
                        
                        cx_disp = int(x_disp + w_disp // 2)
                        cy_disp = int(y_disp + h_disp // 2)
                        osd_img.draw_cross(cx_disp, cy_disp, size=10, color=(0, 255, 0))
                
                else:
                    # 串口发送 (未识别，心跳包)
                    if uart3:
                        send_uart_packet(uart3, 0, 0, 0, 0, 0)

                current_fps = clock.fps()
                
                # [核心修改] 在屏幕左上角绘制 FPS 和 IP 地址
                # 位置(10,10)写FPS, 位置(10,50)写IP, 字体大小32, 红色
                osd_img.draw_string_advanced(10, 10, 32, f"FPS: {current_fps:.1f}", color=(255, 0, 0))
                osd_img.draw_string_advanced(10, 50, 32, f"IP: {ip_display_text}", color=(0, 0, 255))
                
                Display.show_image(osd_img, 0, 0, Display.LAYER_OSD3)

                # --- 视频流推流 ---
                if stream_client or should_save_one:
                    stream_img = sensor.snapshot(chn=CAM_CHN_ID_1)
                    if det_boxes:
                        for det_boxe in det_boxes:
                            x1, y1, x2, y2 = int(det_boxe[2]), int(det_boxe[3]), int(det_boxe[4]), int(det_boxe[5])
                            w, h = x2-x1, y2-y1
                            stream_img.draw_rectangle(x1, y1, w, h, color=color_four[det_boxe[0]][1:], thickness=2)
                    if stream_client:
                        try:
                            stream_client.send(BOUNDARY.encode())
                            stream_client.send(stream_img.compress(quality=50))
                        except:
                            stream_client.close(); stream_client = None
                    if should_save_one:
                        save_current_frame(stream_img, current_class_name)
                        should_save_one = False
                    stream_img = None

                # --- Socket 监听命令 ---
                try:
                    conn, addr = s.accept()
                    conn.settimeout(0.1)
                    req = conn.recv(1024).decode()
                    if "GET /stream" in req:
                        if stream_client: stream_client.close()
                        stream_client = conn
                        stream_client.send(MP_HEADER.encode())
                    elif "GET /get_fps" in req:
                        conn.send(f"HTTP/1.1 200 OK\r\n\r\n{current_fps:.1f}".encode()); conn.close()
                    elif "GET /save_cmd" in req:
                        should_save_one = True
                        conn.send("HTTP/1.1 200 OK\r\n\r\nOK".encode()); conn.close()
                    else: conn.close()
                except: pass

            rgb888p_img = None
            gc.collect()

    except Exception as e:
        print(f"Main Loop Error: {e}")
    finally:
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        if 'uart3' in locals() and uart3: uart3.deinit()
        sensor.stop()
        Display.deinit()
        MediaManager.deinit()
        if 's' in locals(): s.close()
        gc.collect()
        time.sleep(1)
        nn.shrink_memory_pool()
    return 0

if __name__=="__main__":
    ensure_dir(DEFAULT_SAVE_PATH)
    detection()