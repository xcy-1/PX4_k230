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
import ustruct             
from machine import UART
from machine import FPIOA

# ================= 用户配置区域 =================
WIFI_SSID = "1234"
WIFI_PASS = "12345789"
PORT = 8080
DEFAULT_SAVE_PATH = "/sdcard/images/"

UART3_TX_PIN = 50
UART3_RX_PIN = 51
UART_BAUDRATE = 115200

# ================= 全局变量 =================
# 【核心修改】用于存储4个不重复标签的数组
history_labels = [] 
current_fps = 0
should_save_one = False
current_class_name = "default"

# ================= 网页前端代码 =================
HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <title>K230 History Recorder</title>
    <style>
        body { background: #111; color: white; font-family: sans-serif; text-align: center; }
        .container { display: flex; flex-wrap: wrap; justify-content: center; }
        .video-box { border: 2px solid #444; margin: 10px; }
        .data-box { border: 2px solid #444; padding: 20px; margin: 10px; width: 300px; text-align: left; }
        
        /* 4个文本框的样式 */
        .label-input { 
            width: 100%; 
            padding: 10px; 
            margin-bottom: 10px; 
            background: #222; 
            color: #0f0; 
            border: 1px solid #555; 
            font-size: 18px; 
            box-sizing: border-box;
        }
        
        h3 { margin-top: 0; color: #aaa; }
        #status { color: yellow; font-size: 14px; margin-top: 10px;}
        .btn { padding: 10px 20px; background: blue; color: white; border: none; cursor: pointer; width: 100%; }
        .btn-clear { background: #c00; margin-top: 10px; }
    </style>
</head>
<body>
    <h1>K230 Object Recorder</h1>
    
    <div class="container">
        <div class="video-box">
            <img src="/stream" style="width:640px; max-width:100%;" />
        </div>
        
        <div class="data-box">
            <h3>Detected History (Max 4)</h3>
            
            <input type="text" id="box0" class="label-input" readonly value="-">
            <input type="text" id="box1" class="label-input" readonly value="-">
            <input type="text" id="box2" class="label-input" readonly value="-">
            <input type="text" id="box3" class="label-input" readonly value="-">
            
            <div id="status">Waiting for update...</div>
            <br>
            <button class="btn" onclick="fetch('/save_cmd')">Save Snapshot</button>
            <button class="btn btn-clear" onclick="fetch('/clear_history')">Clear History</button>
        </div>
    </div>

    <script>
        async function updateLoop() {
            try {
                // 【核心修改】请求列表数据
                let response = await fetch('/get_labels?t=' + new Date().getTime());
                if (response.ok) {
                    let data = await response.json();
                    
                    document.getElementById('status').innerText = "Updated at " + new Date().toLocaleTimeString();
                    document.getElementById('status').style.color = "#0f0";
                    
                    // 清空所有框
                    for(let i=0; i<4; i++) {
                         document.getElementById('box'+i).value = "-";
                    }

                    // 填入数据 (有多少填多少)
                    let labels = data.history;
                    for(let i=0; i<labels.length && i<4; i++) {
                        document.getElementById('box'+i).value = (i+1) + ". " + labels[i];
                    }
                }
            } catch (error) {
                console.log("Connection lost...");
                document.getElementById('status').innerText = "Connection lost...";
                document.getElementById('status').style.color = "red";
            }
            
            // 【核心修改】5000毫秒 (5秒) 后再次执行
            setTimeout(updateLoop, 5000); 
        }
        
        // 启动循环
        setTimeout(updateLoop, 2000);
    </script>
</body>
</html>
"""

# 显示配置
display_mode="lcd"
if display_mode=="lcd":
    DISPLAY_WIDTH = ALIGN_UP(640, 16)
    DISPLAY_HEIGHT = 480
else:
    DISPLAY_WIDTH = ALIGN_UP(1920, 16)
    DISPLAY_HEIGHT = 1080

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
    print(f"Connected! Access address: http://{ip}:{PORT}")
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
    print(f"Saved: {file_name}")

def read_deploy_config(config_path):
    with open(config_path, 'r') as json_file:
        return ujson.load(json_file)

def send_uart_packet(uart, has_obj, class_id, cx, cy, area):
    if uart is None: return
    try:
        packet = ustruct.pack(">BBBBHHIB", 0xAA, 0x55, has_obj, class_id, cx, cy, area, 0xED)
        uart.write(packet)
    except Exception as e:
        print("UART Send Error:", e)

# ================= 主逻辑 =================
def detection():
    global current_class_name, should_save_one, current_fps, history_labels

    # 1. 初始化串口
    try:
        fpioa = FPIOA()
        fpioa.set_function(UART3_TX_PIN, FPIOA.UART3_TXD)
        fpioa.set_function(UART3_RX_PIN, FPIOA.UART3_RXD)
        uart3 = UART(UART.UART3, baudrate=UART_BAUDRATE, bits=UART.EIGHTBITS, parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)
        print("UART3 Ready.")
    except Exception as e:
        print(f"UART3 INIT ERROR: {e}")
        uart3 = None

    # 2. 连接 WiFi
    ip = connect_wifi()
    if not ip: ip_display_text = "WiFi Disconnected"
    else: ip_display_text = f"http://{ip}:{PORT}"
    
    # 3. Socket 初始化
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('0.0.0.0', PORT))
    s.listen(5)
    s.setblocking(False) 
    stream_client = None

    # 4. AI 模型配置
    print("Loading Model...")
    deploy_conf=read_deploy_config(config_path)
    kmodel_name=deploy_conf["kmodel_path"]
    labels_list=deploy_conf["categories"]
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

    # Padding
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

    # 5. 摄像头与显示初始化
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

    osd_img = image.Image(DISPLAY_WIDTH, DISPLAY_HEIGHT, image.ARGB8888)
    
    try:
        MediaManager.init()
        sensor.run()
        
        rgb888p_img = None
        stream_img = None
        data = np.ones((1,3,width,height),dtype=np.uint8)
        ai2d_output_tensor = nn.from_numpy(data)
        
        clock = time.clock()
        frame_counter = 0 
        
# ... (前面的代码保持不变) ...
        
        # 定义一个假数据用于测试，确保不是空列表问题 (你可以稍后删掉这一行)
        # history_labels = ["Test_Item"] 

        print("Loop Start...")
        
        while True:
            clock.tick()
            frame_counter += 1
            osd_img.clear() 
            
            rgb888p_img = sensor.snapshot(chn=CAM_CHN_ID_2)
            
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

                det_boxes = []
                if model_type == "AnchorBaseDet":
                    det_boxes = aicube.anchorbasedet_post_process(results[0], results[1], results[2], kmodel_frame_size, frame_size, strides, num_classes, confidence_threshold, nms_threshold, anchors, nms_option)
                elif model_type == "GFLDet":
                    det_boxes = aicube.gfldet_post_process(results[0], results[1], results[2], kmodel_frame_size, frame_size, strides, num_classes, confidence_threshold, nms_threshold, nms_option)
                else:
                    det_boxes = aicube.anchorfreedet_post_process(results[0], results[1], results[2], kmodel_frame_size, frame_size, strides, num_classes, confidence_threshold, nms_threshold, nms_option)
                
                # --- 数据处理逻辑 ---
                if det_boxes:
                    for det_boxe in det_boxes:
                        cls_id = int(det_boxe[0])
                        # 确保 label_name 是字符串
                        label_name = str(labels_list[cls_id])
                        
                        # 【调试打印】看看是不是真的识别到了
                        # print(f"Detected: {label_name}") 
                        
                        # 数组缓存逻辑
                        if label_name not in history_labels:
                            if len(history_labels) < 4:
                                history_labels.append(label_name)
                                print(f">>> ADDED TO HISTORY: {label_name}") # 看到这个打印才说明存进去了
                        
                        current_class_name = label_name 

                        # 坐标计算 & 串口
                        x1, y1, x2, y2 = int(det_boxe[2]), int(det_boxe[3]), int(det_boxe[4]), int(det_boxe[5])
                        w, h = x2 - x1, y2 - y1
                        cx, cy = int((x1+x2)/2), int((y1+y2)/2)
                        if uart3: send_uart_packet(uart3, 1, cls_id, cx, cy, w*h)

                        # 画框
                        x_disp = int(x1 * DISPLAY_WIDTH // OUT_WIDTH)
                        y_disp = int(y1 * DISPLAY_HEIGHT // OUT_HEIGHT)
                        w_disp = int(w * DISPLAY_WIDTH // OUT_WIDTH)
                        h_disp = int(h * DISPLAY_HEIGHT // OUT_HEIGHT)
                        draw_color = color_four[cls_id] 
                        osd_img.draw_rectangle(x_disp, y_disp, w_disp, h_disp, color=draw_color, thickness=4)
                        osd_img.draw_string_advanced(x_disp, max(0, y_disp-30), 32, label_name, color=draw_color)
                
                else:
                    if uart3: send_uart_packet(uart3, 0, 0, 0, 0, 0)
                
                current_fps = clock.fps()
                osd_img.draw_string_advanced(10, 10, 32, f"FPS: {current_fps:.1f}", color=(255, 0, 255, 0))
                osd_img.draw_string_advanced(10, 50, 32, f"IP: {ip_display_text}", color=(255, 0, 0, 255))
                Display.show_image(osd_img, 0, 0, Display.LAYER_OSD3)

                # --- 网络处理 (核心修复部分) ---
                try:
                    conn, addr = s.accept()
                    conn.settimeout(0.1)
                    req = conn.recv(1024).decode()
                    
                    if req:
                        first_line = req.splitlines()[0]
                        
                        if "GET /get_labels" in first_line:
                            resp = {"history": history_labels}
                            json_body = ujson.dumps(resp)
                            # 【修复】必须加上 Content-Length，否则浏览器会一直等到超时
                            header = (
                                "HTTP/1.1 200 OK\r\n"
                                "Content-Type: application/json\r\n"
                                "Content-Length: {}\r\n"
                                "Connection: close\r\n"
                                "Access-Control-Allow-Origin: *\r\n\r\n"
                            ).format(len(json_body))
                            
                            conn.sendall(header.encode() + json_body.encode())
                            conn.close()
                            # print(f"Sent JSON: {json_body}") # 打开这个查看是否发送了数据
                        
                        elif "GET /clear_history" in first_line:
                            history_labels = []
                            conn.send("HTTP/1.1 200 OK\r\nContent-Length: 0\r\n\r\n".encode())
                            conn.close()
                            print("History Cleared")

                        elif "GET /stream" in first_line:
                            if stream_client: stream_client.close()
                            stream_client = conn
                            stream_client.send("HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n".encode())

                        elif "GET /save_cmd" in first_line:
                            should_save_one = True
                            conn.send("HTTP/1.1 200 OK\r\nContent-Length: 0\r\n\r\n".encode())
                            conn.close()

                        elif "GET / " in first_line:
                            # 发送 HTML 页面
                            conn.send("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n".encode())
                            conn.sendall(HTML_PAGE.encode())
                            conn.close()
                        else:
                            conn.close()
                except OSError: 
                    pass 

                # --- 视频推流 (降低频率) ---
                if (stream_client or should_save_one) and (frame_counter % 6 == 0):
                    try:
                        stream_img = sensor.snapshot(chn=CAM_CHN_ID_1)
                        if det_boxes: 
                             for det_boxe in det_boxes:
                                x1, y1, x2, y2 = int(det_boxe[2]), int(det_boxe[3]), int(det_boxe[4]), int(det_boxe[5])
                                stream_img.draw_rectangle(x1, y1, x2-x1, y2-y1, color=(0,255,0), thickness=2)
                        
                        if stream_client:
                            stream_client.send("\r\n--frame\r\nContent-Type: image/jpeg\r\n\r\n".encode())
                            stream_client.send(stream_img.compress(quality=40))
                        
                        if should_save_one:
                            save_current_frame(stream_img, current_class_name)
                            should_save_one = False
                    except:
                        if stream_client: stream_client.close(); stream_client = None

            rgb888p_img = None
            gc.collect()

    except Exception as e:
        print(f"Main Loop Error: {e}")
    finally:
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