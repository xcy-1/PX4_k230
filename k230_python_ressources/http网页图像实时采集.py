import network
import socket
import time
import gc
import os
from media.sensor import *
from media.media import *
from machine import Pin


from media.display import Display


# --- 用户配置 ---
WIFI_SSID = "1234"           
WIFI_PASS = "12345789"       
PORT = 80
DEFAULT_SAVE_PATH = "/data/data/images/"

# [配置] VGA + YUV420SP (速度最快)
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# --- 全局变量 ---
current_class_name = "default"
save_counter = 0
should_save_one = False
current_fps = 0  # 实时帧率变量

# --- 硬件初始化 ---
try:
    from machine import FPIOA
    fpioa = FPIOA()
    if hasattr(FPIOA, 'GPIO53'):
        fpioa.set_function(53, FPIOA.GPIO53)
        key_pin = Pin(53, Pin.IN, Pin.PULL_DOWN)
    else:
        key_pin = None
except:
    key_pin = None

def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print(f"正在连接热点: {WIFI_SSID} ...")
        wlan.connect(WIFI_SSID, WIFI_PASS)
        retry = 0
        while not wlan.isconnected():
            time.sleep(1)
            retry += 1
            if retry > 15:
                print("WiFi 连接失败")
                return None
    ip = wlan.ifconfig()[0]
    print(f"连接成功! 访问地址: http://{ip}")
    return ip

def ensure_dir(path):
    try: os.stat(path)
    except: 
        try: os.mkdir(path)
        except: pass

def save_current_frame(img, class_name):
    """保存逻辑"""
    global save_counter
    full_dir = DEFAULT_SAVE_PATH + class_name
    try: os.stat(DEFAULT_SAVE_PATH); 
    except: 
        try: os.mkdir(DEFAULT_SAVE_PATH); 
        except: pass
    try: os.stat(full_dir); 
    except: os.mkdir(full_dir)

    file_name = f"{class_name}_{save_counter}_{time.ticks_ms()}.jpg"
    full_path = f"{full_dir}/{file_name}"
    
    # 截图时使用高质量
    img_data = img.compress(quality=95)
    with open(full_path, 'wb') as f:
        f.write(img_data)
    
    print(f"已保存: {full_path}")
    save_counter += 1
    del img_data

# --- 网页 HTML ---
# 增加了 JS 定时器，每秒请求一次 /get_fps
HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>K230 Ultimate Stream</title>
    <style>
        body { font-family: sans-serif; background: #181818; color: #eee; text-align: center; margin: 0; padding: 10px; }
        .monitor { position: relative; display: inline-block; margin: 10px auto; border: 4px solid #333; }
        #cam_view { display: block; max-width: 100%; width: 640px; background: #000; }
        
        /* FPS 悬浮窗 */
        #fps_box {
            position: absolute; top: 5px; right: 5px;
            background: rgba(0,0,0,0.7); color: #0f0;
            padding: 4px 8px; border-radius: 4px; font-weight: bold; font-size: 16px;
            pointer-events: none;
            min-width: 60px;
            text-align: right;
        }

        .panel { background: #282828; padding: 20px; border-radius: 10px; max-width: 600px; margin: 0 auto; }
        .row { display: flex; gap: 10px; margin-bottom: 15px; }
        input { flex: 1; padding: 10px; background: #444; border: 1px solid #555; color: white; border-radius: 4px;}
        button { padding: 10px 20px; cursor: pointer; border: none; border-radius: 4px; font-weight: bold; color: white;}
        .btn-set { background: #007bff; }
        .btn-shot { background: #dc3545; width: 100%; font-size: 18px; padding: 15px; }
        .btn-shot:active { background: #a71d2a; }
        #log { color: #aaa; margin-top: 10px; font-size: 14px; }
    </style>
</head>
<body>
    <h2>K230 极速推流控制台</h2>
    
    <div class="monitor">
        <img id="cam_view" src="/stream" alt="Waiting for stream..." />
        <div id="fps_box">FPS: --</div>
    </div>

    <div class="panel">
        <div class="row">
            <input type="text" id="cls_name" value="default" placeholder="输入文件夹名称">
            <button class="btn-set" onclick="setPath()">设置路径</button>
        </div>
        <button class="btn-shot" onclick="capture()">📸 立即截屏保存</button>
        <div id="log">系统就绪</div>
    </div>

    <script>
        const logDiv = document.getElementById('log');
        const fpsBox = document.getElementById('fps_box');

        // --- 核心：每秒询问一次 FPS ---
        setInterval(() => {
            fetch('/get_fps')
                .then(response => response.text())
                .then(fps => {
                    fpsBox.innerText = "FPS: " + fps;
                    // 颜色逻辑
                    let n = parseInt(fps);
                    if(n < 10) fpsBox.style.color = 'red';
                    else if(n < 20) fpsBox.style.color = 'orange';
                    else fpsBox.style.color = '#0f0';
                })
                .catch(() => {});
        }, 1000);

        function setPath() {
            const name = document.getElementById('cls_name').value;
            fetch('/set_path?name=' + encodeURIComponent(name))
                .then(r => logDiv.innerText = "路径已设为: " + name);
        }

        function capture() {
            logDiv.innerText = "指令发送中...";
            logDiv.style.color = "yellow";
            fetch('/save_cmd').then(r => {
                logDiv.innerText = "✅ 保存成功!";
                logDiv.style.color = "#0f0";
                setTimeout(() => { logDiv.innerText = "就绪"; logDiv.style.color = "#aaa"; }, 2000);
            }).catch(() => logDiv.innerText = "❌ 保存失败");
        }
    </script>
</body>
</html>
"""

def main():
    ip = connect_wifi()
    if not ip: return

    sensor = Sensor(width=FRAME_WIDTH, height=FRAME_HEIGHT)
    sensor.reset()
    sensor.set_framesize(width=FRAME_WIDTH, height=FRAME_HEIGHT)
    try:
        sensor.set_pixformat(Sensor.YUV420SP)
    except:
        sensor.set_pixformat(Sensor.YUV420)
    
    MediaManager.init()
    sensor.run()

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('0.0.0.0', PORT))
    s.listen(5)
    s.setblocking(False) 

    global current_class_name, should_save_one, current_fps
    stream_client = None 

    print(">>> 服务已启动")

    MP_HEADER = "HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n"
    BOUNDARY = "\r\n--frame\r\nContent-Type: image/jpeg\r\n\r\n"

    # FPS 统计变量
    frame_counter = 0
    last_tick = time.ticks_ms()

    while True:
        try:
            img = sensor.snapshot()
            
            # --- FPS 核心计算逻辑 ---
            frame_counter += 1
            now = time.ticks_ms()
            if time.ticks_diff(now, last_tick) >= 1000:
                current_fps = frame_counter
                frame_counter = 0
                last_tick = now
                # print(f"FPS: {current_fps}") # 调试用

            # 按键与保存
            if (key_pin and key_pin.value() == 1) or should_save_one:
                save_current_frame(img, current_class_name)
                should_save_one = False
                time.sleep(0.1)

            # 推流
            if stream_client:
                try:
                    jpg = img.compress(quality=40)
                    stream_client.send(BOUNDARY.encode())
                    stream_client.send(jpg) 
                except OSError:
                    stream_client.close()
                    stream_client = None
            
            # 处理新请求
            try:
                conn, addr = s.accept()
                conn.settimeout(0.5)
                req = conn.recv(1024).decode()
                line1 = req.split('\n')[0]
                
                if "GET /stream" in line1:
                    if stream_client: stream_client.close()
                    stream_client = conn
                    stream_client.send(MP_HEADER.encode())
                
                elif "GET / " in line1:
                    conn.send("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n".encode())
                    conn.send(HTML_PAGE.encode())
                    conn.close()

                # 新增：FPS 查询接口
                elif "GET /get_fps" in line1:
                    conn.send(f"HTTP/1.1 200 OK\r\nAccess-Control-Allow-Origin: *\r\n\r\n{current_fps}".encode())
                    conn.close()
                
                elif "GET /save_cmd" in line1:
                    should_save_one = True
                    conn.send("HTTP/1.1 200 OK\r\nAccess-Control-Allow-Origin: *\r\n\r\nOK".encode())
                    conn.close()

                elif "GET /set_path" in line1:
                    try: current_class_name = line1.split('name=')[1].split(' ')[0]
                    except: pass
                    conn.send("HTTP/1.1 200 OK\r\n\r\nOK".encode())
                    conn.close()
                else:
                    conn.close()

            except OSError:
                pass

            del img
            # gc.collect() 

        except Exception as e:
            print("Error:", e)
            if stream_client:
                stream_client.close()
                stream_client = None

if __name__ == "__main__":
    try:
        ensure_dir("/data")
        ensure_dir("/data/data")
        ensure_dir(DEFAULT_SAVE_PATH)
        main()
    except Exception as e:
        print("Error:", e)
    finally:
        MediaManager.deinit()