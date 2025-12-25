import utime, ubinascii, ujson, usocket, _thread, network, gc
from media.sensor import *
from media.display import *
from media.media import *
# 全局变量
img = None  # 拍摄到的照片
imgLock = _thread.allocate_lock()  # 图像锁
RunCamera = True  # 线程退出标志
ssCNT = 0  # 帧计数
gc_threshold = 200  # 每发送200张图像进行一次垃圾回收

SSID = '1234'
PASSWORD = '12345789'

def network_use_wlan(is_wlan=True):
    if is_wlan:
        sta=network.WLAN(0)
        sta.connect(SSID, PASSWORD)
        print(sta.status())
        while sta.ifconfig()[0] == '0.0.0.0':
            os.exitpoint()
        print(sta.ifconfig())
        ip = sta.ifconfig()[0]
        return ip
    else:
        a=network.LAN()
        if not a.active():
            raise RuntimeError("LAN interface is not active.")
        a.ifconfig("dhcp")
        print(a.ifconfig())
        ip = a.ifconfig()[0]
        return ip

# HTTP服务
def http_server():
    global img, imgLock, RunCamera, ssCNT, gc_threshold

    # 连接网络，获取IP地址
    IPaddress = network_use_wlan()
    Port = 80
    ai = usocket.getaddrinfo(IPaddress, Port)
    addr = ai[0][-1]
    print(f'\tCreate HTTP server listen at {IPaddress}:{Port}\n\n')

    # 创建 HTTP 服务器
    s = usocket.socket()
    s.setsockopt(usocket.SOL_SOCKET, usocket.SO_REUSEADDR, 1)
    s.bind(addr)
    s.listen(5)

    while RunCamera:
        try:
            cl, addr = s.accept()
            print(f"Client connected from {addr}")
            request = cl.recv(1024)
            if not request:
                continue
            request_str = request.decode()
            header = "HTTP/1.1 200 OK\r\n" \
                     "Server: Tao\r\n" \
                     "Content-Type: multipart/x-mixed-replace;boundary=Tao\r\n" \
                     "Cache-Control: no-cache\r\n" \
                     "Pragma: no-cache\r\n\r\n"
            cl.send(header.encode())

            while RunCamera:
                try:
                    if imgLock.acquire(1, 1):  # 申请img变量的锁，阻塞1秒
                        if img is not None:  # 检查img是否为None
                            img_bytes = img.compress(quality=50)
                            imgLock.release()
                            header = f"--Tao\r\nContent-Type: image/jpeg\r\nContent-Length: {len(img_bytes)}\r\n\r\n"
                            cl.send(header.encode())
                            cl.send(img_bytes)

                            del img_bytes
                            if ssCNT % gc_threshold == 0:
                                gc.collect()
                                print(f"Garbage collected at frame {ssCNT}")
                        else:
                            imgLock.release()
                            print('img变量为None')
                    else:
                        print('img变量锁申请超时')
                except Exception as e:
                    if e.errno == 11:
                        print('\t-----Error 11-----')
                        break
                    else:
                        print(f"\n\tHTTP错误：{e}")
                        break
                utime.sleep_ms(100)  # 限制帧率不超过10，给其它线程留下CPU时间
                ssCNT += 1  # 帧计数
        except Exception as e:
            print(f"\n\t建立HTTP服务时出错：{e}")
            cl.close()
            s.close()
            ap.active(False)
            utime.sleep(5)  # 等5秒，重启WiFi
            s = usocket.socket(usocket.AF_INET, usocket.SOCK_STREAM)
            s.bind(addr)
            s.listen(5)
        finally:
            cl.close()
            utime.sleep(1)  # 等1秒，确保所有数据都已发送完毕

# 拍摄
def th_Camera():
    global img, imgLock, RunCamera, ssCNT

    cam = Sensor()  # 默认摄像头2，一共支持3个摄像头
    cam.reset()
    cam.set_framesize(width=640, height=360, chn=CAM_CHN_ID_0)  # 每个摄像头有3个通道，我们使用ch0
    cam.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)  # 目前还不支持rgb888转换到jpg格式

    Display.init(Display.ST7701, to_ide=True, osd_num=2)
    MediaManager.init()
    cam.run()

    clock = time.clock()
    fps = 0

    while RunCamera:
        clock.tick()
        if imgLock.acquire(1, 1):  # 申请变量img的锁，阻塞1秒
            del img
            gc.collect()
            img = cam.snapshot()
            # 只显示帧数，不显示FPS
            img.draw_string_advanced(5, 5, 36, f'{ssCNT}', color=(255, 0, 0))
            imgLock.release()
        utime.sleep_ms(50)  # 为保证数据及时刷新，2倍于推送帧率
        fps = clock.fps()
        # 在控制台打印FPS
        print(f'当前FPS: {fps:.1f}')
    cam.stop()
    utime.sleep_ms(100)
    MediaManager.deinit()

# 显示
def th_Display():
    global img, imgLock, RunCamera

    while img == None and RunCamera == True:  # 死等摄像头启动后赋值给img
        utime.sleep_ms(100)

    while RunCamera:
        if imgLock.acquire(0, 0):  # 申请变量img的锁，无阻塞
            display = Display()
            display.show_image(img, x=80, y=60)  # 显示图片
            imgLock.release()
        utime.sleep_ms(100)

if __name__ == "__main__":
    RunCamera = True  # 线程退出条件，比如按Key键3秒后修改此值即可关闭程序
    imgLock = _thread.allocate_lock()  # 线程锁实例

    _thread.start_new_thread(th_Camera, ())   # 摄像头线程
    _thread.start_new_thread(th_Display, ())  # 显示线程
    _thread.start_new_thread(http_server, ()) # 推流线程

    while RunCamera:
        utime.sleep(1)
