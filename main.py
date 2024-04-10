import sensor
import image
import lcd
import KPU as kpu
import time
import openmv_numpy as np1
from kalman_filter import Tracker_Manager
from machine import UART,Timer
from fpioa_manager import fm

#映射串口引脚
fm.register(4, fm.fpioa.UART1_RX, force=True)
fm.register(5, fm.fpioa.UART1_TX, force=True)
#初始化串口
uart = UART(UART.UART1, 115200, read_buf_len=4096)

lcd.init()
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_hmirror(True)
sensor.set_vflip(True)
sensor.set_windowing((224, 224))
clock = time.clock()
sensor.run(1)
task = kpu.load(0x500000)
anchor = (0.1753, 0.4166, 0.5347, 1.2673, 1.1765, 2.8969, 2.3182, 4.5694, 5.0335, 5.1513)
a = kpu.init_yolo2(task, 0.4, 0.3, 5, anchor)


A = np1.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]])
H_k = np1.eye(4)
Q = np1.eye(4, value=0.1)
R = np1.eye(4)
B = None
Manager = Tracker_Manager()

# list 与蓝色polygon重叠
list_overlapping_blue_polygon = []
# list 与黄色polygon重叠
list_overlapping_yellow_polygon = []
# 计算每个ID从经过两条触发线的时间
list_blue_time = {}
list_yellow_time = {}
# 进入数量
down_count = 0
# 离开数量
up_count = 0

global_up = 0
global_down = 0
while(True):
    clock.tick()
    img = sensor.snapshot()
    code = kpu.run_yolo2(task, img)
    a = img.draw_rectangle(70, 0, 30, 224, color=1, thickness=2, fill=True)
    a = img.draw_rectangle(140, 0, 30, 224, color=255, thickness=2, fill=True)
    if code:
        for i in code:
            a=img.draw_rectangle(i.rect(),(0,255,0),2)
            x = i.rect()[0]+i.rect()[2]*0.5
            y = i.rect()[1]+i.rect()[3]*0.5
            img.draw_circle(int(x), int(y), 2, color=(255, 255, 255), thickness=2, fill=True)
            Manager.match(int(x),int(y),A,H_k,Q,R)
            lcd.draw_string(i.x(), i.y(), 'person', lcd.RED, lcd.WHITE)
    Manager.update()
    trails_pre = Manager.get_motion_trail_pre()

    for ID, trail in trails_pre:
        if len(trail):
            x, y = trail[0][0], trail[0][1]
            #print(a.get_pixel(120,70)[1])
            #print(a.get_pixel(60,70)[1])
            # 撞线检测点，(x1，y1)
            if img.get_pixel(x,y) is not None:

                if img.get_pixel(x,y)[1] == 32:
                # 如果撞 蓝polygon
                    now_time = time.ticks_ms()
                    if ID not in list_overlapping_blue_polygon:
                        list_overlapping_blue_polygon.append(ID)
                        list_blue_time[ID] = now_time
                    # 判断 黄polygon list 里是否有此 track_id
                    # 有此 track_id，则 认为是 外出方向
                    if ID in list_overlapping_yellow_polygon:
                        # 外出+1
                        up_count += 1
                        # 删除 黄polygon list 中的此id
                        list_overlapping_yellow_polygon.remove(ID)

                        host_time = (now_time - list_yellow_time[ID])
                        uart.write('out_time '+str(host_time)+' ') #数据回传
                        list_yellow_time.pop(ID)
                        pass
                    else:
                        # 无此 track_id，不做其他操作
                        pass
                elif img.get_pixel(x,y)[1] == 227:
                # 如果撞 黄polygon
                    now_time = time.ticks_ms()
                    if ID not in list_overlapping_yellow_polygon:
                        list_overlapping_yellow_polygon.append(ID)
                        list_yellow_time[ID] = now_time
                    # 判断 蓝polygon list 里是否有此 track_id
                    #  有此 track_id，则 认为是 进入方向
                    if ID in list_overlapping_blue_polygon:
                        # 进入+1
                        down_count += 1
                        # 删除 蓝polygon list 中的此id
                        list_overlapping_blue_polygon.remove(ID)
                        host_time = (now_time - list_blue_time[ID])
                        uart.write('in_time '+str(host_time)+' ') #数据回传
                        list_blue_time.pop(ID)
                        pass
                    else:
                        # 无此 track_id，不做其他操作
                         pass
                    pass
                else:
                    pass
            a = img.draw_string(x, y, str(ID), color=(0, 60, 128), scale=2.0)
    list_overlapping_all = list_overlapping_yellow_polygon + list_overlapping_blue_polygon
    for id1 in list_overlapping_all:
        is_found = False
        for ID, trail in trails_pre:
            if ID == id1:
                is_found = True
                break
            pass
        pass
        if not is_found:
        # 如果没找到，删除id
            if id1 in list_overlapping_yellow_polygon:
                list_overlapping_yellow_polygon.remove(id1)
                pass
            if id1 in list_overlapping_blue_polygon:
                list_overlapping_blue_polygon.remove(id1)
                pass
        pass
    list_overlapping_all.clear()
    pass
        ## 清空list
        #if len(tracked_boxes) > 0:
            #tracked_boxes.clear()
        # pass
    #else:
        #list_overlapping_blue_polygon.clear()
        #list_overlapping_yellow_polygon.clear()

    # print(down_count,up_count)
    if down_count > global_down:
        uart.write('in_count '+str(down_count))
        global_down = down_count
    if up_count > global_up:
       uart.write('out_count '+str(up_count))
       global_up = up_count
    a = img.draw_string(0, 0, 'In:'+str(down_count)+' '+'Out:'+str(up_count), color=(0, 60, 128), scale=2.0)
    a = lcd.display(img)
a = kpu.deinit(task)

