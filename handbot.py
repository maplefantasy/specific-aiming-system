from turtle import delay
import cv2
import mediapipe as mp
import math,time,threading
from wlkata_mirobot import WlkataMirobot
arm = WlkataMirobot()

mp_drawing = mp.solutions.drawing_utils         # mediapipe 繪圖方法
mp_drawing_styles = mp.solutions.drawing_styles # mediapipe 繪圖樣式
mp_hands = mp.solutions.hands                   # mediapipe 偵測手掌方法

w=800
h=600
fontFace = cv2.FONT_HERSHEY_SIMPLEX#文字字型
lineType = cv2.LINE_AA             #文字邊框



# 根據兩點的座標，計算角度
def vector_2d_angle(v1, v2):
    v1_x = v1[0]
    v1_y = v1[1]
    v2_x = v2[0]
    v2_y = v2[1]
    try:
        angle_= math.degrees(math.acos((v1_x*v2_x+v1_y*v2_y)/(((v1_x**2+v1_y**2)**0.5)*((v2_x**2+v2_y**2)**0.5))))
    except:
        angle_ = 180
    return angle_

# 根據傳入的 21 個節點座標，得到該手指的角度
def hand_angle(hand_):
    angle_list = []
    # thumb 大拇指角度
    angle_ = vector_2d_angle(
        ((int(hand_[0][0])- int(hand_[2][0])),(int(hand_[0][1])-int(hand_[2][1]))),
        ((int(hand_[3][0])- int(hand_[4][0])),(int(hand_[3][1])- int(hand_[4][1])))
        )
    angle_list.append(angle_)
    # index 食指角度
    angle_ = vector_2d_angle(
        ((int(hand_[0][0])-int(hand_[6][0])),(int(hand_[0][1])- int(hand_[6][1]))),
        ((int(hand_[7][0])- int(hand_[8][0])),(int(hand_[7][1])- int(hand_[8][1])))
        )
    angle_list.append(angle_)
    # middle 中指角度
    angle_ = vector_2d_angle(
        ((int(hand_[0][0])- int(hand_[10][0])),(int(hand_[0][1])- int(hand_[10][1]))),
        ((int(hand_[11][0])- int(hand_[12][0])),(int(hand_[11][1])- int(hand_[12][1])))
        )
    angle_list.append(angle_)
    # ring 無名指角度
    angle_ = vector_2d_angle(
        ((int(hand_[0][0])- int(hand_[14][0])),(int(hand_[0][1])- int(hand_[14][1]))),
        ((int(hand_[15][0])- int(hand_[16][0])),(int(hand_[15][1])- int(hand_[16][1])))
        )
    angle_list.append(angle_)
    # pink 小拇指角度
    angle_ = vector_2d_angle(
        ((int(hand_[0][0])- int(hand_[18][0])),(int(hand_[0][1])- int(hand_[18][1]))),
        ((int(hand_[19][0])- int(hand_[20][0])),(int(hand_[19][1])- int(hand_[20][1])))
        )
    angle_list.append(angle_)
    return angle_list

# 根據手指角度的串列內容，返回對應的手勢名稱
def hand_pos(finger_angle):
    f1 = finger_angle[0]   # 大拇指角度
    f2 = finger_angle[1]   # 食指角度
    f3 = finger_angle[2]   # 中指角度
    f4 = finger_angle[3]   # 無名指角度
    f5 = finger_angle[4]   # 小拇指角度

    # 小於 50 表示手指伸直，大於等於 50 表示手指捲縮
    if f1>=50 and f2>=50 and f3>=50 and f4>=50 and f5>=50:
        return '0'
    if f1<50 and f2<50 and f3<50 and f4<50 and f5<50:
        return '5'

def upper_left(xP, yP):
    global asi,arm1,arm2,p5
    asi=0
    # print('左上')
    mx = 400 - xP
    my = 300 - yP
    vx = mx * 0.08
    vy = my * 0.04
    arm1 = arm1 + vx
    arm2 = arm2 - vy
    if -100<arm1<120 and -40<arm2<70:
        target_angles = {1:arm1,2:arm2}
        arm.set_joint_angle(target_angles)
        p5="無異常"
        asi=0
    else:
        arm1,arm2=17,-11
        target_angles = {1:arm1,2:arm2}
        arm.set_joint_angle(target_angles)
        p5="發生異常,已超出手臂安全值"
        # time.sleep(3)
        asi=0

def upper_right(xP, yP):
    global asi,arm1,arm2,p5
    asi=0
    # print('右上')
    mx = xP - 400
    my = 300 - yP
    vx = mx * 0.08
    vy = my * 0.04
    arm1 = arm1 - vx
    arm2 = arm2 - vy
    if -100<arm1<120 and -40<arm2<70:
        target_angles = {1:arm1,2:arm2}
        arm.set_joint_angle(target_angles)
        p5="無異常"
        asi=0
    else:
        arm1,arm2=17,-11
        target_angles = {1:arm1,2:arm2}
        arm.set_joint_angle(target_angles)
        p5="發生異常,已超出手臂安全值"
        # time.sleep(3)
        asi=0

def lower_left(xP, yP):
    global asi,arm1,arm2,p5
    asi=0
    # print('左下')
    mx = 400 - xP
    my = yP - 300
    vx = mx * 0.08
    vy = my * 0.04
    arm1 = arm1 + vx
    arm2 = arm2 + vy
    if -100<arm1<120 and -40<arm2<70:
        target_angles = {1:arm1,2:arm2}
        arm.set_joint_angle(target_angles)
        p5="無異常"
        asi=0
    else:
        arm1,arm2=17,-11
        target_angles = {1:arm1,2:arm2}
        arm.set_joint_angle(target_angles)
        p5="發生異常,已超出手臂安全值"
        # time.sleep(3)
        asi=0

def lower_right(xP, yP):
    global asi,arm1,arm2,p5
    asi=0
    # print('右下')
    mx = xP - 400
    my = yP - 300
    vx = mx * 0.08
    vy = my * 0.04
    arm1 = arm1 - vx
    arm2 = arm2 + vy
    if -100<arm1<120 and -40<arm2<70:
        target_angles = {1:arm1,2:arm2}
        arm.set_joint_angle(target_angles)
        p5="無異常"
        asi=0
    else:
        arm1,arm2=17,-11
        target_angles = {1:arm1,2:arm2}
        arm.set_joint_angle(target_angles)
        p5="發生異常,已超出手臂安全值"
        # time.sleep(3)
        asi=0

def opencv0():
    global asi,arm,arm1,arm2,arm3,arm4,arm5,arm6,p5
    p5="無異常"
    asi = 0
    cam = cv2.VideoCapture(1)
    mpHands = mp.solutions.hands
    hands = mpHands.Hands()
    mpDraw = mp.solutions.drawing_utils
    handLmsStyle = mpDraw.DrawingSpec(color=(0,0,255),thickness=3)
    handConStyle = mpDraw.DrawingSpec(color=(255,255,255),thickness=2)
    pTime = 0
    cTime = 0
    time1 = time.time()

    while True:
        xlist=[]
        ylist=[]
        ret, img= cam.read()
        img = cv2.resize(img,(w,h))
        p1,p2,p4="丟失","丟失","no data"
        if ret:
            imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) #BGR轉RGB
            result = hands.process(imgRGB) #將圖片帶入偵測方法內
            if result.multi_hand_landmarks:
                for handLms in result.multi_hand_landmarks:
                    mpDraw.draw_landmarks(img,handLms,mpHands.HAND_CONNECTIONS,handLmsStyle,handConStyle)
                    finger_points = []
                    for i,lm in enumerate(handLms.landmark):
                        xPos = round(lm.x,2)
                        yPos = round(lm.y,2)
                        #選定範圍框
                        xP = int(lm.x*w)
                        yP = int(lm.y*h)
                        p2=f"{xP},{yP}"
                        # print(f'{xP},{yP}')
                        if i == 9 :
                            TxPos=xPos
                            TyPos=yPos
                            if 0.45<TxPos<0.55 and 0.45<TyPos<0.55 :
                                cv2.putText(img,f"success",(0,300),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),2)
                                p1="確認"
                            else:
                                p1="位置修正中"
                                if time.time() - time1 > 1 and asi==0:
                                    time1 = time.time()
                                    asi=1
                                    if xP < 400 and yP < 300:
                                        thread_1 = threading.Thread(target=upper_left(xP ,yP))  # 例項化一個執行緒物件，使執行緒執行這個函式
                                        thread_1.start()  # 啟動這個執行緒
                                    if xP >= 400 and yP < 300:
                                        thread_1 = threading.Thread(target=upper_right(xP ,yP))
                                        thread_1.start()
                                    if xP < 400 and yP >= 300:
                                        thread_1 = threading.Thread(target=lower_left(xP ,yP))
                                        thread_1.start()
                                    if xP >= 400 and yP >= 300:
                                        thread_1 = threading.Thread(target=lower_right(xP ,yP))
                                        thread_1.start()
                for hand_landmarks in result.multi_hand_landmarks:
                    finger_points = []                   # 記錄手指節點座標的串列
                    for i in hand_landmarks.landmark:
                        # 將 21 個節點換算成座標，記錄到 finger_points
                        x = i.x*w
                        y = i.y*h
                        finger_points.append((x,y))
                    if finger_points:
                        finger_angle = hand_angle(finger_points) # 計算手指角度，回傳長度為 5 的串列
                        #print(finger_angle)                     # 印出角度 ( 有需要就開啟註解 )
                        # text = hand_pos(finger_angle)            # 取得手勢所回傳的內容
                        # cv2.putText(img, text, (30,120), fontFace, 5, (255,0,0), 10, lineType) # 印出文字
            # if xlist != [] or ylist != [] :
            #     mx,my,lx,ly = max(xlist),max(ylist),min(xlist),min(ylist)
            #     cv2.rectangle(img, (lx, ly), (mx, my), (0, 255, 255), 2)
            #     if hand_pos(finger_angle) == "5":
            #         p3 = f"{int((((mx-lx)**2)+((my-ly)**2))**0.5)}"
            #         # cv2.putText(img,z,(lx, ly-20),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,100,0),2)
            #     else:
            #         # cv2.putText(img,"condition error",(lx, ly-20),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),2)
            #         p3 = "手勢錯誤"

            cTime = time.time()
            fps = 1/(cTime-pTime)
            pTime = cTime
            p4 = f"{int(fps)}"
            cv2.rectangle(img, (360, 270), (440, 330), (0, 255, 0), 1)
            img = cv2.line(img, (0,300), (800,300), (0, 255, 0), 1)
            img = cv2.line(img, (400,0), (400,600), (0, 255, 0), 1)
            #4大區塊中心位
            img = cv2.line(img, (200,150), (200,150), (0, 255, 0), 3) #左上 200,150
            img = cv2.line(img, (600,150), (600,150), (0, 255, 0), 3) #右上 600,150
            img = cv2.line(img, (200,450), (200,450), (0, 255, 0), 3) #左下 200,450
            img = cv2.line(img, (600,450), (600,450), (0, 255, 0), 3) #右下 600 450

            cv2.imshow('img',img)
        if cv2.waitKey(1) == 32:
            break
        p3=f'[軸一:{int(arm1)},軸二:{int(arm2)}]'
        lb2.config(text=f"手指九號點位置:{p1}")
        lb3.config(text=f"目前位置:{p2}")
        lb4.config(text=f"各軸距位:{p3}")
        lb5.config(text=f"當前FPS:{p4}")
        lb6.config(text=f"移動狀況:{p5}")
        win.update()
arm.home()
target_angles = {1:17, 2:-11, 3:-20, 4:38, 5:-73, 6:0}
arm.set_joint_angle(target_angles)
time.sleep(1)
arm1=arm.angle.joint6
arm2=arm.angle.joint5
arm3=arm.angle.joint4
arm4=arm.angle.joint3
arm5=arm.angle.joint2
arm6=arm.angle.joint1


import tkinter as tk
#主程式
win = tk.Tk()
win.title("總數據視窗")
win.resizable
win.geometry("400x200")

win.iconbitmap("./001.ico")
win.config(bg="#CAFFFF")     #背景顏色
win.attributes("-alpha",0.95,"-topmost",1)#透明度

#label
lb1 = tk.Label(bg="white",text="總數據顯示")
lb1.pack()
lb2 = tk.Label(bg="white",text="手指九號點位置:丟失")
lb2.pack()
lb3 = tk.Label(bg="white",text="目前位置:丟失")
lb3.pack()
lb4 = tk.Label(bg="white",text="各軸距位:no data")
lb4.pack()
lb5 = tk.Label(bg="white",text="當前FPS:no data")
lb5.pack()
lb6 = tk.Label(bg="white",text="移動狀況:無異常")
lb6.pack()
opencv0()