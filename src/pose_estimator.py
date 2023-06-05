"""
カメラから体の姿勢を推定
"""

import cv2
import mediapipe as mp
import math
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_pose = mp.solutions.pose

##アドレス登録
from socket import socket,AF_INET,SOCK_DGRAM
HOST=""
PORT=5000
SERVER="192.168.0.111"
sock=socket(AF_INET,SOCK_DGRAM)
sock.bind((HOST,PORT))
## 

# Webカメラから入力
cap = cv2.VideoCapture(0)
with mp_pose.Pose(
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5) as pose:
  while cap.isOpened():

    LCOS=None

    try:
        success, image = cap.read()
        if not success:
            print("Ignoring empty camera frame.")
            continue
        image.flags.writeable = False
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = pose.process(image)

        # 腕の角度を計算 cosを出力
        if results.pose_landmarks is None :
            print('off screen')     
        else:
            RShoulder = (results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_SHOULDER].x, results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_SHOULDER].y)
            RElbow = (results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_ELBOW].x, results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_ELBOW].y)
            RHip = (results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_HIP].x, results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_HIP].y) 
            RSE = math.sqrt((RShoulder[0] - RElbow[0])*(RShoulder[0] - RElbow[0]) + (RShoulder[1] - RElbow[1])*(RShoulder[1] - RElbow[1]))
            REH = math.sqrt((RElbow[0] - RHip[0])*(RElbow[0] - RHip[0]) + (RElbow[1] - RHip[1])*(RElbow[1] - RHip[1]))
            RHS = math.sqrt((RHip[0] - RShoulder[0])*(RHip[0] - RShoulder[0]) + (RHip[1] - RShoulder[1])*(RHip[1] - RShoulder[1]))
            RCOS = (RSE*RSE + RHS*RHS - REH*REH) / (2*RSE*RHS)
            LShoulder = (results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_SHOULDER].x, results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_SHOULDER].y)
            LElbow = (results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_ELBOW].x, results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_ELBOW].y)
            LHip = (results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_HIP].x, results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_HIP].y)
            LSE = math.sqrt((LShoulder[0] - LElbow[0])*(LShoulder[0] - LElbow[0]) + (LShoulder[1] - LElbow[1])*(LShoulder[1] - LElbow[1]))
            LEH = math.sqrt((LElbow[0] - LHip[0])*(LElbow[0] - LHip[0]) + (LElbow[1] - LHip[1])*(LElbow[1] - LHip[1]))
            LHS = math.sqrt((LHip[0] - LShoulder[0])*(LHip[0] - LShoulder[0]) + (LHip[1] - LShoulder[1])*(LHip[1] - LShoulder[1]))
            LCOS = (LSE*LSE + LHS*LHS - LEH*LEH) / (2*LSE*LHS)
            print('Rcos', RCOS,'    Lcos', LCOS)
            

        # 検出されたポーズの骨格をカメラ画像に重ねて描画
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        mp_drawing.draw_landmarks(
            image,
            results.pose_landmarks,
            mp_pose.POSE_CONNECTIONS,
            landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())
        cv2.imshow('MediaPipe Pose', cv2.flip(image, 1))
        if cv2.waitKey(5) & 0xFF == 27:
            break

        
        if not LCOS is None:
           ##送信
            msg=f"{RCOS},{LCOS}"
            sock.sendto(msg.encode(),(SERVER,PORT))
            msg,address=sock.recvfrom(65535) #OKメールをまつ
            print(msg)
            ##


    except KeyboardInterrupt:
       sock.close() #ソケット閉じる
       break
cap.release()