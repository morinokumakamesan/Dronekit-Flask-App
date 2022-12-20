from flask import Flask, render_template
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
from pymavlink import mavutil
import math

app = Flask(__name__)

# 方角リスト作成
direction = ['北','東','南','西']

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/getmethod/<voice_keyword>')
def get_javascript_data(voice_keyword):
    print(voice_keyword)
  
    if(voice_keyword == '接続'):
        # 機体と接続
        global vehicle
        vehicle = connect('127.0.0.1:14550', wait_ready=True, baud=115200)
        print("Mode: %s" % vehicle.mode.name)

        # アーミング可能かチェック
        while not vehicle.is_armable:
            print("Waiting for vehicle to initialize...")
            time.sleep(1)

    elif(voice_keyword == 'アーム'):
        # アーミングの実行
        print("Arming motors")
        vehicle.mode = VehicleMode("GUIDED")
        vehicle.armed = True

        while not vehicle.armed:
            print("Waiting for arming...")
            time.sleep(1)

    elif(voice_keyword == 'テイクオフ'):
        # 目標高度の設定
        targetAltitude = 10
        vehicle.simple_takeoff(targetAltitude)

        while True:
            print("Altitude: ", vehicle.location.global_relative_frame.alt)
            if vehicle.location.global_relative_frame.alt >= targetAltitude * 0.95:
                print("Reached target altitude")
                break

            time.sleep(1)

    elif(voice_keyword in direction):
        move(voice_keyword)

    elif(voice_keyword == 'rtl'):
        print("Returning to Launch")
        vehicle.mode = VehicleMode("RTL")

        print("Close vehicle object")
        vehicle.close()

    return voice_keyword

def move(keyword):
    # x,y,z方向への速度（m/s）を指定
    move_direction = {'北': [10, 0, 0],'東':[0, 10, 0],'南':[-10, 0, 0],'西':[0, -10, 0]}
    x_direction = move_direction[keyword][0]
    y_direction = move_direction[keyword][1]
    z_direction = move_direction[keyword][2]

    # MAVLinkメッセージ生成
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 
        0, 0, 
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, 
        0b0000111111000111, 
        0, 0, 0, 
        x_direction, y_direction, z_direction,
        0, 0, 0,
        0, 0)

    # MAVLinkメッセージ送信
    # 1秒おきに3回メッセージを送信（一回の方角指示で30m進む計算）
    for x in range(0, 3):
        vehicle.send_mavlink(msg)
        time.sleep(1)

if __name__ == "__main__":
    app.run(debug=False, port=8888, threaded=False)