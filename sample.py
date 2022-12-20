from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

global vehicle
# 921600 115200 57600
# vehicle = connect('udp:192.168.4.2:14550', wait_ready=True, baud=115200)
# vehicle = connect('udp:0.0.0.0:14550', wait_ready=False, baud=115200)
# vehicle = connect('0.0.0.0:14550', wait_ready=False, baud=115200)
vehicle = connect('tcp:127.0.0.1:5762', wait_ready=False)
# Use returned Vehicle object to query device state - e.g. to get the mode:
print("Mode: %s" % vehicle.mode.name)
print(vehicle.is_armable)

# vehicle.mode = VehicleMode("STABILIZE")
# vehicle.armed = True

while not vehicle.is_armable:
    print("Waiting for vehicle to initialize...")
    time.sleep(1)

print("   Supports direct actuator control: %s" % vehicle.capabilities.set_actuator_target)

#アーミング実行
#フライトモードを「GUIDED」に変更し、armedにTrueを設定する
print("Arming motors")
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True

#アーミングが完了するまで待機
while not vehicle.armed:
    print("Waiting for arming...")
    time.sleep(1)

#目標高度を設定
targetAltitude = 20
#テイクオフ実行
#20メートルの高さまで離陸する
print("Take off!!")
vehicle.simple_takeoff(targetAltitude)

while True:
    print("Altitude: ", vehicle.location.global_relative_frame.alt)
    if vehicle.location.global_relative_frame.alt >= targetAltitude * 0.95:
        print("Reached target altitude")
        break

    time.sleep(1)

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)


    # send command to vehicle on 1 Hz cycle
    for x in range(0, 10):
        vehicle.send_mavlink(msg)
        time.sleep(1)

# #MAVLinkメッセージを生成する
# msg = vehicle.message_factory.set_position_target_local_ned_encode(
#     0, #ブートからの時間(今回は未使用)
#     0, 0, #ターゲットシステム、コンポーネント
#     mavutil.mavlink.MAV_FRAME_LOCAL_NED, #フレーム
#     0b0000111111000111, # タイプマスク、0:有効、1:無効
#     0, 0, 0, # x、y、z位置(今回は未使用)
#     0, 10, 0, # 速度 m/s
#     0, 0, 0, #x、y、z加速度(未サポート)
#     0, 0) #ヨー、ヨーレート

# #MAVLinkメッセージ送信
# for x in range(0, 10):
#     vehicle.send_mavlink(msg)
#     print('進んでいるはず')
#     time.sleep(1)