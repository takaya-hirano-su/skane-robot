import sys
ROOTPATH=str("/home/pi/hirano-dev/projects/snake-robot-project")
sys.path.append(ROOTPATH)

from datetime import datetime
import time
from src import SnakeController,middle_position


def main():

    snake_controller=SnakeController() #インスタンス化

    snake_controller.set_initial_position() #蛇をまっすぐにする
    time.sleep(3)

    goal_positions=snake_controller.read_present_position() #今の角度
    snake_controller.torque_enable() #サーボを待機状態にset

    t=0 #経過時間 [μs]
    date_time_now=datetime.now()

    while True:

        try:
            print("goal",goal_positions)
            date_time_prev=date_time_now #1step前の時刻
            date_time_now=datetime.now() #今の時刻
            t+=(date_time_now-date_time_prev).microseconds #経過時間 [μs]

            goal_positions=snake_controller.update_goal_positions(
                t=t,goal_positions=goal_positions,theta_base=middle_position-200
            ) #goal角度の更新
            snake_controller.control(goal_positions=goal_positions) #goal角度まで動かす
            print(t/1000000)
            print("present",snake_controller.read_present_position())

        except KeyboardInterrupt:
            print("\nFinished")
            snake_controller.torque_disable()
            snake_controller.torque_disable()
            break

    snake_controller.torque_disable()
    snake_controller.end_control()
    

if __name__=="__main__":
    main()