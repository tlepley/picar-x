
from robot_hat import Servo
from robot_hat.utils import reset_mcu
from time import sleep

reset_mcu()
sleep(0.2)

if __name__ == '__main__':
    print(f"Set servo to zero")
    for i in range(12):
        print(f"Setting servo {i} set to zero")
        Servo(i).angle(-30)
        sleep(0.1)
        Servo(i).angle(10)
        sleep(0.1)
        Servo(i).angle(0)
        sleep(0.1)
        print(f"DONE: servo {i} set to zero")
