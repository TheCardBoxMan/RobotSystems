import picarx_improved as pc
import time

car=pc.Picarx()
vel=0.125
dir_min=-30
dir_max=30


def move (direct, dist, angle):
    if angle > dir_max:
        angle = dir_max
    if angle < dir_min:
        angle = dir_min

    car.set_dir_servo_angle(angle)
    speed = 50
    wait = dist * vel

    if direct == "f":
        car.forward(speed)
    else:
        car.backward(speed)

    time.sleep(wait)
    car.stop()

def parallel_park(direct):
    if direct == "r":
        move("b",5,30)
        move('b',1,0)
        move("b",5,-30)
        move("f",2,0)
    else:
        move("b",5,-30)
        move('b',1,0)
        move("b",5,30)
        move("f",2,0)

if __name__ == "__main__":
    action=input("1: Move forwards or backwards\n2: Parallel park\n3: K turn\n Selection: ")
    
    if action == "1":
        direct = input("Forward (f) or Backward (b): ")
        dist = input("Distance in inches: ")
        angle = input("Angle: ")
        move(direct,dist,angle)
    elif action == "2":
        direct=input("Left (l) or Right (r) Parking: ")
        parallel_park(direct)
    elif action == "3":
        print("Not implemented")