#!/usr/bin/env python3
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, OUTPUT_D, MoveTank, SpeedPercent, MediumMotor
from ev3dev2.sensor import INPUT_1, INPUT_4, INPUT_2
from ev3dev2.sensor.lego import TouchSensor, ColorSensor
from time import sleep

# RGB:
#
# bialy: 255 255 255 / 255 255 255
# czarny: 34 38 25 / 36 45 30
# pol na pol: 174 255 255 / 218 235 243
# czerwony: 255 47 25 / 255 48 30
# niebieski: 50 103 167 / 47 110 179
# zielony: 40 127 55 / 37 132 60


class Robot():
    def __init__(self) -> None:
        self.left_motor = LargeMotor(OUTPUT_D)
        self.right_motor = LargeMotor(OUTPUT_A)
        # TODO: podlacz podnosnik
        self.lifter = MediumMotor(OUTPUT_B)
        self.drive_tank = MoveTank(OUTPUT_A, OUTPUT_D)
        self.left_sensor = ColorSensor(INPUT_1)
        self.right_sensor = ColorSensor(INPUT_4)
        self.button = TouchSensor(INPUT_2)
        self.white_left = 255
        self.white_right = 255
        self.black_left = [34, 36, 28]
        self.black_right = [31, 35, 28]
        self.range_left = self.white_left - self.black_left[0]
        self.range_right = self.white_right - self.black_right[0]
        self.left_rgb = 0
        self.right_rgb = 0
        self.red_left = [242, 41, 25]
        self.red_right = [243, 42, 28]
        self.blue_left = [42, 119, 235]
        self.blue_right = [45, 101, 215]
        self.green_left = [36, 150, 97]
        self.green_right = [36, 123, 75]
        self.threshold = 21
        self.basic_speed = 5
        self.max_speed = 60
        self.rotation_speed = 20
        self.last_error = 0
        # self.Kp = 0.057
        # self.Kd = 0.025
        self.Kp = 10
        self.Kd = 8
        self.dt = 0.01
        self.package = False
        self.color = False
        self.color_side = 0

    def stop(self) -> None:
        self.drive_tank.on(SpeedPercent(0), SpeedPercent(0))

    def turn_90(self, mode:int=0) -> None:
        self.stop()
        self.drive_tank.on_for_degrees(SpeedPercent(self.rotation_speed), SpeedPercent(self.rotation_speed), 90)
        # turn left
        if mode == 0:
            self.drive_tank.on_for_degrees(SpeedPercent((-1)*self.rotation_speed), SpeedPercent(self.rotation_speed), 210)
        # turn right
        else:
            self.drive_tank.on_for_degrees(SpeedPercent(self.rotation_speed), SpeedPercent((-1)*self.rotation_speed), 210)
        self.drive_tank.on_for_degrees(SpeedPercent(self.rotation_speed), SpeedPercent(self.rotation_speed), 90)
        sleep(self.dt)

    def turn_180(self) -> None:
        self.stop()
        self.drive_tank.on_for_degrees(SpeedPercent((-1)*self.rotation_speed), SpeedPercent(self.rotation_speed), 430)
        sleep(self.dt)

    def lifting(self, mode:int=0) -> None:
        # up
        if mode == 0:
            self.lifter.run_timed(speed_sp=-512, time_sp=1000)
        # down
        if mode == 1:
            self.lifter.run_timed(speed_sp=512, time_sp=1000)

    def adjust_readings(self, reading: int, mode:int=0) -> int:
        """
        Probnie zrobione na idealny zakres rgb 0-255, zakomentowane to kod dostosowujacy do prawego sensora, pozbyc sie mode wtedy
        """
        # left
        # if mode == 0:
        #     return int(reading * (255/self.range_left))
        # # right
        # else:
        #     return int(reading * (255/self.range_right))
        return int(reading * (self.range_right/self.range_left))

    def correct_error(self, error: int, mode:int=0) -> int:
        # left
        if mode == 0:
            error -= self.black_left[0]
            error = error / (self.white_left-self.black_left[0])
        # right
        else:
            error -= self.black_right[0]
            error = error / (self.white_right-self.black_right[0])
        return error

    def get_rgb(self) -> None:
        self.left_rgb = self.left_sensor.rgb
        self.right_rgb = self.right_sensor.rgb
        rl, gl, bl = self.left_rgb
        rr, gr, br = self.right_rgb

    def error_value(self) -> int:
        left_val = self.left_rgb[0]
        right_val = self.right_rgb[0]
        self.black_left[0] = min(self.black_left[0], left_val)
        self.black_right[0] = min(self.black_right[0], right_val)
        # adjusting errors
        left_val = self.correct_error(left_val)
        right_val = self.correct_error(right_val, 1)
        return left_val - right_val

    def adjust_speed(self, error: int) -> int:
        # error = self.error_value()
        derivative = error - self.last_error
        adjusted_sp = self.Kp * error + self.Kd * derivative
        self.last_error = error
        return int(adjusted_sp)

    def cap_speed(self, speed: int) -> int:
        if speed > self.max_speed:
            return self.max_speed
        elif speed < (-1)*self.max_speed:
            return (-1)*self.max_speed
        else:
            return speed

    def move_engines(self) -> None:
        error = self.error_value()
        adj_sp = self.adjust_speed(error)
        move_sp_l = self.cap_speed(self.basic_speed + adj_sp)
        move_sp_r = self.cap_speed(self.basic_speed - adj_sp)
        self.drive_tank.on(SpeedPercent(move_sp_l), SpeedPercent(move_sp_r))

    def check_color(self) -> bool:
        self.get_rgb()
        color = True
        for i in range(3):
            if (self.left_rgb[i] in range(self.green_left[i] - self.threshold, self.green_left[i] + self.threshold)) or (
                self.left_rgb[i] in range(self.blue_left[i] - self.threshold, self.blue_left[i] + self.threshold)
            ):
                continue
            else:
                color = False
                break
        if color:
            self.color_side = 0
            return color
        color = True
        for i in range(3):
            if (self.right_rgb[i] in range(self.green_right[i] - self.threshold, self.green_right[i] + self.threshold)) or (
                self.right_rgb[i] in range(self.blue_right[i] - self.threshold, self.blue_right[i] + self.threshold)
            ):
                continue
            else:
                color = False
                break
        if color:
            self.color_side = 1
            return color
        return color

    def check_color_both(self) -> bool:
        self.get_rgb()
        color = True
        # left side
        for i in range(3):
            if (self.left_rgb[i] in range(self.green_left[i] - self.threshold, self.green_left[i] + self.threshold)) or (
                self.left_rgb[i] in range(self.blue_left[i] - self.threshold, self.blue_left[i] + self.threshold)
            ):
                continue
            else:
                color = False
                return color
        for n in range(3):
            if (self.right_rgb[i] in range(self.green_right[i] - self.threshold, self.green_right[i] + self.threshold)) or (
                self.right_rgb[i] in range(self.blue_right[i] - self.threshold, self.blue_right[i] + self.threshold)
            ):
                continue
            else:
                color = False
                return color
        return color

    def check_black(self) -> bool:
        self.get_rgb()
        black = True
        # left side
        for i in range(3):
            if self.left_rgb[i] in range(self.black_left[i] - self.threshold, self.black_left[i] + self.threshold) or (
                self.left_rgb[i] in range(self.blue_left[i] - self.threshold, self.blue_left[i] + self.threshold)
            ):
                continue
            else:
                black = False
                return black
        for n in range(3):
            if self.right_rgb[n] in range(self.black_right[n] - self.threshold, self.black_right[n] + self.threshold) or (
                self.right_rgb[i] in range(self.blue_right[i] - self.threshold, self.blue_right[i] + self.threshold)
            ):
                continue
            else:
                black = False
                return black
        return black

    def get_package(self) -> None:
        self.drive_tank.on_for_degrees(SpeedPercent((-1)*self.basic_speed), SpeedPercent((-1)*self.basic_speed), 75)
        self.turn_180()
        self.drive_tank.on_for_degrees(SpeedPercent((-1)*self.basic_speed), SpeedPercent((-1)*self.basic_speed), 75)
        self.lifting()
        self.package = True
        self.drive_tank.on_for_degrees(SpeedPercent(self.basic_speed), SpeedPercent(self.basic_speed), 75)

    def leave_package(self) -> None:
        self.drive_tank.on_for_degrees(SpeedPercent((-1)*self.basic_speed), SpeedPercent((-1)*self.basic_speed), 75)
        self.turn_180()
        self.drive_tank.on_for_degrees(SpeedPercent((-1)*self.basic_speed), SpeedPercent((-1)*self.basic_speed), 75)
        self.lifting(1)
        self.package = False
        self.drive_tank.on_for_degrees(SpeedPercent(self.basic_speed), SpeedPercent(self.basic_speed), 75)

    def run(self) -> None:
        self.lifting(1)
        run = True
        state = 0
        while run:
            # stopping
            if self.button.is_pressed:
                self.stop()
                run = False
                break

            if state == 0:
                # just drive
                if self.check_color():
                    self.turn_90(self.color_side)
                    state = 1
                    continue
                self.move_engines()

                # found color
            elif state == 1:
                if self.check_color_both():
                    if self.package:
                        self.leave_package()
                        state = 2
                        continue
                    else:
                        self.get_package()
                        state = 2
                        continue
                self.move_engines()

                # get out of color line
            elif state == 2:
                if self.check_black():
                    self.turn_90(self.color_side)
                    state = 0
                    continue
                self.move_engines()


    # for testing
    def just_drive(self) -> None:
        run = True
        while run:
            if self.button.is_pressed:
                self.stop()
                run = False
                break
            self.move_engines()

    def calibrate(self) -> None:
        self.left_sensor.calibrate_white()
        self.right_sensor.calibrate_white()
        calibrate = True
        while calibrate:
            if self.button.is_pressed:
                calibrate = False
                break
            rl, gl, bl = self.left_sensor.rgb
            rr, gr, br = self.right_sensor.rgb
            print("Lewy: ", rl, gl, bl)
            print("Prawy: ", rr, gr, br)
            sleep(2)


if __name__== "__main__":
    robot = Robot()
    robot.run()
