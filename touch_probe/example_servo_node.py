import json
import math
import rospy
import sys
import RPi.GPIO as GPIO
from std_msgs.msg import Float32
from geometry_msgs.msg import Point32
from time import sleep

#pulse width of difference rotations
d_45 = 1.0
d_90 = 1.5
d_180 = 2.5

frequency = 50.0
t_per_cycle = (1.0 / frequency) * 1000.0

#convert to duty cycles
duty_45 = (d_45 / t_per_cycle) * 100.0
duty_90 = (d_90 / t_per_cycle) * 100.0
duty_180 = (d_180 / t_per_cycle) * 100.0

#gear spec
radius = 2.25
cir = 2.0 * radius * math.pi
d = cir / 20.0
cm_theta = 18.0 / d

z_radius = 1.0
z_cir = 2.0 * z_radius * math.pi
z_d = z_cir / 10.0
z_cm_theta = 36.0 / d



class Servo_node:
    def __init__(self):
        rospy.init_node('servo_node', anonymous=False)
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        # Setting up for pin 12. Make sure to adjust for your own needs
        motor_x = 13
        motor_y = 12
        motor_z = 20

        GPIO.setup(motor_x, GPIO.OUT)
        GPIO.setup(motor_y, GPIO.OUT)
        GPIO.setup(motor_z, GPIO.OUT)
        # 0.75-2.75
        self.pwm_x = GPIO.PWM(motor_x, frequency)
        # 2-3
        self.pwm_y = GPIO.PWM(motor_y, frequency)
        # 0.8-1.8
        self.pwm_z = GPIO.PWM(motor_z, frequency)
        #set start position to (0,0)
        self.pwm_z.start(duty_45)
        sleep(0.5)
        self.pwm_z.ChangeDutyCycle(0)

        self.pwm_x.start(duty_180)
        sleep(0.5)
        self.pwm_x.ChangeDutyCycle(0)

        self.pwm_y.start(duty_45)
        sleep(0.5)
        self.pwm_y.ChangeDutyCycle(0)
        #topic takes angle as message
        self.sub_x = rospy.Subscriber("/servo_ctrl/s1", Float32, self.set_servo_x_angle)
        self.sub_y = rospy.Subscriber("/servo_ctrl/s2", Float32, self.set_servo_y_angle)
        self.sub_z = rospy.Subscriber("/servo_ctrl/s3", Float32, self.set_servo_z_angle)
        #topic for position commands
        self.pos_sub = rospy.Subscriber("/servo_ctrl/pos", Point32, self.set_coordinate)

    def set_servo_x_angle(self, msg):
        rospy.loginfo("setting servo")
        self.pwm_x.ChangeDutyCycle(self.saturate_input(msg.data))# Note tha this does not correspond to angle
        sleep(1)
        self.pwm_x.ChangeDutyCycle(0)
        sleep(0.5)

    def set_servo_y_angle(self, msg):
        rospy.loginfo("setting servo")
        self.pwm_y.ChangeDutyCycle(self.saturate_input(msg.data))  # Note tha this does not correspond to angle
        sleep(1)
        self.pwm_y.ChangeDutyCycle(0)
        sleep(0.5)

    def set_servo_z_angle(self, msg):
        rospy.loginfo("setting servo")
        self.pwm_z.ChangeDutyCycle(self.saturate_input(msg.data))  # Note tha this does not correspond to angle
        sleep(1)
        self.pwm_z.ChangeDutyCycle(0)
        sleep(0.5)
    def set_coordinate(self, msg):
        #conversion between coordinate to motor angles
        rospy.loginfo("setting position")
        #correction for motors
        #offset added to make sure the touch probe is at (0,0) initially
        #may need to change depends on your motor
        x_offset = 0
        y_offset = -5
        z_offset = 0
        x = msg.x
        y = msg.y
        z = msg.z
        z_pub = rospy.Publisher('servo_ctrl/s3', Float32, queue_size=10)
        x_pub = rospy.Publisher('servo_ctrl/s1', Float32, queue_size=10)
        y_pub = rospy.Publisher('servo_ctrl/s2', Float32, queue_size=10)
        x_angle = 180 - x * cm_theta + x_offset
        y_angle = 45 + y * cm_theta + y_offset
        z_angle = 45 + (1.5 - z) * z_cm_theta + z_offset

        if x == -1 or y == -1 or z == -1:
            if x == -1:
                self.pwm_x.ChangeDutyCycle(0)
            else:
                x_pub.publish(Float32(x_angle))
            if y == -1:
                self.pwm_y.ChangeDutyCycle(0)
            else:
                y_pub.publish(Float32(y_angle))
            if z == -1:
                self.pwm_z.ChangeDutyCycle(0)
            else:
                z_pub.publish(Float32(z_angle))
        elif x >= 0 and x <= 2.5 and y >= 0 and y <= 4:
            # z_pub.publish(Float32(45))
            x_pub.publish(Float32(x_angle))
            y_pub.publish(Float32(y_angle))
            z_pub.publish(Float32(z_angle))

    def saturate_input(self, angle):
        #conversion from angle to duty cycles
        print(angle)
        pw_per_deg = (duty_180 - duty_90) / 90;
        duty = pw_per_deg * (angle - 45) + duty_45
        print(duty)
        return max(min(duty,100),0)


def main_loop():
    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == "__main__":
    servo = Servo_node()
    main_loop()
