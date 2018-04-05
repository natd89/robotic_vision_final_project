import rospy
import numpy as np
from pdb import set_trace as pause
from std_msgs.msg import Float64MultiArray
import RPi.GPIO as gpio



def setup_pins(pins_in=[], pins_out=[]):

    # set the GPIO mode to BOARD or BCM (broadcom chip)
    gpio.setmode(gpio.BOARD)

    # setup the input pins
    for i in range(len(pins_in)):
        gpio.setup(pins_in[i], gpio.IN)

    # setup the output pins
    for i in range(len(pins_out)):
        gpio.setup(pins_out[i], gpio.OUT)


        
def setup_pwm(pins, frequency, start=True, dutyCycle=6.95):
    pwm = []
    for i in range(len(pins)):
        pwm.append(gpio.PWM(pins[i], frequency))
    if start:
        for i in range(len(pwm)):
            pwm[i].start(dutyCycle)
    return pwm
            

def change_duty_cycle(pwm, dutyCycle):
    pwm.ChangeDutyCycle(dutyCycle)


def cleanup():
    # set the table position to 0,0
    dutyCycle1 = 6.59
    dutyCycle2 = 6.59
    change_duty_cycle(pwm[0],float(dutyCycle1))
    change_duty_cycle(pwm[1],float(dutyCycle2))
    rospy.sleep(1)
    gpio.cleanup()


if __name__=="__main__":

    # Setup ros node and initialize publisher
    rospy.init_node('servos', anonymous=True)
    rospy.on_shutdown(cleanup)
    pub = rospy.Publisher('servo_commands',Float64MultiArray,queue_size=1) 
    servo_commands = Float64MultiArray()

    # set up IO pins and PWM signal (pins 12 and 33 are PWM capable)
    setup_pins(pins_out=[12,33])
    pwm = setup_pwm([12,33],50)

    rate = rospy.Rate(100)

    # set the table position to 0,0
    dutyCycle1 = 6.59
    dutyCycle2 = 6.59
    change_duty_cycle(pwm[0],float(dutyCycle1))
    change_duty_cycle(pwm[1],float(dutyCycle2))

    raw_input('press key when ready')
    
    t0 = rospy.get_time()
    
    
    while not rospy.is_shutdown():
        t = rospy.get_time()
        #dutyCycle1 = raw_input("select duty cycle 1 or press 'c'")
        dutyCycle1 = 4.95*np.cos(2*(t-t0))+6.59
        #if not dutyCycle1=='c':
        change_duty_cycle(pwm[0],float(dutyCycle1))
        #dutyCycle2 = raw_input("select duty cycle 2 or press 'c'")
        dutyCycle2 = 4.95*np.sin(2*(t-t0))+6.59
        #if not dutyCycle2=='c':
        change_duty_cycle(pwm[1],float(dutyCycle2))
        servo_commands.data = [dutyCycle1,dutyCycle2]
        pub.publish(servo_commands)
        rate.sleep()
        


