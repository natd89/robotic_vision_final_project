import rospy
import numpy as np
from pdb import set_trace as pause
from std_msgs.msg import Float64MultiArray
import RPi.GPIO as gpio

class servo_control():

    def __init__(self):
        rospy.Subscriber('servo_commands',Float64MultiArray,self.callback) 
        self.servo_commands = Float64MultiArray()
        self.pins_out = [12,33]
        self.pins_in=[]
        self.rate = rospy.Rate(100)
        self.frequency = 50
        self.duty_cycle = [6.95,6.95]
        self.commands = []
        self.pwm = []
        
        check = 1
        while len(self.commands)==0:
            if check:
                print 'waiting for servo commands'
                check = 0
        
    def callback(self,msg):
        self.commands = np.array(msg.data)
        
        
    def setup_pins(self):
            
        # set the GPIO mode to BOARD or BCM (broadcom chip)
        gpio.setmode(gpio.BOARD)
        
        # setup the input pins
        for i in range(len(self.pins_in)):
            gpio.setup(self.pins_in[i], gpio.IN)
            
        # setup the output pins
        for i in range(len(self.pins_out)):
            gpio.setup(self.pins_out[i], gpio.OUT)


        
    def setup_pwm(self, start=True, dutyCycle=6.95):

        for i in range(len(self.pins_out)):
            self.pwm.append(gpio.PWM(self.pins_out[i], self.frequency))
        if start:
            for i in range(len(self.pwm)):
                self.pwm[i].start(dutyCycle)

            

    def change_duty_cycle(self):
        for i in range(len(self.duty_cycle)):
            self.pwm[i].ChangeDutyCycle(self.duty_cycle[i])


    def convert_commands(self):
        self.duty_cycle = -0.99*self.commands+6.95
        self.duty_cycle = np.clip(self.duty_cycle,-5,5)
        
    def run(self):
        
        # set the table position to 0,0
        self.change_duty_cycle()

        while not rospy.is_shutdown():

            self.convert_commands()
            self.change_duty_cycle()
            self.rate.sleep()
                
            if rospy.is_shutdown():
                # set the table position to 0,0
                self.duty_cycle = [6.59,6.95]
                self.change_duty_cycle()
                rospy.sleep(0.5)
                gpio.cleanup()
                print 'Shutting Down...'


if __name__=="__main__":
    
    # Setup ros node and initialize publisher
    rospy.init_node('servos', anonymous=True)

    # initailize the servo class
    servo = servo_control()
    
    # set up IO pins and PWM signal (pins 12 and 33 are PWM capable)
    servo.setup_pins()
    servo.setup_pwm()
        
    # run main function
    servo.run()

        


