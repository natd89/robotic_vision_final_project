import rospy
import smbus
import numpy as np
from pdb import set_trace as pause
from std_msgs.msg import Float64MultiArray
import RPi.GPIO as gpio

class servo_control():

    def __init__(self):
        rospy.Subscriber('servo_commands',Float64MultiArray,self.callback) 
        self.servo_commands = Float64MultiArray()
        self.bus = smbus.SMBus(1)
        self.servo1 = 255/2
        self.servo2 = 255/2
        self.address = 0x08
        self.rate = rospy.Rate(30)
        self.commands = []
        
        check = 1
        while len(self.commands)==0:
            if check:
                print 'waiting for servo commands'
                check = 0
        

    def callback(self,msg):
        self.commands = np.array(msg.data)

    def send_i2c(self):
        try:
            self.bus.write_i2c_block_data(self.address,1,[self.servo1,self.servo2])
        except IOError:
            rospy.sleep(0.01)

    def convert_commands(self):
        self.servo1 = int(-25.5*self.commands[0]+127.5)
        self.servo2 = int(-25.5*self.commands[1]+127.5)
        self.servo1 = np.clip(self.servo1,0,255)
        self.servo2 = np.clip(self.servo2,0,255)

        
    def run(self):
        
        # set the table position to 0,0
        self.send_i2c()

        while not rospy.is_shutdown():
            
            self.convert_commands()
            self.send_i2c()
            print self.servo1, self.servo2
            self.rate.sleep()            
            
            if rospy.is_shutdown():
                self.bus.write_i2c_block_data(self.address,1,[127,127])
                rospy.sleep(0.5)
                print 'Shutting Down...'


if __name__=="__main__":
    
    # Setup ros node and initialize publisher
    rospy.init_node('servos', anonymous=True)

    # initailize the servo class
    servo = servo_control()
    
    # run main function
    servo.run()

        


