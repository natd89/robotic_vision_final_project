import rospy
import numpy as np
import control
from pdb import set_trace as pause
from std_msgs.msg import Float64MultiArray

class bc():

    def __init__(self):
        self.ball_pos = []
        self.xd = [0,0]
        self.dt = 1/30.
        self.rate = rospy.Rate(30)
        self.flag = 1
        self.commands = Float64MultiArray()
        self.g = 9.81
        self.fc = 30
        self.alpha = (2*np.pi*self.dt*self.fc)/(2*np.pi*self.dt*self.fc + 1)
        self.error_old = np.array([[0],[0]])
        self.edot_old = np.array([[0],[0]])
        self.pos_des = np.array([[0],[0]])
        self.integrator = 0
        self.tau = 0.05

        # PID gains
        self.kp = -1.0
        self.kd = -0.5
        self.ki = -0.5


        
        # initialize subscriber and publisher
        # subscribe to message from camera position estimate
        rospy.Subscriber('/ball_pos', Float64MultiArray, self.callback)
        # publish servo commands based on calculated control input
        self.pub = rospy.Publisher('/servo_commands', Float64MultiArray, queue_size=1)

        while len(self.ball_pos)==0:
            a = 1

    def callback(self, msg):
        temp = msg.data
        self.ball_pos=np.array([[temp[0]],[temp[1]]])


    def compute_pid(self):

        # low pass filter the error
        self.low_pass()
        
        # calculate the derivative of the error
        self.edot = (2*self.tau-self.dt)/(2*self.tau+self.dt)*self.edot_old + (2/(2*self.tau +self.dt))*(self.error-self.error_old)

        # integrate the error
        self.integrator = self.integrator + (self.dt/2)*(self.error + self.error_old)

        up = self.kp*(self.error)
        ud = self.kd*(self.edot)
        ui = self.ki*(self.integrator)
        
        self.u = np.clip(up + ud + ui,-5.,5.)
        
        # implement integrator anti-windup
        if self.ki!=0:
            u_unsat = up+ud+ui
            k_antiwindup = self.dt/self.ki
            self.integrator = self.integrator + k_antiwindup*(self.u-u_unsat)

        print self.u
            
        
    def low_pass(self):
        self.error = self.alpha*self.error + (1-self.alpha)*self.error_old

        
    def run(self):

        while not rospy.is_shutdown():
            # if start of program == 0, ball starts at (0,0)
            if self.flag:
                self.pos_prev = np.array([[self.ball_pos[0][0]],[self.ball_pos[1][0]]])
                self.flag=0           
            else:
                # low-pass filter the position and velocity 
                self.error = self.pos_des-self.ball_pos
                
                self.compute_pid()
                
                self.commands.data = [self.u[0],self.u[1]]
                self.pub.publish(self.commands)
                self.pos_prev = self.ball_pos
                self.error_old = self.error
                self.edot_old = self.edot
                self.rate.sleep()
                
                
if __name__=="__main__":

    rospy.init_node('LQR_controller',anonymous=True)
        
    controller = bc()

    controller.run()
