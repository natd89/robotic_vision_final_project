import rospy
import numpy as np
import control
from std_msgs.msg import Float64MultiArray

class bc():

    def __init__(self):
        self.ball_pos
        self.xhat
        self.xd
        self.dt = 1/30.
        self.rate = rospy.Rate(30)
        self.flag = 1
        self.commands = Float64MultiArray()
        
        # initialize subscriber and publisher
        # subscribe to message from camera position estimate
        rospy.Subscriber('/ball_pos', Float64MultiArray, self.callback)
        # publish servo commands based on calculated control input
        self.pub = rospy.Publisher('/servo_commands', Float64MultiArray, queue_size=1)

    def callback(self, msg):
        temp = msg.data
        self.ball_pos=np.array([[temp[0]],[temp[1]]])


    def compute_k(self):
        A = np.array([[0,0,1.,0],
                      [0,0,0,1.],
                      [0,0,0,0],
                      [0,0,0,0]])
        
        B = np.array([[0,0],
                      [0,0],
                      [5/7.*self.g,0],
                      [0,5/7.*self.g]])
        
        Q = np.array([[1.,0,0,0],
                      [0,1.,0,0],
                      [0,0,1.,0],
                      [0,0,0,1.]])
        
        R = np.array([[100.,0,0,0],
                      [0,100.,0,0],
                      [0,0,100.,0],
                      [0,0,0,100.]])

        self.K = contro.lqr(A,B,Q,R)

        
    def run(self):

        while not rospy.is_shutdown():
            # if start of program == 0, ball starts at (0,0)
            if self.flag:
                pos_prev = np.array([[0],[0]])
                self.flag=0            
                # compute the LQR gain
                self.compute_k()
                
            # compute the velocity
            ball_vel = (self.ball_pos-pos_prev)/self.dt
            
            # build the state vector
            xhat = np.array([[self.ball_pos[0]-self.xd[0]],
                             [self.ball_pos[1]-self.xd[1]],
                             [ball_vel[0]],
                             [ball_vel[1]]])

            # compute the desired servo outputs
            u = np.matmul(-self.K,xhat)                    
            
            self.commands.data = [u[0],u[1]]
            self.pub.publish(self.commands)
            
            self.rate.sleep()

            
if __name__=="__main__":

    rospy.init_node('LQR_controller',anonymous=True)
        
    controller = bc()

    controller.run()
