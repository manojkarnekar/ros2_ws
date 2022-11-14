#!/usr/bin/env python
import rospy
import roslib
import os
import sys
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8MultiArray
from std_msgs.msg  import  String
from std_msgs.msg import Int8
from std_msgs.msg import UInt8
from std_msgs.msg import Int16
from std_msgs.msg import Float32
import time
import datetime
import sqlite3
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatusArray, GoalID

env_vars = os.environ 
  
class rosdb():
    def __init__(self):
        
        dbPath = ""
        
        if('DB_PATH' in env_vars):
            dbPath = env_vars['DB_PATH']
        
        if(dbPath==""):
            print("Error: DB_PATH is not set. Exiting..")
            exit(1)

        self.conn = sqlite3.connect(dbPath)
        rospy.init_node('rosdb', anonymous=True)
        
        self.r = rospy.Rate(1)
        self.state = "0"
        self.nextstate = None
        self.lastvoltage = self.voltage = "0"
        self.isCharging = False
        self.ir1 = self.ir2 = 1
        self.liftState = 0
        self.angle = 0
        self.PathId = None
        self.pathIndex = 0
        self.pathString =  ""
        self.fclear = True
        self.bclear = True
        self.isHalted = True
        self.lastMove = rospy.get_time()
        self.waypointType = None
        self.statePub = rospy.Publisher('amro_state', UInt8, queue_size=10, latch=True)
        self.breakPub = rospy.Publisher('tab_control', String, queue_size=10, latch=True)
        self.cancelGoal = rospy.Publisher('move_base/cancel', GoalID, queue_size=10, latch=True)
        self.currentGoalId = None      

        self.lMotor = rospy.Publisher('lmotor_cmd', Int16, queue_size=10, latch=False)
        self.rMotor = rospy.Publisher('rmotor_cmd', Int16, queue_size=10, latch=False)

        rospy.Subscriber("lockstate",UInt8,self.lockStateChecker)
        rospy.Subscriber("move_base/status",GoalStatusArray,self.moveBaseListner)
        rospy.Subscriber("sensors",UInt8MultiArray,self.checkSensors)
        rospy.Subscriber("cmd_vel",Twist,self.checkMovement)
        rospy.Subscriber("ir",UInt8MultiArray,self.updateIR)
        rospy.Subscriber("battery",Int16,self.updateBattery)
        rospy.Subscriber("lift_state",Int8,self.updateLiftState)
        rospy.Subscriber("rotation_degree",Int16,self.updateAngle)
        self.moveBase = actionlib.SimpleActionClient('move_base',MoveBaseAction)        

    def updateAngle(self, msg):
        self.angle = msg.data

    def updateBattery(self, msg):
        self.isCharging = msg.data>0

    def updateIR(self, msg):
        self.ir1 = msg.data[0]
        self.ir2 = msg.data[1]

    def checkIfCharging(self):
        if(self.isCharging):
            self.nextstate=170
        elif(self.state==170):
            self.nextstate=20

    def rotateForCharging(self):
        #os.system("rosservice call /StartLocalization")
        print("Starting Alignment..")
        self.lMotor.publish(-5)
        self.rMotor.publish(+5)
        self.nextstate = 166

    def stopForCharging(self):
        if(self.isCharging):
            self.lMotor.publish(0)
            self.rMotor.publish(0)

    def moveForCharging(self):
        if(self.angle in range(-10,10)):
            self.lMotor.publish(5)
            self.rMotor.publish(5)
            self.nextstate = 168
        #else:
        #    print("Moving to Station..")
        #    self.nextstate = 166

        return

        lm = 0
        rm = 0
        ir1 = self.ir1
        ir2 = self.ir2

        speed = 5
        if(self.isCharging):
            self.nextstate=170
        else:
            if(ir1==0):
                lm = speed
            else:
                lm = -speed

            if(ir2==0):
                rm = speed
            else:
                rm = -speed

        self.lMotor.publish(lm)
        self.rMotor.publish(rm)

    def updateState(self):
        
        c = self.conn.cursor()
        
        val = self.getStateData(c)
        
        if(val and len(val)>0):

            if(not self.PathId==val[2]):
                self.pathString = ""

            self.tripId = val[0]
            self.state = val[1]
            self.PathId = val[2]
            self.statePub.publish(self.state)
            
        print("State : " + str(self.state))    
        print(">> CurrentPathId : " + str(self.PathId))
        print(">> Index : " + str(self.pathIndex))
        print(">> String : " + str(self.pathString))
   
        if(self.state==10):
            self.getReady()

        elif(self.state==50):
            self.trySetGoal(c, 60)
        
        elif(self.state == 70):
            self.checkForClutteredHalt()    
       
        elif(self.state == 140):
            goalMsg = GoalID()
            goalMsg.id = self.currentGoalId
            goalMsg.stamp = rospy.Time.now()
            self.cancelGoal.publish(goalMsg)
            self.currentGoalId = None
            os.system("rosservice call /move_base/clear_costmaps") 
            os.system("rosservice call /StartLocalization")
            self.nextstate = 80

        #elif(self.state==160):
            #ToDo: Set Chargin Path
            #self.PathId = 13
            #self.trySetGoal(c, 162)

        elif(self.state==164):
            self.rotateForCharging()

        elif(self.state==166):
            self.moveForCharging()

        elif(self.state==168):
            self.stopForCharging()

        if(self.state==130):
            self.breakPub.publish("1")    
        else:
            self.breakPub.publish("0")

        self.checkIfCharging()

        if(self.nextstate!=None):
            self.updateNextState(c)
            
        c.close()
        self.r.sleep()
    
    def getStateData(self,c):
        c.execute('SELECT Id,StatusId,PathId FROM amro_trips ORDER BY Id DESC LIMIT 1;')
        return c.fetchone()
    
    def updateNextState(self,c):
        q = 'UPDATE amro_trips SET StatusId=? where Id=?;'
        params=(self.nextstate,self.tripId)
        
        #if(self.pathIndex!=None):
        #    q = 'UPDATE amro_trips SET StatusId=?,PathId=? where Id=?;'
        #    params=(self.nextstate, self.pathIndex, self.tripId)
        #    self.pathIndex = None
        
        c.execute(q,params)
        self.conn.commit()
        self.nextstate=None

    def trySetGoal(self, c, ns):
        if(self.PathId!=-1 and str(self.PathId)!=''):
            if(self.pathString==""): 
                c.execute('''SELECT PathString from paths where id=?''', (self.PathId,))
                self.pathString = c.fetchone()
                self.pathIndex = 0
                
            if(self.pathIndex>=0):
                q = '''SELECT Type,PosX,PosY,PosZ,PosW FROM waypoints WHERE waypoints.Id=?'''
                p = self.pathString[0].split(',')
                c.execute(q, (p[self.pathIndex],))
                val = c.fetchone()
                self.goalStateSetter(val, ns)
        
    def lockStateChecker(self, msg):
        if(self.nextstate==None):
            if(self.state==30):
                if(msg.data==1):
                    self.nextstate=40
            elif(self.state==40):
                if(msg.data==0):
                    self.nextstate=50
            elif(self.state==90):
                if(msg.data==1):
                    self.nextstate=100
            elif(self.state==100):
                if(msg.data==0):
                    self.nextstate=110

    def voltageListener(self,msg):
        self.voltage = str(msg.data)
    
    def goalStateSetter(self, val, ns):
        if(val!=None and len(val)>=5):
            self.waypointType = int(val[0])
            os.system("rosservice call /move_base/clear_costmaps") 
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = float(val[1])
            goal.target_pose.pose.position.y = float(val[2])
            goal.target_pose.pose.orientation.z = float(val[3])
            goal.target_pose.pose.orientation.w = float(val[4])
            self.moveBase.send_goal(goal)
            self.nextstate=ns
            
    def moveBaseListner(self, msg):
        if(not msg):
            return

        for im in range(0,len(msg.status_list)):
        #if(msg and len(msg.status_list)>0):
            status_set =  msg.status_list[im]
            status = status_set.status
            goalId = status_set.goal_id.id

            print(">>>...." + str(self.currentGoalId) + " " + str(goalId) + " Status:" + str(status))

            if(status==1 and self.nextstate==None and (self.state==60 or self.state==162)):
                self.nextstate = 70
                self.currentGoalId = goalId

            if(self.currentGoalId==goalId and self.nextstate==None):
                #if(self.state>=50 and self.state<80):
                if(status==4):
                    self.nextstate = 140
                elif(status==5):
                    self.nextstate = 140
                elif(status==3):
                    self.currentGoalId = None
                    p = 0
                    if(self.pathString!=""): 
                        p = len(self.pathString[0].split(',')) - 1
                    if(self.pathIndex>=p and p!=0):
                        print('Goal Reached.')
                        self.nextstate = 80
                    elif(self.waypointType==2):
                        print('Waiting for Elevators..')
                        self.nextstate = 180
                    elif(self.waypointType==3):
                        print('Waiting In Elevators..')
                        self.nextstate = 200
                    elif(self.waypointType==4):
                        print('Charging Area Reached.')
                        self.nextstate = 164
                    else:
                        print('Setting Next Waypoint..')
                        self.pathIndex = self.pathIndex + 1
                        self.nextstate = 50

    def updateLiftState(self, msg):
        liftState = int(msg.data)
        print("Lift State : ",liftState, " CurrentPathId : ", self.PathId)
        if(self.state==180 and liftState==3):
            self.nextstate = 50
            self.pathIndex = self.pathIndex + 1
        elif(self.state==200 and liftState==5):
            self.nextstate = 50    
            self.pathIndex = self.pathIndex + 1
        #self.liftState = liftState
            
    def checkSensors(self, msg):
        k = msg.data
        if len(k)>=3:
            self.fclear = k[0]==0 and k[1]==0 and k[2]==0
        if len(k)>=5:
            self.bclear = k[3]==0 and k[4]==0

    def checkMovement(self, msg):
        #print(self.lastMove)
        self.isHalted = msg.linear.x==0 and msg.angular.y==0
        if(self.isHalted == False):
            self.lastMove = rospy.get_time()

    def getReady(self):   
        #rospy.wait_for_service("/StartLocalization")
        #os.system("rosservice call /StartLocalization")
        print("Ready..")
        self.nextstate = 20

    def checkForClutteredHalt(self): 
        #print("--" + str(rospy.get_time()) + " " +  str(self.lastMove)) 
        if(self.isHalted):
            current_t = rospy.get_time()
            if((current_t - self.lastMove) > 5):
                os.system("rosservice call /move_base/clear_costmaps")
                os.system("rosservice call /StartLocalization")

            #if((current_t - self.lastMove) > 10):
                #os.system("rosservice call /move_base/clear_costmaps") 
                #if self.bclear and self.fclear:
                #    os.system("rosservice call /StartLocalization")
                #if self.state==70:
                #    self.state==50
                #self.lastMove = current_t


if __name__ == '__main__': 
    rdb = rosdb()
    print("Initializing...")
    while not rospy.is_shutdown():
        rdb.updateState()
