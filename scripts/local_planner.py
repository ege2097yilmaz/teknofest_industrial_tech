
from queue import Empty
import rospy, tf, sys

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

from teknofest_industrial_tech.srv import *
from teknofest_industrial_tech.msg import qr
from teknofest_industrial_tech.msg import linePoint

class planner:
    def __init__(self):
        rospy.logwarn("[INFO] Planner is starting !!!")

        # creating necesarry objects
        self.hiz = 0
        self.donus = 0
        self.index = 0

        self.loadPath_counter = 0
        self.unloadPath_counter = 0

        self.max_vel = rospy.get_param("/max_velocity", 0.28) #maximum linear velocity
        self.min_vel = rospy.get_param("/min_velocity", 0.1) #minimum linear velocity
        self.frequency = rospy.get_param("/control_frequency", 8) #frequency of control
        self.accelaration = rospy.get_param("/acceleration", 1.02) #acceleration of the robot
        self.tolerance = rospy.get_param("/rotation_tolerance", 0.08) #rottation tolerance
        self.odom = rospy.get_param("odom_topic", "/odom") #odom topic
        self.imu = rospy.get_param("imu_topic", "/imu") # imu topic
        self.max_rotation = rospy.get_param("max_rotation", 0.7) #max rotation velocity

        # parameters for lane detecttion 
        self.p = rospy.get_param("proportion", 1.3) 
        self.I = rospy.get_param("integration", 2150)

        self.planning = False #for planning process
        self.Qr = False
        self.state = False
        self.recovery_count = 0 #recovery counter to trigger
        self.detect_obstacle = False #detect obstacle to trigger passing around the obstacle

        self.directLetter =""

        self.Imu_msg = []
        self.load_path = []
        self.unload_path = []
        self.start_path = []
        self.unload_dict = []

        # defining sub and pub objects
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1000)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_safety)
        self.midPointSub = rospy.Subscriber('/points', linePoint, self.lane_function)
        self.imuSubs = rospy.Subscriber(self.imu, Imu, self.imuFunc)
        self.qrSubs = rospy.Subscriber('/qr_state', qr, self.qrFunc)
        rospy.Subscriber('/odom_filtered',Odometry,self.odomFunc)

        # tf broadcaster
        self.listener = tf.TransformListener()

        # twist object
        self.twist = Twist()

        # seperate directions are based on values correspond to angle in radian
        self.directs = {
            "R": 0.68,
            "L": -0.65,
            "U": 1.0,
            "D": 0.05
        }

        # run the main func
        self.main()

    # odometry callback function
    def odomFunc(self, odomDatas):
        self.vel_x = odomDatas.twist.twist.linear.x
        self.vel_y = odomDatas.twist.twist.linear.y
        if self.vel_x < self.min_vel:
            self.vel_x = self.min_vel

    def lidar_safety(self, veri):
        self.bolgeler = {
            # seperate sub areas and anumurate them through datas coming from lidar
            'on1': min(min(veri.ranges[0:9]), 30),
            'on2': min(min(veri.ranges[349:359]), 30),
            'on_sol': min(min(veri.ranges[10:49]), 30),
            'sol': min(min(veri.ranges[50:89]), 30),
            'sag': min(min(veri.ranges[269:308]), 30),
            'on_sag': min(min(veri.ranges[309:348]), 30)
        }

    # callback function for Imu datas
    def imuFunc(self, data):
        self.Imu_msg = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        
    # publish stable cmd_vel for specific time
    def follow_lane(self, wait_time, linear, angular):
        self.twist.linear.x = linear
        self.twist.angular.z = angular
        self.cmd_vel_pub.publish(self.twist)
        rospy.sleep(wait_time)

    # longitudunal control 
    def hareket_kontrol(self, bolgeler):
        self.planning = True
        global percent

        percent = self.accelaration # acceralation of the robot
        self.linear_vel =  self.vel_x * percent

        # control of maximum velocity
        if self.linear_vel > self.max_vel:
            self.linear_vel = self.max_vel

        # control of lane is detected or not
        if self.avaiable == True:
            rospy.loginfo("[INFO] lane is detected")
            # linear velocity
            self.hiz = self.linear_vel
            state = True
        else: 
            rospy.logerr("lane is not detected")
            self.hiz = 0.0
            state = False
            while self.recovery_count > 45:
                self.follow_lane(1, -0.25, 0.0)
                self.recovery_count = 0

        #controlling areas
        if bolgeler['on1'] < 0.4:
            self.hiz = 0.0
            # self.follow_lane(0.01, 0.0, 0.0) #stop the robot
            # self.detect_obstacle = True
            # self.planning = False
            print("on1")

        if bolgeler['on2'] < 0.4:
            self.hiz = 0.0
            # self.follow_lane(0.01, 0.0, 0.0) #stop the robot
            # self.detect_obstacle = True
            # self.planning = False
            print("on2")

        if bolgeler['on_sol'] < 0.4:
            self.hiz = 0.0
            print("on_sol")

        if bolgeler['on_sag'] < 0.4:
            self.hiz = 0.0
            # self.follow_lane(0.01, 0.0, 0.0) #stop the robot
            # self.detect_obstacle = True
            # self.planning = False
            print("on_sag")

        if bolgeler["sag"] < 0.4:
            self.hiz = 0.0
            # self.follow_lane(0.01, 0.0, 0.0) #stop the robot
            # self.detect_obstacle = True
            # self.planning = False
            print("sag")

        return self.hiz, state

    # callback for midpoint of the line
    def lane_function(self, points):
        self.avaiable = points.avaiable
        self.cx = points.cx
        self.cy = points.cy
        self.w = points.w

    # path planning service
    def get_directions(self, point1, point2):

        # waiting for service
        rospy.wait_for_service("find_path")
        try:
            path1 = rospy.ServiceProxy("find_path", directions)
            response = path1(point1, point2, "S1")
            self.load_path = response.load_directions
            self.unload_path = response.unload_directions
            self.start_path = response.go_start
            rospy.logwarn(response)

        except rospy.ServiceException as w:
            # print("service called fialed %s", w)
            rospy.logerr("service called fialed %s. be sure that points are correct", w)
            self.planning = False

            # waiting for srv server
            rospy.sleep(0.5)

    # adjust the list of directions
    def adjustment_of_directs(self):
        fullPath = self.load_path + self.unload_path + self.start_path
        self.unload_dict = self.load_path + self.unload_path

        # control of reaching home point
        if self.index >= len(fullPath):
            self.directLetter = ""
            rospy.loginfo("Goal reached")

            # re-adjust the initial values
            self.index = 0
            self.planning = False

        # controlling of fullpath inside the directions (if u dont understan this, go into the path_find.py script)
        while self.index < len(fullPath):
            self.directLetter = fullPath[self.index]
            print(f"current direct letter: {self.directLetter}")
            if self.Qr == True:
                self.index += 1
                self.follow_lane(3.5, 0.25, 0.0)

                if self.loadPath_counter == 0:
                    # control to reach load and unload positions
                    try:
                        reference = self.load_path[self.index]
                    except:
                        rospy.logdebug("load position has been reached")
                        self.follow_lane(10, 0.0, 0.0)
                        self.loadPath_counter +=1

                if self.unloadPath_counter == 0:
                    # control to reach load and unload positions
                    try:
                        reference = self.unload_dict[self.index]
                    except:
                        rospy.logdebug("unload position has been reached")
                        self.follow_lane(10, 0.0, 0.0)
                        self.unloadPath_counter +=1
                self.Qr = False
            break

    def qrFunc(self, data):
        if data.qr == True:
            rospy.logdebug("Qr detected")
            self.Qr = True

    def setDemandz(self):
        rospy.logdebug("publishing cmd_vel to adjust orientation")
        if self.directLetter == "U":
            prop = 3.0
            time = 1.0
        else:
            prop = 2.5 #proportion control value
            time = 0.5
        i_value = 1.75 #i value
        ori_error = self.rot[2] - self.directs[self.directLetter]
        vel_z = -float(ori_error)*prop/i_value

        if vel_z >  self.max_rotation:
            vel_z =  self.max_rotation 
        elif vel_z < -1 * self.max_rotation :
            vel_z = -1 *  self.max_rotation 

        if 0 < vel_z < 0.15:
            vel_z = 0.16
        elif -0.15< vel_z <0:
            vel_z = -0.16

        # turn around the robot up to see a line with PID
        self.twist.linear.x = 0.0
        self.twist.angular.z = vel_z
        self.cmd_vel_pub.publish(self.twist)
        rospy.sleep(time) # this is for stablity when robot change it's direction

    def go_around_obstacle(self, ref_area):
        if ref_area['on1'] < 1.0 or ref_area['on2'] < 1.0:
            rospy.logdebug("buzzer activated")
            rospy.sleep(15)
            if ref_area['on1'] < 1.0 or ref_area['on2'] < 1.0:
                #first is frequency, second linear, third angular velocity
                # buraya gülmeyin lütfen
                self.follow_lane(1.0, 0.0, 0.5) #turn left  
                self.follow_lane(4.5, 0.3, 0.0) #go forward
                self.follow_lane(1.0, 0.0, -0.5) #turn right
                self.follow_lane(2.0, 0.3, 0.0) #go forward
                self.follow_lane(1.0, 0.0, -0.5) #turn right
                self.follow_lane(4.5, 0.3, 0.0) #go forward
                self.follow_lane(0.5, 0.3, 0.5) #turn left
            self.detect_obstacle = True
        else:
            self.detect_obstacle = False
            self.planning = True

        # self.safety_distance = 0.8 #inflation radius
        # P = 1.2 #proportion control
        # I = 0.01 #integral control
        # # here @buzzer trigger buzzer function

        # if ref_area['on1'] < 1.0 or ref_area['on2'] < 1.0 or ref_area['on_sag'] < 1.0:
        #     rospy.loginfo("minimum distance is %i", ref_area['on_sag'])
        #     error = -1*(ref_area['on_sag'] - self.safety_distance)
        #     demz = (error * P) + (I * error)
        #     print(f"angular velocity: {demz}")
        #     # adjust angular velocity
        #     if demz > 0.5:
        #         demz = 0.5
        #     elif demz < -0.5:
        #         demz = -0.5
        #     self.follow_lane(0.1, 0.35, demz) #first is frequency, second linear, third angular velocity
        #     self.detect_obstacle = True
        # else:
        #     self.detect_obstacle = False
        #     self.planning = True

            
    def main(self):
        rospy.loginfo("[INFO] waiting for sensor datas !!!")

        # waiting to be sure whether publishers are publisng datas or not
        rospy.sleep(0.5)
        while not rospy.is_shutdown():

            # setting the frequency of loop
            rospy.sleep(1/self.frequency)
            try:
                # tf listener to get rotation of the robot
                (self.trans,self.rot) = self.listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
                # print(f"robot orientation is: {self.rot}")
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            
            # defining directions func
            self.adjustment_of_directs()
            rospy.logwarn("[INFO] planning process is: %s", self.planning)

            # Planning is start
            if self.planning:
                tol = self.tolerance

                # adjusting longutidunal control
                self.hiz, self.state = self.hareket_kontrol(bolgeler=self.bolgeler)

                # state control of the robot in case of not finding lane
                if self.state == False:
                    tol = 0.01
                    self.recovery_count += 1
                    rospy.logwarn("selected tolerance is 0.01")
                
                # because of quartenion matrixes, it is necessary to adjust tolerance value for U direction
                if self.directLetter == "U":
                    tol = 0.02
                    self.rot[2] = abs(self.rot[2])

                # firstly look onto orientation
                if abs(self.rot[2] - self.directs[self.directLetter]) < tol:

                    if sys.getsizeof(self.cx) is not Empty:

                        rospy.loginfo("lane tracking is activated")

                        # PID for lateral control
                        error = self.cx - self.w
                        self.twist.linear.x = self.hiz
                        self.twist.angular.z = -float(error) * self.p / self.I  #P=1.1 I=2150 for lane tracking
                        self.cmd_vel_pub.publish(self.twist)
                        
                    else:
                        rospy.logwarn("self.cx is empty")
                        self.twist.linear.x = 0
                        self.twist.angular.z = 0
                    self.cmd_vel_pub.publish(self.twist)

                else: self.setDemandz()

            # controlling obstacles and getting rid of them
            elif self.planning == False and self.detect_obstacle == True:
                self.hareket_kontrol(bolgeler=self.bolgeler)
                self.go_around_obstacle(ref_area=self.bolgeler)
                rospy.logwarn("obstacle detected waiting for getting free")

            else:
                # set to PLANNING=TRUE to start planning process
                self.planning = True
                self.twist.linear.x = 0
                self.twist.angular.z = 0
                self.cmd_vel_pub.publish(self.twist)

                self.loadPath_counter = 0
                self.unloadPath_counter = 0
 
                # take inputs to create a route for motion planning
                rospy.logdebug("[INFO] Attempting planning process")
                first = input("Enter load point:")
                second = input("Enter unload point:")

                # input for path planning
                self.get_directions(str(first), str(second))