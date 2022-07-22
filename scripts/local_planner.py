
from queue import Empty
import rospy, tf, sys

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

from teknofest_industrial_tech.srv import *
from teknofest_industrial_tech.msg import qr
from teknofest_industrial_tech.msg import linePoint
from teknofest_industrial_tech.msg import positions

from paho.mqtt import client as mqtt_client

class planner:
    def __init__(self):
        rospy.logwarn("[INFO] Planner is starting !!!")

        # creating necesarry objects
        self.hiz = 0
        self.donus = 0
        self.index = 0

        # control indexes for the lift
        self.first_loadPath_counter = 0
        self.first_unloadPath_counter = 0
        self.second_loadPath_counter = 0
        self.second_unloadPath_counter = 0

        self.max_vel = rospy.get_param("/max_velocity", 0.28) #maximum linear velocity
        self.min_vel = rospy.get_param("/min_velocity", 0.1) #minimum linear velocity
        self.frequency = rospy.get_param("/control_frequency", 8) #frequency of control
        self.accelaration = rospy.get_param("/acceleration", 1.02) #acceleration of the robot
        self.tolerance = rospy.get_param("/rotation_tolerance", 0.08) #rottation tolerance
        self.odom = rospy.get_param("odom_topic", "/odom") #odom topic
        self.imu = rospy.get_param("imu_topic", "/imu") # imu topic
        self.max_rotation = rospy.get_param("max_rotation", 0.7) #max rotation velocity
        self.wait_after_qr = rospy.get_param("wait_after_qr", 5.8)
        self.ref = rospy.get_param("turnning_at_boot", True)

        # parameters for lane detecttion 
        self.p = rospy.get_param("proportion", 1.3) 
        self.I = rospy.get_param("integration", 2150)
        
        self.planning = False #for planning process
        self.Qr = False
        self.state = False
        self.recovery_count = 0 #recovery counter to trigger
        self.detect_obstacle = False #detect obstacle to trigger passing around the obstacle
        self.first = ""
        self.second = ""
        self.third = ""
        self.fourth = ""

        self.directLetter =""

        # mqtt definitations
        self.broker = 'broker.emqx.io'
        self.port = 1883
        self.topic = "NCT/mqtt/channel/odom"
        self.topic2 = "NCT/mqtt/channel/odom/terminal"
        self.mqtt_qr = False

        self.client = self.connect_mqtt()
        # mqtt_msg = "True" + "," + "0.0" + "," + "0.0"
        # self.publish(mqtt_msg)

        # path definations
        self.Imu_msg = []
        self.first_load_path = []
        self.first_unload_path = []
        self.second_load_path = []
        self.second_unload_path = []
        self.start_path = []
        self.unload_dict = []

        # defining sub and pub objects
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1000)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_safety)
        self.midPointSub = rospy.Subscriber('/points', linePoint, self.lane_function)
        self.imuSubs = rospy.Subscriber(self.imu, Imu, self.imuFunc)
        self.qrSubs = rospy.Subscriber('/qr_state', qr, self.qrFunc)
        self.position_subs = rospy.Subscriber('/positions', positions, self.positions_parser)
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

    def positions_parser(self, datas):
        self.baslangic = datas.baslangic
        self.first = datas.first_load
        self.second = datas.first_unload
        self.third = datas.second_load
        self.fourth = datas.second_unload

    def subscribe(self, client: mqtt_client):
        message = ''
        def on_message(client, userdata, msg):
            print(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic")
        
        return message

    def connect_mqtt(self) -> mqtt_client:
        def on_connect(client, userdata, flags, rc):
            if rc == 0:
                print("Connected to MQTT Broker!")
            else:
                print("Failed to connect, return code %d\n", rc)

        client = mqtt_client.Client()
        client.on_connect = on_connect
        client.connect(self.broker, self.port)
        return client

    def publish(self, msg1):
        result = self.client.publish(self.topic, msg1)
        status = result[0]
        if status == 0:
            print(f"Send `{msg1}`and to topic `{self.topic}`")
        else:
            print(f"Failed to send message to topic {self.topic}")

    def publish_terminal(self, msg1):
        result = self.client.publish(self.topic2, msg1)
        status = result[0]
        if status == 0:
            print(f"Send `{msg1}`and to topic `{self.topic2}`")
        else:
            print(f"Failed to send message to topic {self.topic2}")

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
            self.publish_terminal("[INFO] lane is detected")
            # linear velocity
            self.hiz = self.linear_vel
            state = True
        else: 
            rospy.logerr("lane is not detected")
            self.publish_terminal("lane is not detected")
            self.hiz = 0.0
            state = False
            while self.recovery_count > 45:
                self.follow_lane(1.6, -0.25, 0.0)
                self.recovery_count = 0

        #controlling areas
        if bolgeler['on1'] < 0.4:
            self.hiz = 0.0
            self.follow_lane(0.01, 0.0, 0.0) #stop the robot
            self.detect_obstacle = True
            self.planning = False
            print("on1")

        if bolgeler['on2'] < 0.4:
            self.hiz = 0.0
            self.follow_lane(0.01, 0.0, 0.0) #stop the robot
            self.detect_obstacle = True
            self.planning = False
            print("on2")

        if bolgeler['on_sol'] < 0.4:
            self.hiz = 0.0
            print("on_sol")

        if bolgeler['on_sag'] < 0.4:
            self.hiz = 0.0
            self.follow_lane(0.01, 0.0, 0.0) #stop the robot
            self.detect_obstacle = True
            self.planning = False
            print("on_sag")

        if bolgeler["sag"] < 0.4:
            # self.hiz = 0.0
            self.follow_lane(0.01, 0.0, 0.0) #stop the robot
            self.detect_obstacle = True
            self.planning = False
            print("sag")

        return self.hiz, state

    # callback for midpoint of the line
    def lane_function(self, points):
        self.avaiable = points.avaiable
        self.cx = points.cx
        self.cy = points.cy
        self.w = points.w

    # path planning service
    def get_directions(self, point1, point2, point3, point4):

        # waiting for service
        rospy.wait_for_service("find_path")
        try:
            path1 = rospy.ServiceProxy("find_path", directions)
            response = path1(point1, point2, point3, point4, "S1")
            self.first_load_path = response.first_load_directions
            self.first_unload_path = response.first_unload_directions
            self.second_load_path = response.second_load_directions
            self.second_unload_path = response.second_unload_directions
            self.start_path = response.go_start
            rospy.logwarn(response)
            self.publish_terminal(str(response))

        except rospy.ServiceException as w:
            # print("service called fialed %s", w)
            rospy.logerr("service called fialed %s. be sure that points are correct", w)
            self.publish_terminal("service called fialed be sure that points are correct")
            self.planning = False

            # waiting for srv server
            rospy.sleep(0.5)

    # adjust the list of directions
    def adjustment_of_directs(self):
        fullPath = self.first_load_path + self.first_unload_path + self.second_load_path + self.second_unload_path + self.start_path
        self.first_unload_dict = self.first_load_path + self.first_unload_path
        self.second_load_dict = self.first_load_path + self.first_unload_path + self.second_load_path
        self.second_unload_dict = self.first_load_path + self.first_unload_path + self.second_load_path + self.second_unload_path

        # control of reaching home point
        if self.index >= len(fullPath):
            self.directLetter = ""
            rospy.loginfo("Goal reached")
            self.publish_terminal("goal reached")

            # re-adjust the initial values
            self.index = 0
            self.planning = False

        # controlling of fullpath inside the directions (if u dont understan this, go into the path_find.py script)
        while self.index < len(fullPath):
            self.directLetter = fullPath[self.index]
            print(f"current direct letter: {self.directLetter}")
            if self.Qr == True:
                self.index += 1
                self.follow_lane(self.wait_after_qr, 0.21, 0.0)

                # first load control
                if self.first_loadPath_counter == 0:
                    # control to reach load and unload positions
                    try:
                        reference = self.first_load_path[self.index]
                    except:
                        rospy.logdebug("first load position has been reached")
                        self.publish_terminal("first load position has been reached")
                        self.follow_lane(10, 0.0, 0.0)
                        self.first_loadPath_counter +=1

                # first unload control
                if self.first_unloadPath_counter == 0:
                    # control to reach load and unload positions
                    try:
                        reference = self.first_unload_dict[self.index]
                    except:
                        rospy.logdebug("first unload position has been reached")
                        self.publish_terminal("first unload position has been reached")
                        self.follow_lane(10, 0.0, 0.0)
                        self.first_unloadPath_counter +=1
                self.Qr = False

                # second load control
                if self.second_loadPath_counter == 0:
                    # control to reach load and unload positions
                    try:
                        reference = self.second_load_dict[self.index]
                    except:
                        rospy.logdebug("second load position has been reached")
                        self.publish_terminal("second load position has been reached")
                        self.follow_lane(10, 0.0, 0.0)
                        self.second_loadPath_counter += 1
                self.Qr = False

                # second unload control
                if self.second_unloadPath_counter == 0:
                    # control to reach load and unload positions
                    try:
                        reference = self.second_unload_dict[self.index]
                    except:
                        rospy.logdebug("second unload position has been reached")
                        self.publish_terminal("second unload position has been reached")
                        self.follow_lane(10, 0.0, 0.0)
                        self.second_unloadPath_counter += 1
                self.Qr = False
            break

    def qrFunc(self, data):
        if data.qr == True:
            rospy.logdebug("Qr detected")
            self.mqtt_qr = True
            self.Qr = True

    def setDemandz(self):
        rospy.logdebug("publishing cmd_vel to adjust orientation")
        self.publish_terminal("publishing cmd_vel to adjust orientation")
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

        if 0 < vel_z < 0.2:
            vel_z = 0.3
        elif -0.2< vel_z <0:
            vel_z = -0.3

        # turn around the robot up to see a line with PID
        self.twist.linear.x = 0.0
        self.twist.angular.z = vel_z
        self.cmd_vel_pub.publish(self.twist)
        rospy.sleep(time) # this is for stablity when robot change it's direction

    def go_around_obstacle(self, ref_area):
        if ref_area['on2'] < 1.0 or ref_area['on_sag'] < 1.0:
            rospy.loginfo("buzzer activated")
            self.publish_terminal("buzzer activated")
            rospy.sleep(5)
            # if ref_area['on2'] < 1.0 or ref_area['on_sag'] < 1.0 and count == 1:
            #     #first is frequency, second linear, third angular velocity
            #     # buraya gülmeyin lütfen
            #     self.follow_lane(2.0, 0.0, 0.5) #turn left  
            #     self.follow_lane(4.5, 0.3, 0.0) #go forward
            #     self.follow_lane(2.0, 0.0, -0.5) #turn right
            #     self.follow_lane(2.0, 0.3, 0.0) #go forward
            #     self.follow_lane(2.0, 0.0, -0.5) #turn right
            #     self.follow_lane(3.5, 0.3, 0.0) #go forward
            #     self.follow_lane(0.65, 0.3, 0.5) #turn left
            self.check_obstacle()
            self.detect_obstacle = True
        else:
            self.detect_obstacle = False
            self.planning = True

    def check_obstacle(self):
        if self.bolgeler['on2'] < 1.0 or self.bolgeler['on_sag'] < 1.0:
            rospy.logdebug("go around")
            #first is frequency, second linear, third angular velocity
            # buraya gülmeyin lütfen
            self.follow_lane(2.0, 0.0, 0.5) #turn left  
            self.follow_lane(4.5, 0.3, 0.0) #go forward
            self.follow_lane(2.0, 0.0, -0.5) #turn right
            self.follow_lane(2.0, 0.3, 0.0) #go forward
            self.follow_lane(2.0, 0.0, -0.5) #turn right
            self.follow_lane(3.5, 0.3, 0.0) #go forward
            self.follow_lane(0.65, 0.3, 0.5) #turn left

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
            self.publish_terminal("planning process is TRUE")

            # connect mqtt server
            mqtt_msg = str(self.mqtt_qr) + "," + str(self.vel_x) + "," + str(self.rot[2])
            self.publish(mqtt_msg)
            if self.mqtt_qr == True:
                self.mqtt_qr = False

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

                # connect mqtt server
                mqtt_msg = str(self.mqtt_qr) + "," + "0.0" + "," + str(self.rot[2])
                self.publish(mqtt_msg)
                if self.mqtt_qr == True:
                    self.mqtt_qr = False

                self.hareket_kontrol(bolgeler=self.bolgeler)
                self.go_around_obstacle(ref_area=self.bolgeler)
                rospy.logwarn("obstacle detected waiting for getting free")
                self.publish_terminal("obstacle detected waiting for getting free")

            else:

                if self.ref == True:
                    rospy.sleep(1)
                    self.twist.linear.x = 0
                    self.twist.angular.z = 0.5
                    self.cmd_vel_pub.publish(self.twist)
                    rospy.sleep(18.2)
                    rospy.logdebug("map sequneced has passed")
                    self.ref = False

                # set to PLANNING=TRUE to start planning process
                self.planning = True
                self.twist.linear.x = 0
                self.twist.angular.z = 0
                self.cmd_vel_pub.publish(self.twist)
 
                # take inputs to create a route for motion planning
                rospy.logdebug("[INFO] Attempting planning process")
                self.publish_terminal("obstacle detected waiting for getting free")

                first = self.first
                second = self.second
                third = self.third
                forth = self.fourth

                # first = input("first")
                # second = input("second")
                # third = input("third")
                # forth = input("forth")

                # waiting for positions
                rospy.sleep(3)

                # input for path planning
                self.get_directions(str(first), str(second), str(third), str(forth))