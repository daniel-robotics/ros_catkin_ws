import rospy
import std_msgs.msg

class TemplateNode:
    #********************************************************************************************** 
    #   Init
    #   Starts a ROS node with a given name
    #********************************************************************************************** 
    def __init__(self, name, params=None):
        self.name = name

        # Start node and register shutdown hook
        rospy.init_node(self.name)    
        self.logwarn("Initializing node...", force=True)
        rospy.on_shutdown(self.shutdown) 

        # Read parameters from ROS parameter server
        if params is None:
            self.params = rospy.get_param("~")
        else:
            self.params = params

        # Documentation on publishers and subscribers: http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers
        self.loginfo("Starting publisher on \"template_topic\"")
        self.publisher = rospy.Publisher(name="template_topic",             # name: topic to publish messages to
                                         data_class=std_msgs.msg.UInt32,    # data_class: type of message to publish
                                         queue_size=10,                     # queue_size: messages to queue before dropping old messages
                                         latch=False)

        self.subscriber = rospy.Subscriber(name="template_topic",
                                           data_class=std_msgs.msg.UInt32,
                                           callback=self.subscriber_callback)


        self.logwarn("Node Initialized", force=True)

        if start is True:
            # Runs self.loop() at the rate set by the parameter, until ROS shuts down or CTRL-C is pressed in the terminal
            self.loginfo("Looping at rate %d Hz..." % (self.params.rate))
            self.iterations = 0
            rate = rospy.Rate(self.params.rate)
            while not rospy.is_shutdown():
                self.iterations += 1
                self.loop()
                rate.sleep()
        else:
            # Idle until shutdown (still runs subscriber callbacks)
            self.loginfo("Idling until shutdown...")
            rospy.spin()


    #********************************************************************************************** 
    #   MAIN LOOP (change as necessary)
    #   Function is called at the rate defined in the launch file (or via command line)
    #********************************************************************************************** 
    def loop(self):
        msg = std_msgs.msg.UInt64()
        msg.data = self.iterations
        self.publish(msg)
    
     
    #********************************************************************************************** 
    #   PUBLISHER
    #   Publish a message to the topic defined in __init__
    #********************************************************************************************** 
    def publish(self, msg=std_msgs.msg.UInt64()):
        self.publisher.publish(msg)
        self.loginfo("Publishing value: %d" % self.iterations)


    #********************************************************************************************** 
    #   SUBSCRIBER
    #   Function is called when a message is published to the topic defined in __init__
    #********************************************************************************************** 
    def subscriber_callback(self, msg):
        self.last_received_msg=msg 
        self.loginfo("Received value: %d" % msg.data)


    #********************************************************************************************** 
    #   LOGGING
    #********************************************************************************************** 
    # Logs a Message string to the console (only if parameter "verbose" is True or force=True is provided)
    def loginfo(self, msg, force=False):
        if self.params.verbose is True or force is True:
            rospy.loginfo("[%s] %s" % (self.name, msg) )
    
    # Logs a Warning string to the console (only if parameter "verbose" is True or force=True is provided)
    def logwarn(self, msg, force=False):
        if self.params.verbose is True or force is True:
            rospy.logwarn("[%s] %s" % (self.name, msg) )
   
    # Logs an Error string to the console (only if parameter "verbose" is True or force=True is provided)
    def logerr(self, msg, force=False):
        if self.params.verbose is True or force is True:
            rospy.logerr("[%s] %s" % (self.name, msg) )


    #********************************************************************************************** 
    #   SHUTDOWN HOOK
    #   Runs when ROS is shutting down or if CTRL-C is pressed in the terminal
    #********************************************************************************************** 
    def shutdown(self):
        self.logwarn("Terminating node...", force=True)
        try:
            rospy.delete_param("~")
        except: 
            self.logerr("Could not delete private parameters", force=True)
        
