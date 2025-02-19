"""
 /$$$$$$$$ /$$$$$$$            /$$                   /$$
|_____ $$/| $$__  $$          | $$                  | $$
     /$$/ | $$  \ $$  /$$$$$$ | $$$$$$$   /$$$$$$  /$$$$$$
    /$$/  | $$$$$$$/ /$$__  $$| $$__  $$ /$$__  $$|_  $$_/
   /$$/   | $$__  $$| $$  \ $$| $$  \ $$| $$  \ $$  | $$
  /$$/    | $$  \ $$| $$  | $$| $$  | $$| $$  | $$  | $$ /$$
 /$$/     | $$  | $$|  $$$$$$/| $$$$$$$/|  $$$$$$/  |  $$$$/
|__/      |__/  |__/ \______/ |_______/  \______/    \___/
Copyright (c) 2023/2024, 7Robot
----
"""

import rclpy
from rclpy.wait_for_message import wait_for_message
from rclpy.node import Node
from rclpy.action import ActionClient
from cdf_msgs.action import ActionSequencer as ActionSequencerAction
from cdf_msgs.msg import PicAction
import rclpy.wait_for_message
from std_srvs.srv import Trigger
from std_msgs.msg import Bool, String, Int16
import importlib
from ament_index_python.packages import get_package_share_directory
import inspect
import time


class BigBrother(Node):
    def __init__(self):
        super().__init__('big_brother_node')

        # Declare parameters
        self.declare_parameter('action_serv_topic', 'actionsequencer')

        # Getting parameters
        self.action_serv_topic = self.get_parameter('action_serv_topic') \
                                     .get_parameter_value() \
                                     .string_value

        # Creating health service server
        self.health_srv = self.create_service(Trigger, "big_brother/health",
                                              self.health_handler
                                              )

        # Creating action client instance
        self._action_client = ActionClient(self, ActionSequencerAction,
                                           self.action_serv_topic)

        self.action_pub = self.create_publisher(PicAction, 'action', 10)

        # Check if we can connect to the specified action server
        if not self._action_client.wait_for_server(3):
            self.get_logger().fatal("""Unable to connect to the specified
                                    action server.
                                    Maybe the specified topic is incorrect
                                    """)
            rclpy.shutdown()
            return

        self.get_logger().info("Connection to the action server succedded !")

        # Creating the service for the script name and team color
        self.script_name_srv = self.create_client(Trigger, 'init/script_name')

        # Creating the subscriber for the team color
        self.team_color_sub = self.create_subscription(
            String,
            'init/team_color',
            self.team_color_cb,
            10
        )
        self.score_pub = self.create_publisher(
            Int16,
            "score",
            10
        )

        # Check the connection
        while not self.script_name_srv.wait_for_service(3):
            self.get_logger().warn("""Unable to connect to the specified
                                    service.
                                    Retrying...
                                    """)

        self.get_logger().info("Connection to the script name service succedded !")

        # Init Class's attributes
        self.script = None
        self.team_color = None

        # Run the init phase
        self.init_phase()

        # Wait for the leach to be released
        while not wait_for_message(Bool, self, "big_brother/start")[1].data:
            self.get_logger().warn("Startup sequencer restart requested")
            self.init_phase()

        # Run the script
        self.script.start_timers()
        self.script.run()

    def team_color_cb(self, msg):
        self.team_color = msg.data

    def init_phase(self):
        self.script = None
        self.team_color = None
        self.health = "Bad"

        # Reset the team selected for the user card
        msg = PicAction()
        msg.action_destination = "USER"
        msg.action_msg = "teamnone"
        self.action_pub.publish(msg)

        # Load the provided script
        self.script_name = self.get_script_name()

        # Check the script's content
        while not self.load_script():
            self.get_logger().error(
                "The requested script %s is incorrect nor existing !" %
                self.script_name
            )
            self.script_name = self.get_script_name()

        self.health = "HEALTHY"
        self.get_logger().info("The script %s has been successfully loaded !" %
                               self.script_name)

        # Rearm the pic cards timers
        msg = PicAction()
        msg.action_destination = "CAN"
        msg.action_msg = "restarttimer"
        self.action_pub.publish(msg)
        msg.action_destination = "USER"
        self.action_pub.publish(msg)
        msg.action_destination = "SERVO"
        self.action_pub.publish(msg)

        # Get the color of the team
        self.team_color = self.get_team_color()
        self.get_logger().info("The color of the team is '%s'" %
                               self.team_color)

        # Set the team selected for the user card
        msg = PicAction()
        msg.action_destination = "USER"
        msg.action_msg = "teamj" if self.team_color == "YELLOW" else "teamb"
        self.action_pub.publish(msg)

    def health_handler(self, request, response):
        if self.health != "HEALTHY":
            response.message = "No valid script"
            response.success = False
        else:
            response.success = True

        return response

    def get_team_color(self):
        while self.team_color is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        return self.team_color

    def get_script_name(self):
        while rclpy.ok():
            self.script_name_srv.wait_for_service(3.0)
            request = Trigger.Request()
            future = self.script_name_srv.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            res = future.result()
            if res.success:
                return res.message
            time.sleep(0.1)

    def load_script(self):
        def check_class_content(script_class):
            try:
                attrs_to_check = ['run']
                for attrs in attrs_to_check:
                    getattr(script_class, attrs)
                return True
            except AttributeError:
                self.get_logger().error("The %s attribute doesn't exist." %
                                        attrs)
                return False

        spec = importlib.util.spec_from_file_location(
            "script", self.script_name)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        is_ok = False
        objs = dir(module)

        for obj in objs:
            obj = getattr(module, obj)
            if not inspect.isclass(obj):
                continue
            if not issubclass(obj, Script):
                continue
            is_ok = check_class_content(obj)
            self.script = obj(self)
            break
        if not is_ok:
            return False

        return True


class Script():
    def __init__(self, node):
        self._node = node
        self.logger = node.get_logger()
        self.goal_handler = None
        self.endofmatch = False
        self.score = 0

    def start_timers(self):
        # Check if the pre_endofmatch_handler method is defined
        if hasattr(self, 'pre_endofmatch_handler') and callable(self.pre_endofmatch_handler):
            # Check if the endofmatch_time attribute is defined
            if getattr(self, 'endofmatch_time', None) is not None:
                self.endofmatch_timer = self._node.create_timer(self.endofmatch_time,
                                                                lambda : self.timeout_handler(self.pre_endofmatch_handler, True)
                                                                )

        # Tell the pic cards to start their timers
        msg = PicAction()
        msg.action_destination = "CAN"
        msg.action_msg = "starttimer"
        self._node.action_pub.publish(msg)
        msg.action_destination = "USER"
        self._node.action_pub.publish(msg)
        msg.action_destination = "SERVO"
        self._node.action_pub.publish(msg)

    def set_score(self, score):
        self.score = int(score)
        self._node.score_pub.publish(Int16(data=score))

    def add_score(self, score):
        self.score += int(score)
        self._node.score_pub.publish(Int16(data=self.score))

    def cancel_result_clb(self, future):
        """
        Function called when a cancel request response has been received from
        the server.
        """
        self._node.destroy_timer(self.timer)

    def timeout_handler(self, callback, endofmatch=False):
        """
        Function called when the timeout for the action to complete elapsed
        """
        if endofmatch:
            self.logger.error("End of match reached, cancelling the goal...")
            self.endofmatch = True
        if self.goal_handler is not None:
            self.logger.error("Timeout reached, cancelling the goal...")
            cancel_goal = self.goal_handler.cancel_goal_async()
            cancel_goal.add_done_callback(lambda __: callback())
        else:
            callback()

    def doActionSeq(self, action_name, args="", timeout=None, endofmatch=False):
        """
        The doActionSeq method can be used within a script to send an action to
        an action server.
        A call of doActionSeq is synchronous, and therefore is blocking.

        Different parameters can be passed to the doActionSeq function in order
        to set a timeout or provide extra-arguments to the action.

        Parameters
        ----------

        action_name: str
            Name of the action to be executed.
        args: str
            String that represents all of the arguments to be passed to the
            action.
        timeout: int
            The timeout is used to set a maximum execution time for the action.
            By default, the value is set to None, and, therefore, the function
            will only return when the requsted action completes.
        """
        if not endofmatch and self.endofmatch:
            rclpy.spin(self._node)

        # Add the timeout timer connected to the correct handler
        if timeout is not None:
            self.timer = self._node.create_timer(timeout, lambda: self.timeout_handler(self.cancel_result_clb))

        goal_msg = ActionSequencerAction.Goal()
        goal_msg.order.data = action_name
        goal_msg.argv.data = args

        self._node._action_client.wait_for_server()

        goal_request = self._node._action_client.send_goal_async(goal_msg)

        # Wait to receive ACK from the action server
        rclpy.spin_until_future_complete(self._node, goal_request)

        self.goal_handler = goal_request.result()

        if not self.goal_handler.accepted:
            self.logger.fatal("""
                              The Requested action has been rejected :(
                              Check the action_sequencer log to know more about
                              it.
            """)
            if timeout is not None:
                self._node.destroy_timer(self.timer)

            return self.goal_handler

        goal_res_future = self.goal_handler.get_result_async()

        rclpy.spin_until_future_complete(self._node, goal_res_future)

        result = goal_res_future.result().result
        status = goal_res_future.result().status

        if timeout is not None:
            self._node.destroy_timer(self.timer)

        self.goal_handler = None

        self.logger.info("""Task '{}' with args '{}', terminated with exit code
                         '{}' and result message '{}'"""
                         .format(action_name, args, status,
                                 str(result.context.data))
                         )

        return [status, result]


def main(args=None):
    rclpy.init(args=args)

    big_brother = BigBrother()