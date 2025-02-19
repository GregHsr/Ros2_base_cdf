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
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from ament_index_python.packages import get_package_share_directory

from action_msgs.msg import GoalStatus
from cdf_msgs.action import ActionSequencer as ActionSequencerAction

from threading import Thread, Lock
import time
import importlib.util
import sys
import collections
import traceback
import inspect


class CancelReqested(Exception):
    pass


class Action(Thread):
    """
    Action Base Class inheriting from the Thread Class

    This class is defined for internal use only.
    """
    def __init__(self, action_func, env, **kwargs):
        self._action_func = action_func
        self.kwargs = kwargs
        self._env = env
        self._fun_results = None

        super().__init__(name=action_func.__name__,
                         target=self.target,
                         kwargs=kwargs)

    def __repr__(self):
        kwargs_str = ""
        for key, value in self.kwargs.items():
            kwargs_str += "{}={}, ".format(key, value)
        kwargs_str = kwargs_str[:-2]
        return "{}({})".format(self.name, kwargs_str)

    def results(self):
        """ Returns 'action_func' the function results """
        return self._fun_results

    def target(self, **kwargs):
        """ Function executed in the actual Thread """
        self._env.register_curr_action(self)
        try:
            self._fun_results = self._action_func(self._env, **kwargs)
        except Exception:
            # If an Exception occcurs, we first log the error and next abort
            # the currently runned action sequence
            self._env.logger.error(
                "Unhandled exception during the execution of action:'{}'({}):\n{}"
                .format(self._action_func.__name__,
                        self.getName(),
                        traceback.format_exc()
                        )
            )
            if (self._env.get_goal_handler().status
                    != GoalStatus.STATUS_ABORTED):
                self._env.get_goal_handler().abort()

        self._env.unregister_curr_action(self)


class ActionSequence:
    """
    Action Sequence base class

    Attributes
    ----------
    name: str
        Name of the current Action Sequence. This name is used to trigger the
        Actionc Sequence execution when a new Goal is published on the Action
        Sequencer action server.

    parent_node: <Object ActionSequencer>
        Parent of the current object, which corresponds to
        the ActionSequencer node.

    subscribers: dict()
        Before initialisation this dictionnairy contains various tuples, that
        gives required informations about a subscriber. Each of the tuples
        entry have the following structure:
           ("<topic_path>", topic_type, callback_func)

            where:
                - topic_path: Name of the topic to listen to
                - topic_type: Type of the topic messages
                - callback_func: Callback function for the topic

        After Initialization, these tuples will be replaced by the actual
        subscribers instanciated during the Action Sequencer init.

    publishers: dict()
        Before initialisation this dictionnairy contains various tuples, that
        gives required informations about a publisher. Each of the tuples
        entry have the following structure:
            ("<topic_path>", topic_type)

            where:
                - topic_path: Name of the topic to listen to
                - topic_type: Type of the topic messages

        After Initialization, these tuples will be replaced by the actual
        publishers instanciated during the Action Sequencer init.

    logger: <Logger Object>
        Logger to use in the current Action Sequence

    Methods
    -------
    run(*argv)
        Function called automatically when the action sequence is executed. The
        'argv' parameter is a tuple containing all the parameters
        specified in the goal message received by the action server.
        THIS FUNCTION HAS TO BE OVERLOADED

    cancel()
        Function called when the cancellation of the current action sequence
        has been requested or if a runtime error append during an action
        execution. This function can be overloaded when a specific cancel
        procedure has to be used.

    run_action(action_func, blocking=False, **kwargs)
        This function is used to add an action to the action sequence. This
        action is by default runned in parallel but can also be blocking.
        This function returns an 'Action' object which can be used to get the
        results of the action execution or to use it as a dependancy (see
        wait_for() function).

        Note that we used the term blocking, but in fact each action functions
        has to be non-blocking, in order to be sensitive to cancel requests.
        The blocking term we used here is just blocking while we don't receive
        a cancel order.

    wait_for(*argv, timeout=None)
        This function waits for the listed actions to terminate. These actions
        objects can be specified as positionnal arguments. A timeout can also
        be specified to skip the waiting process after a certain amount of time
        has passed.
    """
    def __init__(self, node, goal_handle):
        self.parent_node = node

        self.name = getattr(self, "name", None)

        self.subscribers = getattr(self, "subscribers", dict()).copy()
        self.publishers = getattr(self, "publishers", dict()).copy()

        self._goal_handle = goal_handle
        self._current_actions = []
        self._current_actions_lock = Lock()

        self.logger = \
            node.get_logger().get_child(self.name)

    def cancel(self):
        """
        Function called when the cancellation of the current action sequence
        has been requested or if a runtime error append during an action
        execution. This function can be overloaded when a specific cancel
        procedure has to be used.
        """
        for action in self._current_actions:
            action.join()

    def get_goal_handler(self):
        """ Returns the goal_handle related to the action sequence """
        return self._goal_handle

    def is_cancel_requested(self):
        """
        Returns True if a cancel order has been requested by the client or by
        the server in case of runtime execution errors.
        """
        if self._goal_handle.status == GoalStatus.STATUS_ABORTED:
            return True
        return self._goal_handle.is_cancel_requested

    def run_action(self, action_func, blocking=False, **kwargs):
        """
        This function is used to add an action to the action sequence. This
        action is by default runned in parallel but can also be blocking.
        This function returns an 'Action' object which can be used to get the
        results of the action execution or to use it as a dependancy (see
        wait_for() function).

        Note that we used the term blocking, but in fact each action functions
        has to be non-blocking, in order to be sensitive to cancel requests.
        The blocking term we used here is just blocking while we don't receive
        a cancel order.

        Parameters
        ----------

        action_func: Callable
            Function that represents the action to execute.
        blocking: Bool
            Specifies if the action should be blocking the
            action sequence execution flow.
        kwargs: **kwargs
            Place here all the keyword arguments you want to pass
            to the action_func function.
        """
        self.logger.debug(
            "Action {} (blocking={}) started with kwargs : {}"
            .format(action_func.__name__, blocking, kwargs))
        if self.is_cancel_requested():
            raise CancelReqested
        action = Action(action_func, self, **kwargs)
        action.start()
        if blocking:
            self.wait_for(action)
        return action

    def register_curr_action(self, action):
        """ Register a currently runned action """
        self._current_actions_lock.acquire()

        self._current_actions.append(action)

        feedbck_msg = ActionSequencerAction.Feedback()
        feedbck_msg.current_actions.data = str(self._current_actions)
        self._goal_handle.publish_feedback(feedbck_msg)

        self._current_actions_lock.release()

    def unregister_curr_action(self, action):
        """ Unregister a currently runned action """
        self._current_actions_lock.acquire()

        self._current_actions.remove(action)

        feedbck_msg = ActionSequencerAction.Feedback()
        feedbck_msg.current_actions.data = str(self._current_actions)
        self._goal_handle.publish_feedback(feedbck_msg)

        self._current_actions_lock.release()

    def wait_for(self, *argv, timeout=None):
        """
        This function waits for the listed actions to terminate. These actions
        objects can be specified as positionnal arguments. A timeout can also
        be specified to skip the waiting process after a certain amount of time
        has passed.

        Parameters
        ----------

        argv: *argv
            Place here, as positionnal arguments, all the action to wait for.
        timeout: int
            Specifies the maximum amount of time to wait for a set of actions
            to complete in seconds.
        """
        if len(argv) == 0:
            argv = self._current_actions.copy()
        self.logger.debug("""Waiting for the following actions to complete :
                          {}""".format(argv))
        for action in argv:
            action.join()

        self.logger.debug("""The following actions has been completed :
                          {}""".format(argv))


class ActionSequencer(Node):
    """
    Action Sequence Node

    This Node acts like an action server that responds to client's requests.
    When a new task is received, first the action server examines it and
    determines if it corresponds to an existing Action Sequence.

    If that's the case, the action server will execute the task.
    In meantime, if a new task is received, the server will examine the
    new request like before and put it in a queue.

    Hence, after the first task has been completed, the second will be popped
    out of the queue and then executed.
    """
    def __init__(self):
        """ Action Sequencer Constructor """
        super().__init__('action_sequencer_node')
        self._action_server = ActionServer(
            self,
            ActionSequencerAction,
            'actionsequencer',
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_call,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        self._loaded_action_sequences = dict()

        # Declare parameters
        self.declare_parameter(
            'action_sequence_file',
            get_package_share_directory('action_sequencer') +
            "/config/action_sequence_default.py"
        )
        self.declare_parameter(
            'team_color',
            'None'
        )

        # Getting parameters
        self.action_sequence_file = self.get_parameter('action_sequence_file')\
                                        .get_parameter_value() \
                                        .string_value
        self.team_color = self.get_parameter('team_color') \
                              .get_parameter_value() \
                              .string_value

        # Setting up the goal queue
        self._action_sequence_queue = collections.deque()
        self._action_sequence_lock = Lock()

        self._current_action_sequence = None

        # Load Action Sequence Classes
        self.load_action_sequences(self.action_sequence_file)

        # Extract and Register subscribers and publishers
        # that can be used in the loaded Action Sequences
        self.register_pub_and_sub()

    def handle_accepted_callback(self, goal_handle):
        """ Callback called when a new task has been accepted """
        action_sequence = self._loaded_action_sequences[
            goal_handle.request.order.data](self, goal_handle)
        with self._action_sequence_lock:
            if self._current_action_sequence is not None:
                # Put incoming goal in the queue
                self._action_sequence_queue.append(action_sequence)
                self.get_logger().info('Action Sequence {} queued'
                                       .format(goal_handle.request.order.data))
            else:
                # Start goal execution right away
                self._current_action_sequence = action_sequence
                goal_handle.execute()

    def goal_callback(self, goal_handle):
        """ Callback called when a new goal request has been received """

        # Check if the given order exists
        if goal_handle.order.data not in self._loaded_action_sequences.keys():
            self.get_logger().error(
                """The Requested Action Sequence '{}' does not
                   exist or/and is not currently loaded"""
                .format(goal_handle.order.data)
            )

            return GoalResponse.REJECT

        # Now we check if the given arguments are matching with the
        # signature specification given by the run() function.

        args = inspect.getfullargspec(
            self._loaded_action_sequences[goal_handle.order.data].run)

        mandatory_args_nb = len(args.args or '') - 1 - len(args.defaults or '')
        nb_given = len(goal_handle.argv.data.split())

        # if nb_given < mandatory_args_nb:
        #     self.get_logger().error(
        #         r"The requested Action Sequence '{}' requires {} positional "
        #         r"argument but {} were given."
        #         .format(goal_handle.order.data, mandatory_args_nb, nb_given)
        #     )

        #     return GoalResponse.REJECT

        # if nb_given - len(args.defaults or '') > mandatory_args_nb and args.varargs is None:
        #     self.get_logger().error(
        #         r"The requested Action Sequence '{}' requires {} positional "
        #         r"argument but {} were given."
        #         "\n"
        #         r"Tips: You maybe have to add an '*argv' param to your 'run' function."
        #         .format(goal_handle.order.data, mandatory_args_nb, nb_given)
        #     )

        #     return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def cancel_action_seq_from_queue(self, action_seq):
        """ Threaded function for Canceling a queued task """
        while (action_seq.get_goal_handler().status !=
                GoalStatus.STATUS_CANCELING):
            time.sleep(0.01)

        with self._action_sequence_lock:
            self._action_sequence_queue.remove(action_seq)
        action_seq.get_goal_handler().execute()

    def cancel_call(self, goal_handle):
        """ Callback called when a cancel request has been received """
        with self._action_sequence_lock:
            for action_seq in self._action_sequence_queue:
                if all(action_seq.get_goal_handler().goal_id.uuid ==
                        goal_handle.goal_id.uuid):
                    Thread(target=self.cancel_action_seq_from_queue,
                           args=[action_seq]).start()
                    break
        return CancelResponse.ACCEPT

    def load_action_sequences(self, file_path):
        """
        Loads all the Action Sequence Classes defined in the specified
        python file 'file_path'.
        """
        spec = importlib.util.spec_from_file_location(
            "action_sequences", file_path
        )

        module = importlib.util.module_from_spec(spec)
        sys.modules["action_sequences"] = module
        spec.loader.exec_module(module)

        module_objs = dir(module)

        for element in module_objs:
            obj = getattr(module, element)
            if (obj is not ActionSequence and
                    isinstance(obj, type) and
                    issubclass(obj, ActionSequence)):
                self._loaded_action_sequences[obj.name] = obj

        self.get_logger().info(
            "The following Action Sequences have been loaded: {}"
            .format(self._loaded_action_sequences)
        )

    def register_pub_and_sub(self):
        """
        Register all the Subscribers and Publisher declared in Action
        Sequences.
        """
        publishers = dict()
        subscribers = dict()
        for action_seq in self._loaded_action_sequences.values():
            for publisher in action_seq.publishers.values():
                publishers[publisher[0]] = \
                        self.create_publisher(publisher[1],
                                              publisher[0],
                                              10)
            for subscriber in action_seq.subscribers.values():
                subscribers[subscriber[0]] = \
                        self.create_subscription(subscriber[1],
                                                 subscriber[0],
                                                 lambda msg: None,
                                                 10)

        self.get_logger().info(
            "The following subscribers have been registered: {}"
            .format(subscribers))
        self.get_logger().info(
            "The following publishers have been registered: {}"
            .format(publishers))

        self.global_subscribers = subscribers
        self.global_publishers = publishers

    def bind_pub_and_sub(self, action_sequence):
        """
        Bind the previously registered Subscribers and Publishers with
        the currently executed action sequence.
        """
        def generic_callback(env, callback):
            return (lambda msg: callback(env, msg))

        for key, value in action_sequence.publishers.items():
            if value[0] in self.global_publishers.keys():
                action_sequence.publishers[key] = \
                        self.global_publishers[value[0]]

        for key, value in action_sequence.subscribers.items():
            if value[0] in self.global_subscribers.keys():
                action_sequence.subscribers[key] = \
                    self.global_subscribers[value[0]]
                self.global_subscribers[value[0]].callback = \
                    generic_callback(self._current_action_sequence, value[2])

    async def execute_callback(self, goal_handle):
        """
        Callback called when a new task is executed.

        Sometimes, it is also called when a queued task is canceled to
        be able to return a result.
        """
        result = ActionSequencerAction.Result()

        if goal_handle.is_cancel_requested:
            # In that case the task was already canceled prior to execution,
            # which means that the task was canceled in the queue.
            goal_handle.canceled()

            self.get_logger().warn(
                "Queued Action Sequence '{}' as been Canceled"
                .format(goal_handle.request.order.data)
            )

            result.context.data = "Queue Cancel !"

            return result

        try:
            self.get_logger().info(
                "Execution of Action Sequence : '{}' with args '{}'"
                .format(goal_handle.request.order.data,
                        goal_handle.request.argv.data,
                        )
            )
            argv = goal_handle.request.argv.data.split()
            try:
                self.bind_pub_and_sub(self._current_action_sequence)
                action_seq_return = self._current_action_sequence.run(*argv)
            except CancelReqested:
                action_seq_return = None
            except Exception:
                if goal_handle.status != GoalStatus.STATUS_ABORTED and \
                        goal_handle.status != GoalStatus.STATUS_CANCELING:
                    self.get_logger().error(
                        r"Unhandled exception when running the run function of the"
                        " '{}' Action Sequence:\n{}"
                        .format(goal_handle.request.order.data,
                                traceback.format_exc())
                    )
                    goal_handle.abort()
                action_seq_return = None
            self._current_action_sequence.wait_for()
            if goal_handle.is_cancel_requested:
                self._current_action_sequence.cancel()
                goal_handle.canceled()

                self.get_logger().warn(
                    "Running Action Sequence '{}' as been Canceled"
                    .format(goal_handle.request.order.data)
                )
            elif goal_handle.status == GoalStatus.STATUS_ABORTED:
                self._current_action_sequence.cancel()

                self.get_logger().error(
                    "Action Sequence '{}' encountered an error while running"
                    .format(goal_handle.request.order.data)
                )
            else:
                goal_handle.succeed()

                self.get_logger().info("Action Sequence '{}' Succeeded"
                                       .format(goal_handle.request.order.data))

            if action_seq_return is None:
                result.context.data = "No return data"
            else:
                # Checking if the return data is a string, and if not we
                # print a warning and try to convert it.
                if type(action_seq_return) != str:
                    self._logger.warn("""The action sequence result wasn't an
                                      str object, and has been therefore
                                      automatically converted.""")
                result.context.data = str(action_seq_return)

            for sub in self._current_action_sequence.subscribers.values():
                sub.callback = lambda msg: None
            return result
        finally:
            with self._action_sequence_lock:
                try:
                    # Start execution of the next goal in the queue.
                    self._current_action_sequence = \
                        self._action_sequence_queue.popleft()
                    current_goal = \
                        self._current_action_sequence.get_goal_handler()
                    current_goal.execute()
                except IndexError:
                    # No goal in the queue.
                    self._current_action_sequence = None


def main(args=None):
    rclpy.init(args=args)

    action_sequencer = ActionSequencer()

    # Here we use a MultiThreadedExecutor to be able to handle multiple
    # callbacks and tasks at a time.
    executor = MultiThreadedExecutor()

    rclpy.spin(action_sequencer, executor=executor)
