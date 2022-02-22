from typing import Any, Optional, Tuple, List
import json
from predicates.state import State
from model.model import from_goal_to_goal, the_model, Model
from planner.plan import plan
from model.operation import Operation
from predicates.state import State
from predicates.errors import NotInStateException
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.publisher import Publisher
from ur_tools_msgs.action import URScriptControl
from std_srvs.srv import Trigger
# from viz_tools_msgs.srv import ManipulateDynamicMarker
import random


import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool

# ---------------------------------------------------------------------------
# ...
# ---------------------------------------------------------------------------


runner_goal: str = "runner_goal"
runner_plan: str = "runner_plan"
step_in_plan: str = "step_in_plan"
plan_status: str = "plan_status"

# publish a goal:

class Runner(Node):
    # marker_done: bool = False
    cube_id = 1
    lock_done = False
    lock_run = False
    
    def __init__(self):
        super().__init__('the_runner')
        self.model: Model = the_model()
        self.state: State = self.model.initial_state
        self.prev_state = self.state
        self.lock_waiting = False
        self.upd_state(runner_goal, None)
        self.upd_state(runner_plan, None)
        self.upd_state(step_in_plan, None)
        self.upd_state(plan_status, None)

        self.ur_robot_action_client = ActionClient(self, URScriptControl, '/ur_script_controller')
        self.robot_action_goal_handle: Optional[ClientGoalHandle] = None

        ## We will not use the goal topic. Should be defind using the state
        self.create_subscription(
            msg_type = String,
            topic = 'set_state',
            callback = self.set_state_callback,
            qos_profile = 10)

        self.pub_state: Publisher = self.create_publisher(
            msg_type=String,
            topic = 'state',
            qos_profile = 10,
        )

        self.create_subscription(
            msg_type = String,
            topic = '/opc_measured',
            callback = self.set_opc_callback,
            qos_profile = 10)

        self.create_subscription(
            msg_type = String,
            topic = '/hands_gesture',
            callback = self.gesture_callback,
            qos_profile = 10)

        self.pub_opc: Publisher = self.create_publisher(
            msg_type=String,
            topic = '/opc_command',
            qos_profile = 10,
        )

        self.timer = self.create_timer(0.1, self.ticker)
    
    def set_state_callback(self, msg: String):
        """
        Here you can send in state changes from outside
        """
        try:
            j = msg.data.replace('\'', '\"')
            kvs: dict[str, Any] = json.loads(j)
            print(f"got a state change: {kvs}")
            self.state = self.state.next(**kvs)
        except TypeError:
            pass
        except json.decoder.JSONDecodeError as e:
            print(f"message is bad: {msg.data}")
            print(e)

    def gesture_callback(self, msg: String):
        """
        listen to gestures
        """
        self.upd_state('gesture', msg.data)

    def set_opc_callback(self, msg: String):
        """
        Here the state from te PLC comes in
        """
        try:
            j = msg.data.replace('\'', '\"')
            kvs: dict[str, Any] = json.loads(j)

            fixed = {k.replace("ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.", ""): v for k,v in kvs.items()}

            kv = {
                "bool_from_plc_1": fixed["bool_from_plc_1"],
                "bool_from_plc_2": fixed["bool_from_plc_2"],
                "bool_from_plc_3": fixed["bool_from_plc_3"],
                "bool_from_plc_4": fixed["bool_from_plc_4"],
                "bool_from_plc_5": fixed["bool_from_plc_5"],
                "int_from_plc_1": fixed["int_from_plc_1"],
                "int_from_plc_2": fixed["int_from_plc_2"],
                "int_from_plc_3": fixed["int_from_plc_3"],
                "int_from_plc_4": fixed["int_from_plc_4"],
                "int_from_plc_5": fixed["int_from_plc_5"],
            }
            # print(f"we got: {fixed}")
            self.state = self.state.next(**kv)
        except TypeError as e:
            print(f"BAD: from opc: {e}")
        except json.decoder.JSONDecodeError as e:
            print(f"message is bad: {msg.data}")
            print(e)

            

    def send_ur_action_goal(self):
        run: bool = self.state.get('robot_run')
        if not run and self.robot_action_goal_handle is not None:
            self.robot_action_goal_handle.cancel_goal()  # maybe do it async?
            print("Cancel of robot action done")
            self.robot_action_goal_handle = None
            self.upd_state('robot_state', "initial")
        elif not run:
            self.upd_state('robot_state', "initial")
        elif run and self.robot_action_goal_handle is None: 
            print("start action")
            goal_msg = URScriptControl.Goal()
            goal_msg.command = self.state.get('robot_command')
            goal_msg.velocity = self.state.get('robot_velocity')
            goal_msg.acceleration = self.state.get('robot_acceleration')
            goal_msg.goal_feature_name = self.state.get('robot_goal_frame')
            goal_msg.tcp_name = self.state.get('robot_tcp_frame')

            print("waiting action")
            if self.ur_robot_action_client.wait_for_server(2):
                print("done waiting action")
                send_goal_future = self.ur_robot_action_client.send_goal_async(goal_msg)
                send_goal_future.add_done_callback(self.ur_action_goal_response_callback)
            else:
                print("timeout action")


    def ur_action_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.upd_state('robot_state', "failed")
            self.robot_action_goal_handle = None
            return

        self.robot_action_goal_handle = goal_handle
        self.get_logger().info('Goal accepted :)')
        self.upd_state('robot_state', "exec")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.ur_action_get_result_callback)


    def ur_action_get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        self.upd_state('robot_state', "done")
        self.upd_state('robot_pose', self.state.get('robot_goal_frame'))
        self.robot_action_goal_handle = None


    def lock_marker_service(self):
        run: bool = self.state.get('lock_run')
        if run and not self.lock_done and not self.lock_waiting:
            self.lock_waiting = True
            print("waiting for service")
            service = self.create_client(Trigger, "/lock_arucos")
            service.wait_for_service()
            print("Service ready")
            msg = Trigger.Request()

            resp = service.call_async(msg)
            resp.add_done_callback(self.locked_done_callback)
            
        elif not run and self.lock_done:
            self.lock_done = False

        self.upd_state('lock_done', self.lock_done)
    
    def locked_done_callback(self, future):
        print("WE LOCKED IT")
        self.lock_waiting = False
        result = future.result().success
        if not result:
            print(f"The locked service did not like the call. Check log in simulator window")
        self.lock_done = result
        self.upd_state('lock_done', self.lock_done)


    def send_to_opc(self):
        kv = {
            "bool_to_plc_1": self.state.get("bool_to_plc_1"),
            "bool_to_plc_2": self.state.get("bool_to_plc_2"),
            "bool_to_plc_3": self.state.get("bool_to_plc_3"),
            "bool_to_plc_4": self.state.get("bool_to_plc_4"),
            "bool_to_plc_5": self.state.get("bool_to_plc_5"),
            "int_to_plc_1": self.state.get("int_to_plc_1"),
            "int_to_plc_2": self.state.get("int_to_plc_2"),
            "int_to_plc_3": self.state.get("int_to_plc_3"),
            "int_to_plc_4": self.state.get("int_to_plc_4"),
            "int_to_plc_5": self.state.get("int_to_plc_5"),
        }
        kv = {f"ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.{k}": v for k, v in kv.items()}
        j = json.dumps(kv)
        self.pub_opc.publish(String(data = j))


    def upd_state(self, key: str, value):
        self.state = self.state.next(**{key: value})

    def ticker(self):
        g = self.state.get(runner_goal)
        p = self.state.get(runner_plan)
        replan = self.state.get("replan")
        replanned = self.state.get("replanned")
            
        if replan and not replanned:
            self.upd_state("replanned", True)
            goal = from_goal_to_goal(self.state)
            print(f"The goal: {goal}")
            new_p = plan(self.state, goal, self.model, 30)
            if new_p == None:
                print(f"Could not find a plan to goal: {goal}")
            elif len(new_p) == 0:
                print(f"We are already in goal: {goal}")
            else:
                self.upd_state(runner_plan, new_p)
                print(f"The new goal: {goal}")
                print(f"and computed this plan: {new_p}")
        
        if not replan:
            self.upd_state("replanned", False)


        if self.prev_state != self.state:
            print(f"")
            for k, v in self.state.items():
                print(f"{k}: {v}")
            print(f"")
    
        self.prev_state = self.state

        # here we call the ticker. Change the pre_start parameter to true when
        # you want to prestart
        self.state = tick_the_runner(self.state, self.model, True)

        
        

        # below, we are publishing the command variables to the simulation via ros
        self.send_ur_action_goal()
        # self.send_marker_service()
        self.lock_marker_service()
        state_json = json.dumps(self.state.state)
        self.pub_state.publish(String(data = state_json))
        self.send_to_opc()
        


def tick_the_runner(state: State, model: Model, pre_start: bool) -> State:
    """
    This function will run the operations based on a plan that are located in the state
    This will just execute one transition at the time
    """

    # Here you can execute the free transitions by checking if they are enabled and then do next on them

    for t in model.transitions:
        if t.eval(state):
            state = t.next(state)

    the_plan: list[str] = state.get(runner_plan)    
    if not the_plan:
        return state.next(plan_status="No plan in state", runner_plan = None, step_in_plan = None)
    
    current_step_in_plan: int = state.get(step_in_plan)
    if not current_step_in_plan:
        # we have not started executing the plan so we start at position 0 in the plan
        current_step_in_plan = 0
        state = state.next(**{step_in_plan: current_step_in_plan})
    
    plan_length = len(the_plan)
    if plan_length <= current_step_in_plan:
        # we are done with the plan and will stop executing and we also
        # reset the current plan so we do not tries to run the same plan again
        return state.next(plan_status="done", runner_plan = None, step_in_plan = None)

    # check what operation we are / should be executing
    current_op_name = the_plan[current_step_in_plan]
    current_op_state: str = state.get(current_op_name)
    current_op: Operation = model.operations[current_op_name]

    next_step = current_step_in_plan + 1

    if current_op_state == "i" and current_op.eval_run(state): # The operation can be started
        next_state = current_op.start(state)
    elif current_op_state == "i": # the operation should be started but is not enabled
        next_state = state.next(plan_status=f"waiting for op {current_op_name} to be enabled. pre: {current_op.precondition} and {current_op.to_run}")
    elif current_op.is_completed(state): # the operation has completed and we can take a step in the plan
        next_state = current_op.complete(state)
        next_state = next_state.next(step_in_plan=next_step, plan_status=f"completing step {current_step_in_plan}")
    elif current_op_state == "e": # the operation is executing, let's check if we can prestart the next
        if not pre_start:
            next_state = state.next(plan_status=f"waiting for op to complete")
        elif plan_length > next_step and model.operations[the_plan[next_step]].eval_run(state):
            next_state = model.operations[the_plan[next_step]].start(state).next(
                plan_status=f"pre_starting {next_step}"
            )
        else:
            next_state = state
    else:
        next_state = state.next(plan_status="doing nothing")
    
    return next_state


def tick_the_random_runner(state: State, model: Model) -> State:
    """
    This function will run the operations based on a plan that are located in the state
    This will just execute one transition at the time
    """

    running_ops: List[Operation] = [o for name, o in model.operations.items() if state.get(name) == "e"]
    next_state = state

    if not running_ops:
        enabled_ops = [o for _, o in model.operations.items() if o.precondition.eval_run(state)]
        if not enabled_ops:
            print("No operations are enabled or running in this state!")
            return state
        
        o = random.choice(enabled_ops)
        next_state = o.start(state)
        print(f"Operation {o.name} started!")

    else:
        ops_can_complete = [o for o in running_ops if o.postcondition.eval_run(state)]
        if ops_can_complete:
            next_state = state
            for o in ops_can_complete:
                next_state = o.complete(next_state)
                print(f"Operation {o.name} completed!")

    return next_state


def run():
    rclpy.init()
    runner = Runner()
    rclpy.spin(runner)
    runner.destroy_node()
    rclpy.shutdown()

