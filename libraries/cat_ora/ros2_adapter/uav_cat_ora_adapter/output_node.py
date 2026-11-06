"""CAT-ORA output adapter: shared PVA conversion and explicit AS2 stop requests."""
import time
import rclpy
from as2_msgs.msg import ControlMode
from as2_msgs.srv import SetControlMode
from uav_planner_interfaces.msg import BehaviorResult
from uav_as2_setpoint_adapter.node import As2SetpointAdapter


class CatOraAs2Adapter(As2SetpointAdapter):
    def __init__(self):
        super().__init__('cat_ora_as2_adapter')
        self.stop_latched=False;self.stop_reason=None;self.stop_started=0
        self.stop_complete=False
        self.stop_futures={};self.stop_errors={};self.stop_feedback=set();self.previous_status=None
        self.stop_clients={v:self.create_client(SetControlMode,'/'+self.config['deployment'][v]['namespace'].strip('/')+'/controller/set_control_mode') for v in self.vehicles}
        self.create_subscription(BehaviorResult,'planner/status',self.on_status,10)
        self.create_timer(.05,self.check_stop)

    def on_status(self,message):
        if message.task_id!=self.task_id:return
        stopping=(BehaviorResult.PAUSED,BehaviorResult.CANCELED,BehaviorResult.FAILED,BehaviorResult.SUCCEEDED)
        if message.overall_status in stopping and not self.stop_latched:
            self.stop_latched=True;self.latest_setpoints=None
            self.stop_complete=False
            self.stop_reason=int(message.overall_status);self.stop_started=time.monotonic()
        elif message.overall_status==BehaviorResult.RUNNING and self.previous_status==BehaviorResult.PAUSED:
            # Resume still requires a fresh AS2 TRAJECTORY request by the task.
            self.stop_latched=False;self.stop_reason=None;self.stop_futures={};self.stop_errors={};self.stop_feedback=set()
            self.stop_complete=False
            self.latest_setpoints=None;self.setpoints_seen=0
            self.mode_ready={v:False for v in self.vehicles}
        self.previous_status=message.overall_status

    def _publish_if_ready(self):
        if not self.stop_latched:super()._publish_if_ready()
        else:self.latest_setpoints=None

    def _on_mode(self,vehicle,message):
        super()._on_mode(vehicle,message)
        if self.stop_latched and not self.stop_complete and vehicle in self.stop_futures:
            if message.input_control_mode.control_mode in (ControlMode.HOVER,ControlMode.POSITION):
                self.stop_feedback.add(vehicle)
            else:self.stop_feedback.discard(vehicle)

    def check_stop(self):
        if not self.stop_latched or self.stop_complete:return
        for vehicle,client in self.stop_clients.items():
            if vehicle in self.stop_errors:continue
            future=self.stop_futures.get(vehicle)
            if future is None and client.service_is_ready():
                request=SetControlMode.Request();request.control_mode.control_mode=ControlMode.HOVER
                request.control_mode.yaw_mode=ControlMode.YAW_ANGLE
                self.stop_futures[vehicle]=client.call_async(request)
            elif future is not None and future.done():
                try:
                    if not future.result().success:self.stop_errors[vehicle]='AS2 rejected HOVER'
                except Exception as exc:self.stop_errors[vehicle]=str(exc)
            if time.monotonic()-self.stop_started>8 and not self.stop_confirmed(vehicle):
                self.stop_errors[vehicle]='AS2 HOVER response/feedback timeout'
        # Errors are visible in telemetry; no retries that repeatedly reset AS2.
        if all(self.stop_confirmed(v) for v in self.vehicles):self.stop_complete=True

    def stop_confirmed(self,vehicle):
        future=self.stop_futures.get(vehicle)
        return vehicle not in self.stop_errors and vehicle in self.stop_feedback and future is not None and future.done() and future.exception() is None and future.result().success

    def extra_state(self):
        return {'stop_requested':self.stop_latched,'stop_reason':self.stop_reason,
                'stop_confirmed':{v:self.stop_confirmed(v) for v in self.vehicles},'stop_errors':self.stop_errors}


def main(args=None):
    rclpy.init(args=args);node=CatOraAs2Adapter()
    try:rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
