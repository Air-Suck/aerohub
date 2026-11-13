#!/usr/bin/env python3
"""Exercise the actual platform service with synthetic DDS inputs, no PX4/motors.

This is a ROS interface/lifecycle test, not a simulation flight test.
"""
import argparse,json,os,signal,subprocess,time
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import VehicleOdometry, VehicleControlMode, VehicleRatesSetpoint, VehicleCommand
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool
from as2_msgs.msg import AlertEvent


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--case', choices=('normal','shutdown','timeout'), default='normal')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    ns='offboard_callback_test';prefix='/'+ns
    config=args.output/'platform.yaml'
    config.write_text('/**:\n  ros__parameters:\n    use_sim_time: false\n    cmd_freq: 50.0\n    info_freq: 10.0\n    external_odom: false\n    fmu_prefix: '+prefix+'\n    target_system_id: 99\n')
    stream=(args.output/'platform.log').open('w')
    server=subprocess.Popen(['ros2','launch','as2_platform_pixhawk','pixhawk_launch.py',
        'namespace:='+ns,'platform_config_file:='+str(config)],stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init();node=rclpy.create_node('offboard_callback_test_client')
    odom=node.create_publisher(VehicleOdometry,prefix+'/fmu/out/vehicle_odometry',qos_profile_sensor_data)
    modes=node.create_publisher(VehicleControlMode,prefix+'/fmu/out/vehicle_control_mode',qos_profile_sensor_data)
    alerts=node.create_publisher(AlertEvent,prefix+'/alert_event',10)
    receipts=[];rates=[];commands=[];subscriptions=[]
    subscriptions.append(node.create_subscription(Odometry,prefix+'/sensor_measurements/odom',lambda m:receipts.append(time.monotonic()),qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(VehicleRatesSetpoint,prefix+'/fmu/in/vehicle_rates_setpoint',lambda m:rates.append(time.monotonic()),qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(VehicleCommand,prefix+'/fmu/in/vehicle_command',lambda m:commands.append((time.monotonic(),m.command)),qos_profile_sensor_data))
    def tick():
        msg=VehicleOdometry();msg.timestamp=msg.timestamp_sample=time.time_ns()//1000
        msg.pose_frame=VehicleOdometry.POSE_FRAME_NED;msg.velocity_frame=VehicleOdometry.VELOCITY_FRAME_NED;msg.q=[1.,0.,0.,0.];odom.publish(msg)
        status=VehicleControlMode();status.timestamp=time.time_ns()//1000;modes.publish(status)
    timer=node.create_timer(.01,tick)
    control=node.create_client(SetBool,prefix+'/set_offboard_mode')
    arming=node.create_client(SetBool,prefix+'/set_arming_state')
    def wait(predicate,seconds=5):
        end=time.monotonic()+seconds
        while not predicate():
            if server.poll() is not None:raise RuntimeError('platform exited')
            if time.monotonic()>end:raise TimeoutError('test deadline exceeded')
            rclpy.spin_once(node,timeout_sec=.005)
    def spin(seconds):
        end=time.monotonic()+seconds;wait(lambda:time.monotonic()>=end,seconds+1)
    report={'scope':'actual_platform_synthetic_DDS_interface','passed':False}
    try:
        wait(lambda:control.service_is_ready() and arming.service_is_ready(),15)
        wait(lambda:len(receipts)>30,5)
        if args.case in ('shutdown','timeout'):
            commands.clear()
            pending=control.call_async(SetBool.Request(data=True));spin(.1)
            if args.case=='shutdown':
                started=time.monotonic();os.killpg(server.pid,signal.SIGINT)
                server.wait(timeout=5)
                end=time.monotonic()+1.3
                while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
                assert not any(command==176 for _,command in commands),'shutdown emitted a late switch'
                report.update(passed=True,shutdown={'exit_s':time.monotonic()-started-1.3,'mode_commands':0})
            else:
                # Pause only this synthetic test's actual platform child, not the launch wrapper.
                children=Path(f'/proc/{server.pid}/task/{server.pid}/children').read_text().split()
                owned=[int(pid) for pid in children if b'as2_platform_pixhawk_node' in Path('/proc',pid,'cmdline').read_bytes()]
                assert len(owned)==1,'cannot identify owned platform child'
                pid=owned[0];os.kill(pid,signal.SIGSTOP)
                try:
                    end=time.monotonic()+3.2
                    while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
                finally:os.kill(pid,signal.SIGCONT)
                wait(pending.done)
                assert not pending.result().success,'interrupted prestream should time out'
                spin(.3)
                assert not any(command==176 for _,command in commands),'expired handshake emitted a switch'
                report.update(passed=True,timeout={'response':pending.result().message,'mode_commands':0})
            return
        start=time.monotonic();rates.clear();commands.clear();receipts.clear()
        first=control.call_async(SetBool.Request(data=True));spin(.1)
        duplicate=control.call_async(SetBool.Request(data=True));wait(duplicate.done)
        assert not duplicate.result().success,'pending duplicate should be rejected'
        manual=control.call_async(SetBool.Request(data=False));wait(manual.done)
        assert not manual.result().success,'manual-mode refusal must be preserved'
        wait(first.done)
        elapsed=time.monotonic()-start;assert first.result().success
        assert elapsed>=1.2,'prestream duration was shortened'
        assert len(rates)>=120,'prestream sample count was shortened'
        assert max((b-a for a,b in zip(receipts,receipts[1:])),default=999)<.1,'odometry blocked during handshake'
        assert sum(command==176 for _,command in commands)==1,'duplicate emitted a second mode switch'
        report['normal']={'response_s':elapsed,'prestream_samples':len(rates),'max_odom_gap_s':max(b-a for a,b in zip(receipts,receipts[1:]))}
        spin(.1)  # synthetic control feedback keeps platform disarmed/non-Offboard
        pending=control.call_async(SetBool.Request(data=True));spin(.1)
        cancel=arming.call_async(SetBool.Request(data=False));wait(cancel.done);wait(pending.done)
        assert not pending.result().success,'disarm must cancel pending handshake even when already disarmed'
        count=sum(command==176 for _,command in commands);spin(1.3)
        assert sum(command==176 for _,command in commands)==count,'canceled handshake issued a late command'
        report['disarm_cancels']=True
        pending=control.call_async(SetBool.Request(data=True));spin(.1)
        alerts.publish(AlertEvent(alert=AlertEvent.KILL_SWITCH,description='isolated interface test'))
        wait(pending.done);assert not pending.result().success,'kill must cancel pending handshake'
        retry=control.call_async(SetBool.Request(data=True));wait(retry.done)
        assert not retry.result().success,'kill must inhibit a new handshake'
        report['kill_cancels_and_inhibits']=True
        report['passed']=True
    except BaseException as exc:
        report['error']=str(exc);raise
    finally:
        (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        timer.cancel();node.destroy_node();rclpy.try_shutdown()
        try:os.killpg(server.pid,signal.SIGINT)
        except ProcessLookupError:pass
        try:server.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(server.pid,signal.SIGKILL);server.wait(timeout=3)
        stream.close()
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
