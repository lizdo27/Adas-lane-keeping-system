#!/usr/bin/env python3
"""ADAS CAR: manual throttle/brake, camera lane-departure steering assistance."""
import argparse
from copy import copy
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import threading
import time
from concurrent.futures import ThreadPoolExecutor
import cv2
import numpy as np
from adas_dashboard import render_dashboard
from driver_input import ManualDrive
from adas_control import LaneAssist
from vroom_lane import Config,detect,annotate
from adas_lane_choice import prefer_right_lane
from vroom_tracking import LaneTracker
from vroom_recording import DiagnosticRecorder

BASE=Path(__file__).resolve().parent

def guidance_line(info,driver):
    # MANUAL/UNAVAILABLE snapshots deliberately contain no steering plan.
    # Missing guard is not evidence that the camera saw only one boundary.
    if info['state']=='MANUAL':return 'No steering target: '+info['reason']
    if info['state']=='UNAVAILABLE':return 'No steering target: waiting for usable fresh lane'
    target=f"Target {info.get('desired_steer_deg',driver['steer']):+.1f} deg | "
    if info.get('guidance_mode')=='FRONT_EDGE_RECOVERY':
        if info.get('guard')!='both':return target+'Recover '+info.get('side','?')+' edge; opposite unknown'
        return target+'Recover front edge, then unwind'
    if info.get('guard')=='both':
        return target+f"Preview offset {info.get('preview_offset_m',0)*100:+.1f} cm / heading {info.get('preview_heading_deg',0):+.1f} deg"
    if info.get('guard') in ('left','right'):return target+'EDGE ONLY: '+info['guard']+' boundary'
    return 'No reliable steering target'

def alert_line(info):
    alert=info.get('lane_alert',{});state=alert.get('state','UNKNOWN')
    side={'left':'TRAI','right':'PHAI'}.get(alert.get('side'),'?')
    if state=='ESTIMATED_OVERLAP':return 'CANH BAO: UOC TINH LAN VACH '+side+' (PHIA TRUOC)',(0,0,255)
    if state=='NEAR_EDGE':return 'SAT VACH '+side+' - UOC TINH PHIA TRUOC',(0,190,255)
    if info.get('residual_risk'):return 'RISK REMAINS: driver must correct / brake',(0,190,255)
    if state=='UNKNOWN':return 'CHUA DU DU LIEU XAC NHAN DANG LAN VACH',(0,190,255)
    if not alert.get('paired'):return 'Observed front edge: clear | other edge / rear unknown',(210,230,210)
    return 'Front estimate: clear | rear tyre contact not assessed',(210,230,210)

def draw_panel(image,info,driver,sent):
    panel=np.zeros((192,image.shape[1],3),np.uint8)
    warning,warning_color=alert_line(info)
    lines=[f"ADAS CAR v10 | {info['state']} | {'ON' if info['enabled'] else 'OFF'}",
           ('NO ASSIST: ' if info['state']=='UNAVAILABLE' else '')+info['reason'],
           f"Driver {driver['steer']:+.1f} deg -> Sent {sent['steer']:+.1f} deg | speed {sent['speed']:+.3f}",
           'W/A/D hold: drive/turn | S: reverse | Space: brake',
           'H: assist ON/OFF | R: record | T: finish | Q: quit',
           warning,
           guidance_line(info,driver),
           f"Image age {info.get('image_age_ms',0):.0f}/800 ms | command compute {sent.get('compute_ms',0):.1f} ms"]
    color=(0,190,255) if info['state'] in ('ASSIST','WARNING') else ((0,0,255) if info['state']=='UNAVAILABLE' else (210,230,210))
    for i,line in enumerate(lines):cv2.putText(panel,line,(7,19+24*i),0,.43,warning_color if i==5 else color,1)
    return np.vstack([image,panel])


def main(default_perception="geometry"):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--perception',choices=('geometry','ai','hybrid'),default=default_perception)
    parser.add_argument('--weights',type=Path,default=None)
    parser.add_argument('--device',default='cpu',help='Hybrid inference device: cpu or 0 for CUDA')
    parser.add_argument('--conf',type=float,default=.25,help='Hybrid AI confidence threshold')
    parser.add_argument('--imgsz',type=int,default=384,help='Hybrid inference size; native camera geometry retained')
    parser.add_argument('--config',type=Path,default=BASE/'vroom_config.json')
    parser.add_argument('--topic',default='/automobile/image_raw')
    parser.add_argument('--speed',type=float,default=.06)
    parser.add_argument('--display-scale',type=float,default=.9,help='Dashboard display scale (default: 90 percent; 1 restores original size)')
    parser.add_argument('--lane-preference',choices=('right','observed'),default='right')
    parser.add_argument('--no-assist',action='store_true',help='Start with driver steering only; H enables assistance')
    parser.add_argument('--save-dir',type=Path,default=BASE/'predictions/adas')
    parser.add_argument('--record',action='store_true')
    parser.add_argument('--record-seconds',type=float,default=120.)
    parser.add_argument('--duration',type=float,default=0.,help='Optional viewer time limit')
    args=parser.parse_args()
    if not math.isfinite(args.display_scale) or not .1<=args.display_scale<=2:parser.error('display-scale must be in [0.1,2]')
    if args.imgsz<160 or args.imgsz>1280 or args.imgsz%32:parser.error('imgsz must be a multiple of 32 in [160,1280]')
    if args.weights is None:
        args.weights=BASE/('models/best_seame_lane.pt' if args.perception=='hybrid' else 'best.pt')
    if not math.isfinite(args.conf) or not 0<args.conf<1:parser.error('Invalid confidence threshold')
    if not math.isfinite(args.speed) or not 0<args.speed<=.08:parser.error('speed must be >0 and <=.08')
    if not math.isfinite(args.record_seconds) or not 0<args.record_seconds<=600:parser.error('Invalid recording duration')
    if not math.isfinite(args.duration) or args.duration<0:parser.error('Invalid duration')
    if not(os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):parser.error('Run ADAS from the desktop')
    config=Config.load(args.config);cv2.setNumThreads(1)
    ai=None
    if args.perception=='ai':
        os.environ.setdefault('YOLO_CONFIG_DIR',str(BASE/'.runtime/ultralytics'))
        from adas_ai_perception import AILanePerception
        ai=AILanePerception(args.weights)
    elif args.perception=='hybrid':
        os.environ.setdefault('YOLO_CONFIG_DIR',str(BASE/'.runtime/ultralytics'))
        from adas_marking_perception import MarkingPerception
        ai=MarkingPerception(args.weights,conf=args.conf,device=args.device,imgsz=args.imgsz)
    title={'geometry':'ADAS CAR | Lane assistance','ai':'ADAS CAR AI lane_area',
           'hybrid':'ADAS CAR AI+CV | v10 steering'}[args.perception]
    os.environ.setdefault('ROS_LOG_DIR',str(BASE/'.runtime/adas-ros-logs'))
    lockfile=open(f'/tmp/bfmc-basic-autonomy-{os.getuid()}.lock','w')
    try:fcntl.flock(lockfile,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:parser.error('Close the other controller with Q first')
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from std_msgs.msg import String,Empty
    from cv_bridge import CvBridge
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('bfmc_adas_car');bridge=CvBridge()
    pub=node.create_publisher(String,'/automobile/command',10)
    mutex=threading.RLock();done=threading.Event();driver=ManualDrive(args.speed)
    assist=LaneAssist(enabled=not args.no_assist);tracker=LaneTracker(max_wall_gap_s=.8)
    latest=dict(msg=None,seq=0,received=0.,stamp=None,epoch=0)
    requested=dict(speed=0.,steer=0.);sent=dict(speed=0.,steer=0.,source='STOP',sent_monotonic_s=0.)
    def send(speed=0.,steer=0.,source='STOP'):
        sent.update(speed=speed,steer=steer,source=source,sent_monotonic_s=time.monotonic())
        if speed==0:pub.publish(String(data=json.dumps(dict(action='3',steerAngle=0.))))
        else:
            pub.publish(String(data=json.dumps(dict(action='2',steerAngle=float(steer)))))
            pub.publish(String(data=json.dumps(dict(action='1',speed=float(speed)))))
    def stop(_=None):
        with mutex:
            driver.stop();requested.update(speed=0.,steer=0.);assist.reset('Driver stopped');send()
    def receive(msg):
        stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        with mutex:
            if latest['stamp'] is not None:
                if stamp==latest['stamp']:return
                if stamp<latest['stamp']:
                    latest['epoch']+=1;assist.invalidate('Camera clock reset: driver control')
            latest.update(msg=msg,seq=latest['seq']+1,stamp=stamp,received=time.monotonic())
    def tick():
        with mutex:
            if done.is_set():stop();return
            now=time.monotonic();connected=pub.get_subscription_count()>0
            v,s=driver.command(now,connected)
            requested.update(speed=v,steer=s)
            compute_start=time.perf_counter()
            v,out=assist.command(v,s,now,connected)
            sent['compute_ms']=(time.perf_counter()-compute_start)*1000
            assist.info['image_age_ms']=max(0.,(now-assist.received)*1000) if assist.received is not None else 0.
            source='STOP' if v==0 else ('ADAS' if assist.info['state']=='ASSIST' else 'MANUAL')
            send(v,out,source)
    camera_qos=copy(qos_profile_sensor_data);camera_qos.depth=1
    node.create_subscription(Image,args.topic,receive,camera_qos)
    node.create_subscription(Empty,'/basic_autonomy/stop',stop,10)
    timer=node.create_timer(.05,tick);executor=SingleThreadedExecutor();executor.add_node(node)
    worker=threading.Thread(target=executor.spin,daemon=True)
    signal.signal(signal.SIGINT,lambda *_:done.set());signal.signal(signal.SIGTERM,lambda *_:done.set())
    recorder=None;seen=0;result=None;overlay=np.zeros((480,640,3),np.uint8)
    frame=overlay.copy();last_display=0.;display_received=None
    display_period=1/5 if args.perception=='hybrid' else 1/15
    window=title+' | Driver controls speed | H assist | Space brake'
    display_size=(round(1280*args.display_scale),round(960*args.display_scale))
    cv2.imshow(window,np.zeros((display_size[1],display_size[0],3),np.uint8))
    began=time.monotonic()
    perception=ThreadPoolExecutor(max_workers=1,thread_name_prefix='adas-perception')
    dashboard=ThreadPoolExecutor(max_workers=1,thread_name_prefix='adas-dashboard')
    display_job=None
    pending=None;pending_epoch=None
    def process_image(msg,received):
        # One in-flight image; the next job takes the latest camera message.
        # Detection/annotation never owns the ROS/control mutex or GUI thread.
        started=time.monotonic()
        frame=bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8').copy()
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        if ai:
            raw,painted=ai.detect(frame,config)
            if args.perception=='hybrid' and args.lane_preference=='right':
                raw=prefer_right_lane(raw,config)
        else:
            raw=detect(frame,config);painted=frame
            if args.lane_preference=='right':raw=prefer_right_lane(raw,config)
        result=tracker.update(raw,stamp,received)
        result['camera_stamp_ns']=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        result['controller']=dict(enabled=False,armed=False,mode='ADAS_ONLY')
        overlay=annotate(painted,result)
        finished=time.monotonic()
        result['queue_wait_ms']=(started-received)*1000
        result['perception_work_ms']=(finished-started)*1000
        result['receipt_to_result_ms']=(finished-received)*1000
        return frame,result,overlay,stamp,received

    def start_clip():
        rec=DiagnosticRecorder(args.save_dir/'clips',args.record_seconds,20.,metadata=dict(
            project='ADAS CAR',control_revision='adas-marking-hybrid-2' if args.perception=='hybrid' else ('adas-ai-2' if ai else 'adas-10'),perception_source=args.perception,model_path=str(args.weights) if ai else None,lane_preference=args.lane_preference,auto_enabled=False,
            inference_imgsz=args.imgsz if args.perception=='hybrid' else None,assist_enabled=assist.enabled,manual_speed_command=args.speed,config=vars(config),
            interpretation='driver_command is requested; actual_command is sent; adas is spatial preview, not measured tyre crossing'))
        print(f'REC started: {rec.directory}',flush=True);return rec
    try:
        worker.start()
        if args.record:recorder=start_clip()
        print(title+': driver throttle/brake. H toggles steering assist; G never starts driving.',flush=True)
        while rclpy.ok() and not done.is_set():
            now=time.monotonic()
            if recorder is not None:
                if recorder.error:raise OSError(recorder.error)
                if now-recorder.began>=recorder.seconds:recorder.stop()
                if recorder.done.is_set():
                    print(f'REC saved: {recorder.directory} | {recorder.saved} frames',flush=True);recorder=None
            if pending is not None and pending.done():
                completed=pending.result();pending=None
                with mutex:
                    accept=pending_epoch==latest['epoch']
                    if accept:
                        frame,result,overlay,stamp,received=completed
                        display_received=received
                        assist.observe(result,stamp,received,time.monotonic())
                        tick()
                        result['adas']=assist.snapshot();result['driver_command']=dict(requested)
                        result['adas']['image_age_ms']=max(0.,(time.monotonic()-received)*1000)
                        result['manual_command']=dict(requested);result['actual_command']=dict(sent)
                        key_active=driver.key
                if accept and recorder is not None:recorder.submit(frame,result,received,key_active)
            with mutex:msg,seq,received,epoch=latest['msg'],latest['seq'],latest['received'],latest['epoch']
            if pending is None and msg is not None and seq!=seen and 0<=now-received<=assist.timeout:
                seen=seq;pending_epoch=epoch;pending=perception.submit(process_image,msg,received)
            with mutex:info=assist.snapshot(time.monotonic());driver_snapshot=dict(requested);sent_snapshot=dict(sent)
            if display_received is not None:info['image_age_ms']=max(0.,(time.monotonic()-display_received)*1000)
            # Rendering has its own bounded worker: fonts/resizing must not block keys.
            if display_job is not None and display_job.done():
                cv2.imshow(window,cv2.resize(display_job.result(),display_size,
                           interpolation=cv2.INTER_AREA if args.display_scale<1 else cv2.INTER_LINEAR));display_job=None
            if display_job is None and now-last_display>=display_period:
                rec_info=dict(saved=recorder.saved,dropped=recorder.dropped) if recorder else None
                display_job=dashboard.submit(render_dashboard,frame,overlay,result,info,driver_snapshot,sent_snapshot,args.speed,rec_info)
                last_display=now
            code=cv2.waitKey(1)&0xff;key=chr(code).lower() if code!=255 else ''
            if key:
                with mutex:
                    if key in ('w','a','s','d'):driver.press(key,time.monotonic());tick()
                    elif key==' ':stop()
                    elif key=='h':assist.toggle();tick()
                    elif key=='g':print('ADAS only: hold W/A/D to drive. G has no start action.',flush=True)
                    elif key=='r':
                        if recorder is None:recorder=start_clip()
                        else:print('Already recording/saving; R does not overwrite.',flush=True)
                    elif key=='t' and recorder is not None:recorder.stop()
                    if recorder is not None:
                        recorder.log_event('key',key=key,adas=assist.snapshot(),
                                           driver_command=dict(requested),actual_command=dict(sent))
                if key in ('q','\x1b'):break
            if cv2.getWindowProperty(window,cv2.WND_PROP_VISIBLE)<1:break
            if args.duration and time.monotonic()-began>=args.duration:break
            done.wait(.005)
    finally:
        done.set();stop()
        perception.shutdown(wait=True,cancel_futures=True)
        dashboard.shutdown(wait=True,cancel_futures=True)
        if recorder is not None:
            recorder.close(timeout=20.)
            if recorder.error:print(f'REC error: {recorder.error}',flush=True)
        timer.cancel();executor.shutdown(timeout_sec=2.)
        worker.join(timeout=2.)
        for _ in range(3):send();time.sleep(.05)
        node.destroy_node();rclpy.shutdown();cv2.destroyAllWindows();lockfile.close()

if __name__=='__main__':main()
