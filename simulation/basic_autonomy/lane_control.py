"""Flat-ground camera geometry and bounded lane controller; no ROS dependencies."""
from dataclasses import dataclass
import math
import cv2
import numpy as np


@dataclass
class Camera:
    # rcCar_assembly + car.launch; ground plane z=0; reference is rear axle.
    height: float = .208449
    pitch: float = math.radians(22.)
    hfov: float = math.radians(62.2)
    forward: float = .25193
    left: float = .0025

    def ground(self, u, v, width, height):
        f = width/(2*math.tan(self.hfov/2))
        right, down = (u-(width-1)/2)/f, (v-(height-1)/2)/f
        c,s = math.cos(self.pitch),math.sin(self.pitch)
        rz = -s-c*down
        if rz >= -.02:
            raise ValueError('Ray does not intersect nearby ground')
        t = -self.height/rz
        return self.forward+t*(c-s*down), self.left-t*right


def lane_target(mask, camera=None, lookahead=.65, max_width=.65):
    """Use visible, unclipped row boundaries; reject ambiguous broad regions."""
    camera = camera or Camera()
    if mask.ndim != 2 or min(mask.shape)<32:
        raise ValueError('Invalid mask shape')
    h,w = mask.shape
    mask = (mask > .5).astype(np.uint8)
    if mask.sum() < h*w*.025:
        raise ValueError('Lane area too small')
    # Fill enclosed segmentation holes, preserving separate regions and open forks.
    contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(mask,contours,-1,1,cv2.FILLED)
    points,pixels,widths,clipped = [],[],[],[]
    broad = 0
    ambiguous = 0
    for v in np.linspace(.18*h,.94*h,64).astype(int):
        # Close only short gaps in each horizontal slice, not distinct branches.
        row = cv2.morphologyEx(mask[v:v+1],cv2.MORPH_CLOSE,np.ones((1,5),np.uint8))[0]
        changes = np.diff(np.r_[0,row,0].astype(int))
        starts, ends = np.where(changes==1)[0], np.where(changes==-1)[0]-1
        spans = [(a,b) for a,b in zip(starts,ends) if b-a>=w*.03]
        if len(spans)>1:
            if v>.65*h: ambiguous+=1
            continue
        if len(spans)!=1: continue
        a,b = spans[0]
        if a<=2 or b>=w-3:
            if not (a<=2 and b>=w-3):
                x,y=camera.ground(float(b if a<=2 else a),float(v),w,h)
                clipped.append((x,y,a<=2,v))
            continue
        x,yl = camera.ground(float(a),float(v),w,h)
        _,yr = camera.ground(float(b),float(v),w,h)
        width = yl-yr
        if not .3<=x<=1.3: continue
        if width>max_width:
            broad += 1
            continue
        if width<.16: continue
        widths.append(width)
        points.append((x,(yl+yr)/2))
        pixels.append((int((a+b)/2),int(v)))
    if ambiguous>=3:
        raise ValueError('Multiple nearby lane branches; manual driving required')
    if broad>=3:
        raise ValueError('Region too wide / possible junction; manual driving required')
    if len(points)<5:
        raise ValueError('Not enough visible lane boundaries')
    xy=np.array(points)
    # Recover a clipped near edge only from width measured in this same frame.
    # No fixed assumed lane width and no use of stale frame geometry.
    width=float(np.median(widths))
    if .16<=width<=.55 and np.std(widths)<.09:
        for x,y,left_clipped,v in clipped:
            if max(.3,xy[:,0].min()-.25)<=x<=min(1.3,xy[:,0].max()):
                center=y+width/2 if left_clipped else y-width/2
                points.append((x,center))
                # Invert the camera's lateral projection at this row.
                _,at_center=camera.ground((w-1)/2,float(v),w,h)
                _,at_right=camera.ground((w-1)/2+1,float(v),w,h)
                u=(w-1)/2+(center-at_center)/(at_right-at_center)
                pixels.append((int(np.clip(u,0,w-1)),int(v)))
    xy=np.array(points)
    if xy[:,0].max()-xy[:,0].min()<.08:
        raise ValueError('Visible lane too short')
    # A small lateral offset can clip the near boundary. Move the target to
    # the visible centerline instead of extrapolating through the clipped area.
    lookahead=float(np.clip(lookahead,xy[:,0].min(),xy[:,0].max()))
    if not .4<=lookahead<=.9:
        raise ValueError('Visible lane target is too near or too far')
    coefficients=np.polyfit(xy[:,0],xy[:,1],2)
    residual=np.sqrt(np.mean((np.polyval(coefficients,xy[:,0])-xy[:,1])**2))
    if residual>.045:
        raise ValueError('Lane center is inconsistent')
    y=float(np.polyval(coefficients,lookahead))
    if not math.isfinite(y) or abs(y)>.45:
        raise ValueError('Lane target too far to one side')
    angle=math.degrees(math.atan2(2*.256*y,lookahead**2+y**2))
    return {'steer':max(-25.,min(25.,angle)), 'target':(lookahead,y),
            'points':points,'pixels':pixels,'fit_error':float(residual)}


class DriveGate:
    """Latching stop + age gate. Timer must run independently of inference."""
    def __init__(self, speed=.04, timeout=.8, steering_sign=1):
        self.speed,self.timeout,self.steering_sign=speed,timeout,steering_sign
        self.armed=False
        self.good=0
        self.steer=0.
        self.stamp=None
        self.completed=None
        self.reason='Warming up; press G when READY'
        self.last_raw=None
        self.rejected=False
        self.last_stop='Not started'

    def stop(self, reason='Stopped by user'):
        self.armed=False
        self.good=0
        self.last_raw=None
        self.reason=reason
        self.last_stop=reason
        self.rejected=False

    def reject(self, reason):
        # Brake immediately, but keep AUTO selected only while the last good
        # result-completion watchdog remains live. Never refresh either clock.
        self.rejected=True
        self.good=0
        self.reason='Waiting for lane: '+reason

    def observe(self, steer, frame_received, now):
        if not math.isfinite(steer) or now-frame_received>self.timeout:
            self.stop('Inference result too old')
            return
        if self.stamp is not None and frame_received<=self.stamp: return
        if self.last_raw is not None and abs(steer-self.last_raw)>25:
            self.stop('Lane changed abruptly')
            return
        self.rejected=False
        if self.armed: self.reason="DRIVING"
        self.steer=steer if self.good==0 else .5*self.steer+.5*steer
        self.last_raw=steer
        self.stamp=frame_received
        self.completed=now
        self.good+=1
        if not self.armed:
            self.reason=('READY - press G to drive' if self.good>=3
                         else f'Warming up: {self.good}/3 fresh results')

    def start(self, now, camera_received, connected):
        if not connected:
            self.reason='No command subscriber'
            return False
        if self.good<3 or self.completed is None or now-self.completed>self.timeout or now-camera_received>self.timeout:
            self.reason='Not ready: need 3 recent valid results'
            return False
        self.armed=True
        self.reason='DRIVING'
        return True

    def command(self, now, camera_received, connected):
        if self.armed:
            if not connected: self.stop('Command bridge disconnected')
            elif now-camera_received>self.timeout:
                self.stop('Camera timeout - press G after recovery')
            elif self.completed is None or now-self.completed>self.timeout:
                self.stop('No valid inference completed in time - press G after recovery')
        if not self.armed or self.rejected: return 0.,0.
        if self.stamp is None or now-self.stamp>self.timeout:
            # Brake on old source geometry, without cancelling the user's AUTO
            # selection while the healthy pipeline finishes its next result.
            # completed is updated ONLY by an accepted new result.
            self.reason='AUTO waiting for next result (braking)'
            return 0.,0.
        self.reason='DRIVING'
        speed=self.speed*max(.45,1-abs(self.steer)/40)
        return speed,self.steer*self.steering_sign


class HybridControl:
    """Manual commands have priority and do not depend on lane segmentation."""
    def __init__(self, speed=.025, timeout=1.2, steering_sign=1):
        self.auto=DriveGate(speed,timeout,steering_sign)
        self.manual_until=0.
        self.manual_key=None
        self.manual_speed=max(.025,min(speed,.06))
        self.sign=steering_sign

    @property
    def armed(self): return self.auto.armed

    @property
    def reason(self):
        if self.manual_key: return 'MANUAL '+self.manual_key.upper()+' (release: brake within 0.6s)'
        return self.auto.reason

    def manual(self,key,now):
        if key not in 'wasd': return
        self.auto.stop('Manual takeover; G to select AUTO')
        self.manual_key=key
        self.manual_until=now+.6

    def stop(self,reason='Stopped by user'):
        self.manual_key=None
        self.manual_until=0.
        self.auto.stop(reason)

    def observe(self,*args): self.auto.observe(*args)
    def reject(self,reason): self.auto.reject(reason)

    def start(self,*args):
        self.manual_key=None
        self.manual_until=0.
        return self.auto.start(*args)

    def command(self,now,camera_received,connected):
        if self.manual_key:
            if now>=self.manual_until or not connected:
                self.stop('Manual released / disconnected; WASD or G')
                return 0.,0.
            key=self.manual_key
            return (-self.manual_speed if key=='s' else self.manual_speed,
                    {'w':0.,'s':0.,'a':20.,'d':-20.}[key]*self.sign)
        return self.auto.command(now,camera_received,connected)
