"""Keyboard driver requests with a hold-to-drive timeout; no camera decisions."""
class ManualDrive:
    def __init__(self,speed=.04):
        self.speed=speed;self.key=None;self.until=0.

    def press(self,key,now):
        if key in ('w','a','s','d'):
            self.key=key;self.until=now+.6
        elif key==' ':self.stop()
        # No autonomous entry point. G and detection results cannot set commands.

    def stop(self):self.key=None;self.until=0.

    def command(self,now,connected,camera_fresh=True):
        if not connected or not camera_fresh or now>=self.until:self.stop()
        if self.key is None:return 0.,0.
        return (-self.speed if self.key=='s' else self.speed,{'w':0.,'s':0.,'a':20.,'d':-20.}[self.key])

