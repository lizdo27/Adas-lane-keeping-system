"""Bounded background recording of processed camera frames and exact diagnostics."""
from copy import deepcopy
from datetime import datetime
import json
import math
from pathlib import Path
import queue
import threading
import time
import cv2
import numpy as np


def diagnostic_json(value):
    """Preserve numeric/boolean types from NumPy in diagnostic JSON."""
    if isinstance(value, np.generic):return value.item()
    if isinstance(value, np.ndarray):return value.tolist()
    raise TypeError(f"Object of type {type(value).__name__} is not JSON serializable")


class DiagnosticRecorder:
    def __init__(self,root,seconds=120.,fps=10.,metadata=None):
        if not all(math.isfinite(x) and x>0 for x in (seconds,fps)):raise ValueError('Invalid recording duration/rate')
        self.directory=Path(root)/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.directory.mkdir(parents=True,exist_ok=False)
        (self.directory/'frames').mkdir()
        self.seconds=seconds;self.fps=fps;self.began=time.monotonic()
        self.queue=queue.Queue(maxsize=8);self.stopping=threading.Event();self.done=threading.Event()
        self.saved=0;self.dropped=0;self.error=None;self.last_stamp=None
        self.metadata=dict(metadata or {},status='recording',algorithm_version=12,
                           timing='Use camera_stamp_ns and received_monotonic_s from frames.jsonl. AVI playback has fixed FPS.',
                           contents='Lossless raw PNGs plus exact processed results. Only frames processed by viewer are recorded.',
                           manual_semantics='manual_key is requested keyboard state, not measured steering or speed',
                           limit_seconds=seconds,video_fps=fps)
        self._manifest('recording')
        self.worker=threading.Thread(target=self._run,name='vroom-recorder',daemon=True)
        self.worker.start()

    def _manifest(self,status):
        data=dict(self.metadata,status=status,saved=self.saved,dropped=self.dropped,error=self.error)
        p=self.directory/'session.json.tmp';p.write_text(json.dumps(data,indent=2,default=diagnostic_json));p.replace(self.directory/'session.json')

    def submit(self,frame,result,received,manual_key=None):
        if self.error:raise OSError(self.error)
        if time.monotonic()-self.began>=self.seconds:self.stop()
        if self.stopping.is_set():return False
        stamp=result['camera_stamp_ns']
        if self.last_stamp is not None and stamp<=self.last_stamp:
            if stamp<self.last_stamp:raise ValueError('Recording camera clock reset')
            return False
        self.last_stamp=stamp
        record=dict(result=deepcopy(result),camera_stamp_ns=stamp,received_monotonic_s=received,manual_key=manual_key)
        try:self.queue.put_nowait((frame.copy(),record))
        except queue.Full:self.dropped+=1;return False
        return True

    def log_event(self,kind,**data):
        # Main UI thread only. Key events survive between processed frames.
        with (self.directory/'control_events.jsonl').open('a') as stream:
            stream.write(json.dumps(dict(event=kind,monotonic_s=time.monotonic(),**data),default=diagnostic_json)+'\n')

    def stop(self):self.stopping.set()

    def close(self,timeout=5.):
        self.stop();self.worker.join(timeout)
        return self.done.is_set()

    def _run(self):
        writer=None;stream=None;size=None
        try:
            stream=(self.directory/'frames.jsonl').open('w')
            while not self.stopping.is_set() or not self.queue.empty():
                try:frame,record=self.queue.get(timeout=.1)
                except queue.Empty:continue
                shape=(frame.shape[1],frame.shape[0])
                if size is None:
                    size=shape;writer=cv2.VideoWriter(str(self.directory/'camera.avi'),cv2.VideoWriter_fourcc(*'MJPG'),self.fps,size)
                    if not writer.isOpened():raise OSError('Cannot create recording video')
                if shape!=size:raise ValueError('Camera size changed during recording')
                filename=f'frames/frame_{self.saved:06d}.png'
                if not cv2.imwrite(str(self.directory/filename),frame,[cv2.IMWRITE_PNG_COMPRESSION,1]):raise OSError('Cannot write recording frame')
                writer.write(frame)
                record.update(image=filename,video_index=self.saved)
                stream.write(json.dumps(record,default=diagnostic_json)+'\n');stream.flush();self.saved+=1
        except Exception as exc:
            self.error=str(exc);self.stopping.set()
        finally:
            if writer is not None:writer.release()
            if stream is not None:stream.close()
            try:self._manifest('error' if self.error else 'complete')
            except Exception as exc:self.error=str(exc)
            self.done.set()
