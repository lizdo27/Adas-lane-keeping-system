import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/perception'))
import numpy as np,cv2
from hybrid_video import Fusion
cv2.setNumThreads(1)
h,w=240,432
zero=np.zeros((h,w),np.float32)
blank=np.full((h,w,3),60,np.uint8)
# A stable lone bright vertical streak is insufficient independent evidence.
streak=blank.copy();cv2.rectangle(streak,(300,110),(310,230),(235,235,235),-1)
f=Fusion()
for n in range(6):_,mask,_=f.process(streak,zero,zero,96,n/30)
assert not mask.any(),'persistent isolated streak incorrectly accepted'
# Aligned dash group may recover even with no AI.
dash=blank.copy();ai=zero.copy()
for a,b in ((110,135),(155,180),(200,225)):
 cv2.rectangle(dash,(110,a),(120,b),(235,235,235),-1);ai[a:b+1,110:121]=.9
f=Fusion()
for n in range(4):_,mask,_=f.process(dash,zero,zero,96,n/30)
assert mask.any(),'independent dash group not recovered'
assert not mask[139:151,105:126].any(),'gaps painted'
_,mask,_=f.process(blank,zero,zero,96,4/30)
assert not mask.any(),'old pixels persisted on blank'
# Current AI remains without CV.
_,mask,_=Fusion().process(blank,ai,zero,96,0)
assert mask.any(),'AI erased when CV absent'
# Time reversal invalidates temporal support.
f=Fusion()
f.process(dash,ai,zero,96,1)
_,mask,_=f.process(blank,zero,zero,96,0)
assert not mask.any()
print('PASS: isolated glare, independent dash chain, gaps, no ghost pixels, AI fallback, timestamp reset')
