"""Vietnamese presentation dashboard. Displays observations and commands, never drives."""
from functools import lru_cache
import math
from pathlib import Path
import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

BG=(19,25,37);CARD=(29,38,53);WHITE=(235,242,250);MUTED=(151,169,190)
GREEN=(89,216,167);AMBER=(255,196,92);RED=(255,111,116);BLUE=(99,180,255)
FONT=Path('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf')

@lru_cache(maxsize=16)
def font(size):return ImageFont.truetype(str(FONT),size)

@lru_cache(maxsize=512)
def text_tile(text,size,color,bg):
    f=font(size);box=f.getbbox(text);w=max(1,box[2]+2);h=size+8
    tile=Image.new('RGB',(w,h),bg);ImageDraw.Draw(tile).text((0,-box[1]+2),text,font=f,fill=color)
    return np.asarray(tile)[:,:,::-1].copy()

@lru_cache(maxsize=512)
def fitted(text,size,max_width):
    if max_width:
        while text and font(size).getlength(text)>max_width:text=text[:-2]+'…'
    return text

def label(canvas,text,x,y,size=18,color=WHITE,bg=BG,max_width=None):
    text=fitted(str(text),size,max_width)
    tile=text_tile(text,size,color,bg);h,w=tile.shape[:2]
    h=min(h,canvas.shape[0]-y);w=min(w,canvas.shape[1]-x)
    if h>0 and w>0:canvas[y:y+h,x:x+w]=tile[:h,:w]

def rect(canvas,x,y,w,h,color):cv2.rectangle(canvas,(x,y),(x+w,y+h),color[::-1],-1)

def number(value,format='.1f'):
    return f'{value:{format}}' if isinstance(value,(float,int)) and math.isfinite(value) else '—'

def explanation(info):
    state=info.get('state','UNAVAILABLE');reason=info.get('reason','')
    if state=='MANUAL':return 'Người lái đang dừng / lùi / tắt hỗ trợ.'
    if state=='UNAVAILABLE':
        if 'Recently threatened' in reason:return 'Mất biên sát xe vừa có nguy cơ: bạn cần tự sửa lái.'
        if 'stale' in reason or 'observation' in reason:return 'Ảnh không còn đủ mới: trả quyền lái tay.'
        if 'Junction' in reason:return 'Nhánh giao chưa rõ biên: người lái chọn hướng.'
        if 'warming' in reason.lower():return 'Đang xác nhận biên qua các ảnh liên tiếp.'
        if 'Transverse' in reason:return 'Vạch ngang chưa đủ biên liên tục để hỗ trợ.'
        return 'Chưa đủ dữ liệu hình học: trả quyền lái tay.'
    if info.get('near_edge_limited'):return 'Chặn góc sửa ép vào biên gần; người lái cần xử lý.'
    if info.get('guidance_mode')=='FRONT_EDGE_RECOVERY':return 'Sửa ngắn khỏi mép; cập nhật lại theo ảnh mới.'
    if state=='MONITOR':return 'Chưa cần sửa: giữ góc người lái yêu cầu.'
    if info.get('limited') or info.get('residual_risk'):return 'Đã sửa nhưng còn nguy cơ: người lái cần xử lý.'
    return 'Dự đoán lệch làn: điều chỉnh góc lái hỗ trợ.'


def render_dashboard(raw,overlay,result,info,driver,sent,speed_setting,recording=None):
    """One camera view with a compact HUD. Display only; never changes commands."""
    result=result or {};state=info.get('state','UNAVAILABLE')
    names={'MANUAL':'LÁI TAY / DỪNG','MONITOR':'ĐANG THEO DÕI','ASSIST':'ĐANG HỖ TRỢ','WARNING':'CÒN NGUY CƠ','UNAVAILABLE':'KHÔNG HỖ TRỢ'}
    accent=RED if state=='UNAVAILABLE' else AMBER if state in ('ASSIST','WARNING') else GREEN
    if raw is None:raw=np.zeros((480,640,3),np.uint8)
    age=info.get('image_age_ms')
    stale=isinstance(age,(int,float)) and age>800
    # Strip the legacy diagnostic header; retain geometry on the source image.
    keep_diagnostic=result.get('perception_source') in ('AI lane_area','AI+CV lane_marking')
    annotated=raw if overlay is None or (stale and not keep_diagnostic) else overlay[-raw.shape[0]:]
    canvas=np.empty((960,1280,3),np.uint8);canvas[:]=BG[::-1]
    h,w=annotated.shape[:2];scale=min(1280/w,960/h)
    width,height=int(round(w*scale)),int(round(h*scale))
    x,y=(1280-width)//2,(960-height)//2
    canvas[y:y+height,x:x+width]=cv2.resize(annotated,(width,height))
    # HUD stays at the image edges; the central road view remains unobstructed.
    rect(canvas,0,0,1280,76,BG)
    label(canvas,'ADAS CAR',20,10,26)
    perception_title={'AI lane_area':'AI • best.pt / lane_area',
                      'AI+CV lane_marking':'AI + XỬ LÝ ẢNH • HỖ TRỢ v10'}.get(result.get('perception_source'),'HỖ TRỢ GIỮ LÀN')
    label(canvas,perception_title,20,45,14,BLUE)
    label(canvas,names.get(state,state),350,12,24,accent,max_width=580)
    label(canvas,'Hỗ trợ BẬT' if info.get('enabled') else 'Hỗ trợ TẮT',350,47,15,MUTED)
    rec=f"REC {recording['saved']} khung · bỏ {recording['dropped']}" if recording else 'R: ghi clip'
    label(canvas,rec,1000,14,17,RED if recording else MUTED,max_width=260)
    label(canvas,f"Ảnh {number(age,'.0f')} ms | xử lý {number(result.get('processing_ms'),'.0f')} ms",960,47,14,MUTED,max_width=300)
    alert=info.get('lane_alert',{});risk=alert.get('state','UNKNOWN')
    side={'left':'trái','right':'phải'}.get(alert.get('side'),'—')
    risk_text={'ESTIMATED_OVERLAP':f'Ước tính lấn mép {side}','NEAR_EDGE':f'Sát mép {side}',
               'NO_FRONT_OVERLAP_ESTIMATED':'Chưa thấy lấn mép trước','UNKNOWN':'Chưa xác nhận tiếp xúc'}.get(risk,'Chưa xác nhận tiếp xúc')
    if risk=='UNKNOWN' and 'conflicts' in alert.get('reason',''):risk_text='Ước lượng chưa thống nhất'
    if stale:risk_text='ẢNH CŨ — chỉ xem, không dùng hỗ trợ';risk='UNKNOWN'
    risk_color=RED if risk=='ESTIMATED_OVERLAP' else AMBER if risk in ('NEAR_EDGE','UNKNOWN') else GREEN
    rect(canvas,16,90,490,68,BG)
    label(canvas,risk_text,28,100,21,risk_color,max_width=465)
    bounds=result.get('boundaries',[])
    source='Tâm từ hai biên' if result.get('center_source')=='two_boundaries' else 'Tâm ước lượng'
    if not bounds:source='Chưa có biên sử dụng được'
    if result.get('lane_selection')=='right_of_observed_divider':source='Làn phải: vạch phân cách ở trái'
    if result.get('perception_source')=='AI lane_area':
        source='Mask AI: '+result.get('status','WAITING')
        label(canvas,f"AI: {result.get('mask_count',0)} mask | conf {number(result.get('model_confidence'),'.2f')}",20,170,16,BLUE,max_width=465)
    label(canvas,f"{len(bounds)} biên • {source}",28,133,14,MUTED,max_width=465)
    if result.get('perception_source')=='AI+CV lane_marking':
        label(canvas,f"AI: {result.get('mask_count',0)} vạch | CV: {result.get('candidate_count',0)} điểm đối chiếu",20,170,16,BLUE,max_width=465)
    rect(canvas,0,828,1280,132,BG)
    label(canvas,explanation(info),20,836,18,accent,max_width=1235)
    target=info.get('desired_steer_deg')
    if state in ('MANUAL','UNAVAILABLE'):target=None
    for x,title,val,color in [(20,'NGƯỜI LÁI',driver.get('steer'),WHITE),(330,'MỤC TIÊU HỖ TRỢ',target,AMBER),(640,'GÓC THỰC GỬI',sent.get('steer'),GREEN)]:
        label(canvas,title,x,869,12,MUTED)
        direction='' if val is None or abs(val)<.05 else ('  ← trái' if val>0 else '  phải →')
        label(canvas,number(val,'+.1f')+'°'+direction,x,890,22,color,max_width=295)
    label(canvas,'LỆNH GA • KHÔNG PHẢI m/s',970,869,12,MUTED)
    label(canvas,number(sent.get('speed'),'.3f')+' / '+number(speed_setting,'.2f'),970,890,22,WHITE,max_width=285)
    label(canvas,'W/A/S/D lái • Space phanh • H hỗ trợ • R ghi • T kết thúc • Q thoát',20,930,15,MUTED,max_width=780)
    label(canvas,'Biên: trái xanh lam / phải hồng • Tâm: xanh / cam',800,932,12,MUTED,max_width=460)
    return canvas
