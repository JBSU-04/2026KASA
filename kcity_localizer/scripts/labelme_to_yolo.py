#!/usr/bin/env python3
import argparse, json, os, random, shutil

CLASSES = ["red", "green", "left_arrow", "yellow"]

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--src", required=True); ap.add_argument("--out", required=True)
    ap.add_argument("--val", type=float, default=.2); ap.add_argument("--seed", type=int, default=20260721)
    a = ap.parse_args(); random.seed(a.seed)
    for split in ("train", "val"):
        os.makedirs(os.path.join(a.out, "images", split), exist_ok=True)
        os.makedirs(os.path.join(a.out, "labels", split), exist_ok=True)
    files=[]
    for jf in os.listdir(a.src):
        if not jf.endswith('.json'): continue
        d=json.load(open(os.path.join(a.src,jf),encoding='utf-8'))
        ip=os.path.join(a.src,d.get('imagePath',jf[:-5]+'.jpg'))
        if os.path.isfile(ip): files.append((jf,ip,d))
    random.shuffle(files); nval=max(1,int(len(files)*a.val)); valset={x[0] for x in files[:nval]}
    for jf,ip,d in files:
        split='val' if jf in valset else 'train'; stem=os.path.splitext(os.path.basename(ip))[0]
        shutil.copy2(ip,os.path.join(a.out,'images',split,os.path.basename(ip)))
        lines=[]; w=float(d.get('imageWidth',0)); h=float(d.get('imageHeight',0))
        if not w or not h:
            from PIL import Image
            with Image.open(ip) as im: w,h=im.size
        for s in d.get('shapes',[]):
            if s.get('label') not in CLASSES or len(s.get('points',[]))<2: continue
            (x1,y1),(x2,y2)=s['points'][:2]; x1,x2=sorted((x1,x2)); y1,y2=sorted((y1,y2))
            lines.append('%d %.6f %.6f %.6f %.6f' % (CLASSES.index(s['label']),((x1+x2)/2)/w,((y1+y2)/2)/h,(x2-x1)/w,(y2-y1)/h))
        open(os.path.join(a.out,'labels',split,stem+'.txt'),'w').write('\n'.join(lines)+'\n')
    with open(os.path.join(a.out,'data.yaml'),'w') as f:
        f.write('path: %s\ntrain: images/train\nval: images/val\nnames: %s\n' % (a.out, CLASSES))
    print('images',len(files),'train',len(files)-nval,'val',nval,'yaml',os.path.join(a.out,'data.yaml'))
if __name__=='__main__': main()
