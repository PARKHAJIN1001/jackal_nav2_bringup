import json,sys,math
from pathlib import Path
from collections import defaultdict
p=Path(__file__).parent/'samples.jsonl'
window=float(sys.argv[1]) if len(sys.argv)>1 else 30.
rows=[]
with p.open() as f:
    for line in f:
        try: rows.append(json.loads(line))
        except json.JSONDecodeError: pass
if not rows: raise SystemExit('no samples')
end=rows[-1]['wall']; start=end-window
rows=[r for r in rows if r['wall']>=start]
groups=defaultdict(list)
for r in rows:
    if r['kind']=='tf' and (r['parent'],r['child']) in [('odom','base_link'),('map','odom')]: groups[r['topic']+': '+r['parent']+' -> '+r['child']].append(r)
    if r['kind']=='composed': groups['map -> base_link (composed)'].append(r)
result={'start':start,'end':end,'duration':window,'poses':{}}
for k,v in groups.items():
    summary={'samples':len(v)}
    for field,index,name,scale in [('xyz',2,'z_m',1.),('rpy',0,'roll_deg',180/math.pi),('rpy',1,'pitch_deg',180/math.pi),('rpy',2,'yaw_deg',180/math.pi)]:
        a=[r[field][index]*scale for r in v]
        if name == 'yaw_deg':
            for j in range(1,len(a)):
                a[j] = a[j-1] + (a[j]-a[j-1]+180)%360-180
        mean=sum(a)/len(a)
        summary[name]={'min':min(a),'max':max(a),'range':max(a)-min(a),'std':math.sqrt(sum((x-mean)**2 for x in a)/len(a)),'first':a[0],'last':a[-1]}
    result['poses'][k]=summary
h=[r for r in rows if r['kind']=='health']
result['inputs']={}
for r in h:
    for t,s in r['topics'].items():
        a=result['inputs'].setdefault(t,{'count':0,'max_receipt_gap':0,'max_age':-1e9,'regressions':0})
        a['count']+=s['n']; a['max_receipt_gap']=max(a['max_receipt_gap'],s['max_receipt_gap']); a['max_age']=max(a['max_age'],s['max_age']);a['regressions']+=s['regressions']
if len(h)>1:
    result['kernel_delta']={family:{k:value-h[0]['kernel'][family].get(k,value) for k,value in counters.items()} for family,counters in h[-1]['kernel'].items()}
    result['ipfrag_high_thresh']=h[-1]['ipfrag_high_thresh']
print(json.dumps(result,indent=2))
