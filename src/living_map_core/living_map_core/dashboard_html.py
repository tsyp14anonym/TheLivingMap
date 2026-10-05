DASH_HTML = r"""<!doctype html><html><head><meta charset="utf-8"><title>Command Post - Living Map</title><style>
*{box-sizing:border-box}body{margin:0;background:#0a1120;color:#e6edf7;font:13px/1.35 system-ui,sans-serif;height:100vh;display:flex;flex-direction:column}
header{padding:8px 14px;background:#0f1a30;border-bottom:1px solid #22314f;display:flex;gap:14px;align-items:center}header h1{font-size:15px;margin:0}
#banner{padding:8px 14px;background:#1b2a48}#banner.go{background:#3a2c0d;color:#ffe0a0}#banner.done{background:#12331f;color:#a6e6bd}
main{flex:1;display:flex;min-height:0}#map{flex:1;position:relative;min-width:0}canvas{width:100%;height:100%;display:block}#tools{position:absolute;left:10px;top:8px;background:#0f1a30dd;border:1px solid #22314f;border-radius:6px;padding:6px 10px}
.fb{background:#22314f;color:#fff;border:0;padding:5px 10px;border-radius:5px;margin-right:4px;cursor:pointer}.fb.on{background:#ffb43a;color:#000}
#tip{position:absolute;pointer-events:none;background:#0f1a30;border:1px solid #4d6ea8;border-radius:6px;padding:6px 9px;max-width:320px;font-size:12px;display:none}
#side{width:450px;background:#0f1a30;border-left:1px solid #22314f;overflow:auto;padding:10px}.card{background:#16233d;border-radius:8px;padding:9px 10px;margin:0 0 9px}.card h3{margin:0 0 6px;font-size:11px;color:#7fb0ff;text-transform:uppercase;letter-spacing:.06em}
.al{border-left:4px solid #888;background:#121d35;border-radius:0 6px 6px 0;padding:6px 8px;margin:5px 0}.p0{border-color:#ff4d4d}.p1{border-color:#ffa640}.p2{border-color:#ffe066}.p3{border-color:#8896ab}
.tag{display:inline-block;border-radius:9px;padding:0 7px;font-size:11px;background:#26314a;margin-right:3px}.warn{background:#7a1f1f}.ok{background:#1f5a34}.mut{color:#93a4c0;font-size:11.5px}
.bar{height:6px;background:#0b1424;border-radius:3px;margin:3px 0}.bar i{display:block;height:100%;background:#5dd6a0;border-radius:3px}button.ap{background:#1d5fbf;color:#fff;border:0;padding:8px 14px;border-radius:5px;cursor:pointer}details{color:#b7c6e2}summary{cursor:pointer;color:#7fb0ff}
.dot{display:inline-block;width:10px;height:10px;border-radius:50%;margin-right:5px}
</style></head><body>
<header><h1>COMMAND POST &middot; live map</h1><span class="mut">Everything here arrived through the van's gateway. Robots reach the Command Post only via the van.</span><a href="http://localhost:8081/flow" target="_blank" style="color:#7fb0ff">data flow &amp; beacon table &rarr;</a><span id="now" style="margin-left:auto"></span></header>
<div id="banner"></div>
<main><div id="map"><canvas id="c"></canvas><div id="tools"></div><div id="tip"></div></div><div id="side">
<div class="card"><h3>Robots docked in the van</h3><div id="robots"></div><div id="btn"></div></div>
<div class="card"><h3>Orders (ONE order covers all related jobs)</h3><div class="mut">Reports are collected for a short window so that e.g. fire + gas, or a victim in a danger zone + the hazard, leave in the same order.</div><div id="orders"></div></div>
<div class="card"><h3>Incidents (related events grouped)</h3><div class="mut">&pi; = priority, 0 is the most urgent and there is no upper limit. It starts at &pi;(base) and DROPS while the event stays unresolved. Decisions are taken once per cycle; a critical event (&pi; below the threshold) skips the wait.</div><div id="incs"></div></div>
<div class="card"><h3>Alerts ranked by priority &pi; (0 = most urgent)</h3><div class="mut">&pi; goes from 0 up to no limit. <b>The lower, the more important.</b> It starts from the kind of event (person, fire, gas...) and the writer's class, and it <b>drops while an event waits</b>, so nothing is ever forgotten. A hazard next to a victim takes the victim's urgency.</div><div id="alerts"></div></div>
<div class="card"><h3>Station messages (robot &rarr; van)</h3><div class="mut">Robots only talk to the van, never to each other. Radio protocol: to be specified.</div><div id="msgs"></div></div>
<div class="card"><h3>Uplink log</h3><div id="batches"></div></div>
<div class="card"><details><summary>How to read this map</summary><div class="mut" style="margin-top:6px">Blue dot: entrance (GPS anchor). Coloured dots: beacons (colour = priority); the ring shrinks as confidence decays. Thin lines: the radio chain. Dashed line + X: the target a beacon points to. Orange disc: fire zone; green disc: gas; grey tick: fire out / person rescued; red USAR: needs the human heavy-rescue team. Hover any dot for the beacon's own words: "I am 4 m ahead, 3 m left of ...".</div></details></div>
</div></main>
<script>
const C={0:'#ff4d4d',1:'#ffa640',2:'#ffe066',3:'#8896ab'},RC={ambulance:'#7dff9a',firetruck:'#ff8a3d',drone:'#d38cff'};
const WHY={0:'P0 critical: relayed instantly',1:'P1 high: next batch or earlier',2:'P2 medium: batched',3:'P3 routine / relay / stairs'};
const MEAN={FIRE:'fire',GAS:'gas leak',HUMAN:'person',ANIMAL:'animal',DEBRIS:'blocked by rubble',REPEATER:'radio relay',BEACON_LOST:'beacon went silent',STAIRS:'stairs / shaft'};
const cv=document.getElementById('c'),g=cv.getContext('2d'),tip=document.getElementById('tip'),$=id=>document.getElementById(id);
function fit(){cv.width=cv.clientWidth*devicePixelRatio;cv.height=cv.clientHeight*devicePixelRatio}addEventListener('resize',fit);fit();
let S=null,V={s:1,H:1,x0:-8,y0:-13},OBJ=[],FL=0,built=false;
async function poll(){try{S=await (await fetch('/state')).json();if(!built){built=true;$('tools').innerHTML=Array.from({length:S.floors},(_,i)=>'<button class="fb'+(i==0?' on':'')+'" id="fb'+i+'" onclick="setF('+i+')">Floor '+i+'</button>').join('')}ui();draw()}catch(e){}setTimeout(poll,500)}
function setF(i){FL=i;for(let k=0;k<S.floors;k++)$('fb'+k).className='fb'+(k==i?' on':'')}
function ui(){$('now').textContent='sim time '+S.now+' s';const b=$('banner'),recs=S.records,rb=S.robots;let msg,cls='';
 const act=Object.values(rb).filter(r=>r.state=='dispatched'||r.state=='working').length;
 if(!recs.length)msg='Waiting for the first report from the van...';else if(S.awaiting_approval){msg='<b>Robots are ready. Waiting for your approval.</b>';cls='go'}else if(act){msg='<b>'+act+' robot(s) working inside the building.</b> The Command Post follows their reports through the van.';cls='done'}
 else msg='Receiving reports ('+recs.length+' so far). Robots are dispatched when a target is known and the briefing is ready.';b.innerHTML=msg;b.className=cls;
 $('btn').innerHTML=S.awaiting_approval?'<button class="ap" onclick="fetch(\'/approve\',{method:\'POST\'})">APPROVE DISPATCH</button>':'';
 $('robots').innerHTML=Object.entries(rb).map(([r,v])=>'<div class="al" style="border-color:'+RC[r]+'"><b>'+v.role_text+'</b> <span class="tag '+(v.state=='dispatched'||v.state=='working'?'ok':'')+'">'+v.state+'</span><div class="mut">'+v.note+'</div>'+(v.targets&&v.targets.length?'<div class="mut">jobs: '+v.targets.map(t=>t.id+' ('+t.type.toLowerCase()+', floor '+t.floor+')').join(', ')+'</div>':'')+'</div>').join('')+(S.skipped.length?'<div class="al p1"><b>Needs the human heavy-rescue team:</b> '+S.skipped.join(', ')+'<div class="mut">sealed behind rubble: no robot can lift debris.</div></div>':'');
 const rs=[...recs].filter(r=>r.type!='REPEATER').sort((a,b)=>a.pi-b.pi);
 $('alerts').innerHTML=rs.map(r=>'<div class="al p'+r.prio+'"><b>'+r.id+' &middot; '+(MEAN[r.type]||r.type)+' &middot; floor '+r.floor+'</b> <span class="tag">P'+r.prio+'</span><span class="tag">Q '+r.Q+'</span>'+(r.out?'<span class="tag ok">FIRE OUT</span>':'')+(r.rescued?'<span class="tag ok">RESCUED</span>':'')+(r.human_needed?'<span class="tag warn">HEAVY RESCUE REQUIRED</span>':'')+(r.reverify?'<span class="tag warn">RE-VERIFY</span>':'')+
  '<div class="bar"><i style="width:'+Math.round(r.conf*100)+'%"></i></div><div class="mut">confidence '+Math.round(r.conf*100)+'% &middot; '+WHY[r.prio]+'<br><b>beacon says:</b> I am '+r.hop_text+'<br><b>target:</b> '+r.tgt_text+'<br>GPS '+r.lat.toFixed(6)+', '+r.lon.toFixed(6)+' (&plusmn;'+r.sigma+' m) &middot; '+r.hops+' hops, '+r.rssi+' dBm</div></div>').join('')||'<span class="mut">nothing yet</span>';
 {const g={};(S.events||[]).forEach(e=>{const k=e.incident||('single '+e.event_id);(g[k]=g[k]||[]).push(e)});$('incs').innerHTML=Object.entries(g).filter(([k,v])=>v.some(e=>e.status!=='RESOLVED')).slice(0,14).map(([k,v])=>'<div class="al"><b>'+(k.indexOf('single')==0?'single event':k)+'</b> '+v.map(e=>e.event_id+' '+e.type+' &pi;'+e.priority_base+'&rarr;'+e.priority+' ['+e.status+(e.assigned_robot?' &middot; '+e.assigned_robot:'')+']').join(' &nbsp;|&nbsp; ')+'</div>').join('')||'<span class="mut">no open incident</span>'}
 $('orders').innerHTML=[...(S.orders||[])].reverse().map(o=>'<div class="al" style="border-color:#7fb0ff"><b>'+o.id+' &middot; '+o.t+' s</b><div class="mut">'+o.text+'</div></div>').join('')||'<span class="mut">no order yet</span>';
 $('msgs').innerHTML=[...S.messages].reverse().map(m=>'<div class="al" style="border-color:'+(RC[m.robot]||'#888')+'"><b>'+m.t+' s &middot; '+m.robot+'</b>: '+m.kind+' <span class="mut">'+(m.payload&&m.payload.id?m.payload.id:'')+'</span></div>').join('')||'<span class="mut">none yet</span>';
 $('batches').innerHTML=[...S.batches].reverse().map(b=>'<div class="al"><b>t='+b.t.toFixed(0)+' s</b> '+b.n+' record(s)<div class="mut">'+({IMMEDIATE:'sent at once, no batching',P0_IMMEDIATE:'critical alert, sent immediately with its context',BATCH_120S:'regular 120 s batch',ADAPTIVE_SCORE:'left early: score became urgent',ADAPTIVE_SIZE:'left early: queue full',STATUS:'robot status message'}[b.reason]||b.reason)+'</div></div>').join('')||'<span class="mut">nothing received</span>'}
function draw(){if(!S)return;const W=cv.width,H=cv.height,x0=-8,x1=42,y0=-13,y1=13,s=Math.min(W/(x1-x0),H/(y1-y0));V={s,H,x0,y0};OBJ=[];g.font=(11*devicePixelRatio)+'px sans-serif';
 const X=x=>(x-x0)*s,Y=y=>H-(y-y0)*s;g.clearRect(0,0,W,H);g.lineWidth=1;g.strokeStyle='#182740';
 for(let x=-8;x<=42;x+=4){g.beginPath();g.moveTo(X(x),0);g.lineTo(X(x),H);g.stroke()}for(let y=-12;y<=12;y+=4){g.beginPath();g.moveTo(0,Y(y));g.lineTo(W,Y(y));g.stroke()}
 g.fillStyle='#4da3ff';g.beginPath();g.arc(X(0),Y(0),7,0,7);g.fill();g.fillStyle='#9fc4f5';g.fillText('entrance anchor (GPS)',X(0)+10,Y(0)+4);OBJ.push({x:0,y:0,r:1,t:'<b>Entrance anchor</b><br>Only place with GPS. Every beacon position is rebuilt by chaining "ahead / right" hops from here.'});
 const by={};S.records.forEach(r=>by[r.bid]=r);const R=S.records.filter(r=>r.floor==FL);
 R.forEach(r=>{const p=by[r.parent];g.strokeStyle='#3a5a8c';g.beginPath();g.moveTo(X(p?p.x:0),Y(p?p.y:0));g.lineTo(X(r.x),Y(r.y));g.stroke()});
 R.forEach(r=>{
  if((r.type=='FIRE'||r.type=='GAS')&&!r.out){g.fillStyle=r.type=='FIRE'?'rgba(255,90,40,.22)':'rgba(200,255,80,.18)';g.beginPath();g.arc(X(r.tx),Y(r.ty),(r.type=='FIRE'?2.5:3)*s,0,7);g.fill()}
  if(['HUMAN','ANIMAL','DEBRIS','FIRE'].includes(r.type)){const col=r.out||r.rescued?'#3aa55a':(r.human_needed?'#ff4d4d':C[r.prio]);g.strokeStyle=col;g.setLineDash([4,4]);g.beginPath();g.moveTo(X(r.x),Y(r.y));g.lineTo(X(r.tx),Y(r.ty));g.stroke();g.setLineDash([]);
   g.lineWidth=3;const a=X(r.tx),b=Y(r.ty);g.beginPath();if(r.out||r.rescued){g.moveTo(a-7,b);g.lineTo(a-2,b+6);g.lineTo(a+8,b-6)}else{g.moveTo(a-7,b-7);g.lineTo(a+7,b+7);g.moveTo(a+7,b-7);g.lineTo(a-7,b+7)}g.stroke();g.lineWidth=1;if(r.human_needed){g.fillStyle='#ff4d4d';g.fillText('USAR',a+10,b-8)}}
  g.globalAlpha=.25+.75*r.conf;g.strokeStyle=C[r.prio];g.lineWidth=2;g.beginPath();g.arc(X(r.x),Y(r.y),(r.reverify?16:12)*devicePixelRatio*r.conf+4,0,7);g.stroke();g.globalAlpha=1;g.lineWidth=1;
  g.fillStyle=r.type=='REPEATER'?'#8896ab':C[r.prio];g.beginPath();g.arc(X(r.x),Y(r.y),(r.type=='REPEATER'?3:5)*devicePixelRatio,0,7);g.fill();if(r.type!='REPEATER'){g.fillStyle='#dce8fb';g.fillText(r.id+' '+r.type,X(r.x)+8,Y(r.y)-8)}
  OBJ.push({x:r.x,y:r.y,r:.7,t:'<b>'+r.id+' &middot; '+(MEAN[r.type]||r.type)+'</b> (floor '+r.floor+')<br>'+WHY[r.prio]+'<br><b>"I am '+r.hop_text+'."</b><br><b>"Target: '+r.tgt_text+'."</b><br>confidence '+Math.round(r.conf*100)+'%, position error about &plusmn;'+r.sigma+' m'})});
 g.fillStyle='#9fb4d8';g.fillText('FLOOR '+FL,X(-7.5),Y(12))}
cv.addEventListener('mousemove',ev=>{const r=cv.getBoundingClientRect(),px=(ev.clientX-r.left)*devicePixelRatio,py=(ev.clientY-r.top)*devicePixelRatio;const wx=px/V.s+V.x0,wy=(V.H-py)/V.s+V.y0;let best=null,bd=1e9;
 for(const o of OBJ){const d=Math.hypot(o.x-wx,o.y-wy);if(d<o.r+.3&&d<bd){bd=d;best=o}}
 if(best){tip.style.display='block';tip.style.left=(ev.clientX-r.left+14)+'px';tip.style.top=(ev.clientY-r.top+14)+'px';tip.innerHTML=best.t}else tip.style.display='none'});
poll();
</script></body></html>"""
