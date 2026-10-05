FLOW_HTML = r"""<!doctype html><html><head><meta charset="utf-8"><title>Living Map - Data flow & beacon table</title><style>
*{box-sizing:border-box}body{margin:0;background:#0a1120;color:#e6edf7;font:13px/1.35 system-ui,sans-serif;height:100vh;display:flex;flex-direction:column}
header{display:flex;align-items:center;gap:14px;padding:7px 14px;background:#0f1a30;border-bottom:1px solid #22314f}header h1{font-size:15px;margin:0}header a{color:#7fb0ff}.mut{color:#93a4c0;font-size:12px}
#top{display:flex;gap:10px;padding:8px 12px 0}#diag{flex:1.6;background:#0d1730;border:1px solid #22314f;border-radius:8px;min-width:0}#diag svg{width:100%;height:290px;display:block}
#stream{flex:1;background:#0d1730;border:1px solid #22314f;border-radius:8px;padding:8px;height:298px;overflow:auto}
#bot{flex:1;min-height:0;padding:8px 12px 12px;display:flex;flex-direction:column}#tw{flex:1;overflow:auto;border:1px solid #22314f;border-radius:8px;background:#0d1730}
table{border-collapse:collapse;width:100%}th{position:sticky;top:0;background:#16233d;text-align:left;padding:6px 8px;font-size:11px;text-transform:uppercase;letter-spacing:.05em;color:#7fb0ff;z-index:1}
td{padding:5px 8px;border-top:1px solid #1a2744;white-space:nowrap}tr.flash td{animation:fl 1.6s}@keyframes fl{0%{background:#5a4a14}100%{background:transparent}}
.pr{display:inline-block;min-width:26px;text-align:center;border-radius:5px;padding:1px 6px;font-weight:700;color:#000}.p0{background:#ff4d4d}.p1{background:#ffa640}.p2{background:#ffe066}.p3{background:#8896ab}
.st{display:inline-block;border-radius:10px;padding:1px 9px;font-size:11.5px;font-weight:600}
.s-KNOWN{background:#26314a}.s-QUEUED{background:#3a3a66}.s-HOLDING{background:#5a4a14;color:#ffe0a0}.s-EN_ROUTE{background:#1d4f8f}.s-ON_SITE{background:#6a3d9a}.s-IN_PROGRESS{background:#9a5a14}.s-RESOLVED{background:#1f5a34;color:#bff0cf}.s-UNREACHABLE{background:#7a1f1f}.s-HEAVY_RESCUE{background:#8a2a5a}.s-LOST{background:#444}.s-ACTIVE{background:#1b2a48;color:#8fa0c0}.s-DROPPED{background:#2b3b2b;color:#b8d8b8}.s-DESTROYED{background:#552}.s-none{background:#161e30;color:#667}
.ev{padding:3px 6px;border-left:3px solid #556;margin:2px 0;background:#121d35;border-radius:0 4px 4px 0;font-size:12px}.ev small{color:#8fa0c0}
.k-DROP{border-color:#40d0ff}.k-RF{border-color:#b48cff}.k-UPLINK{border-color:#ffd54d}.k-DISPATCH{border-color:#7fb0ff}.k-STATION{border-color:#c9d6ff}.k-STATE{border-color:#7dff9a}.k-WRITE{border-color:#7dff9a}
label{margin-right:12px;cursor:pointer}#cnt span{margin-right:14px}
</style></head><body>
<header><h1>DATA FLOW &middot; beacons &harr; Command Post &harr; Executors</h1><span class="mut" id="hud"></span><a href="/" >simulation</a><a id="cplink" target="_blank">Command Post</a><span class="mut" style="margin-left:auto">updates 3&times; per second</span></header>
<div id="top"><div id="diag"><svg id="svg" viewBox="0 0 900 290"></svg></div><div id="stream"><b style="color:#7fb0ff;font-size:11px">LIVE MESSAGES</b><div id="evs"></div></div></div>
<div id="bot"><div style="margin:2px 0 6px"><b style="color:#7fb0ff">BEACON TABLE</b> <span class="mut">each beacon, its priority and its state &middot; <b>on the beacon</b> = what is physically written in the beacon (Executors rewrite it) &middot; <b>at Command Post</b> = what the Command Post knows</span>
<span style="margin-left:14px"><label><input type="checkbox" id="f_rel"> hide relays / stairs</label></span><span id="cnt" class="mut"></span></div>
<div id="tw"><table><thead><tr><th>#</th><th>Type</th><th>Floor</th><th>Writer class</th><th>State at Command Post</th><th>State on the beacon</th><th>Robot</th><th>Conf.</th><th>Priority &pi; (0 = most urgent, lower is more important)</th><th>Last update</th><th>Beacon says</th></tr></thead><tbody id="rows"></tbody></table></div></div>
<script>
const $=id=>document.getElementById(id),NS='http://www.w3.org/2000/svg';
const N={writer:[90,45,'WRITER','#40d0ff'],mesh:[90,150,'BEACON MESH','#b48cff'],van:[360,150,'VAN (gateway)','#4da3ff'],cp:[690,70,'COMMAND POST','#ffd54d'],ambulance:[470,250,'AMBULANCE','#7dff9a'],firetruck:[640,250,'FIRE TRUCK','#ff8a3d'],drone:[810,250,'DRONE','#d38cff']};
const LINKS=[['writer','mesh','drop beacon'],['mesh','van','RF 32 bytes'],['van','cp','uplink'],['van','ambulance',''],['van','firetruck',''],['van','drone','station link'],['ambulance','mesh','write state (RF)']];
let CP=null,SIM=null,seen=0,seenCP=0,prev={},dots=[],cnt={DROP:0,RF:0,UPLINK:0,DISPATCH:0,STATION:0,STATE:0,WRITE:0},started=false;
function el(t,a,p){const e=document.createElementNS(NS,t);for(const k in a)e.setAttribute(k,a[k]);(p||$('svg')).appendChild(e);return e}
function build(){const s=$('svg');
 for(const [a,b,l] of LINKS){const A=N[a],B=N[b];if(a=='ambulance'&&b=='mesh'){el('path',{d:`M470 272 L470 283 L90 283 L90 175`,stroke:'#3a8f5a','stroke-dasharray':'5 4',fill:'none','stroke-width':1.4});const t=el('text',{x:130,y:279,fill:'#7dff9a','font-size':10});t.textContent='Executors write the beacon state (RF)';continue}
  el('line',{x1:A[0],y1:A[1],x2:B[0],y2:B[1],stroke:'#2d4268','stroke-width':2});if(l){const t=el('text',{x:(A[0]+B[0])/2+4,y:(A[1]+B[1])/2-6,fill:'#7d8cab','font-size':10});t.textContent=l}}
 for(const k in N){const [x,y,l,c]=N[k];const w=k=='cp'||k=='van'?130:110;el('rect',{x:x-w/2,y:y-20,width:w,height:40,rx:8,fill:'#13213f',stroke:c,'stroke-width':2.2,id:'n_'+k});const t=el('text',{x,y:y+4,fill:c,'font-size':12,'font-weight':700,'text-anchor':'middle'});t.textContent=l}
 const lc=el('text',{x:20,y:20,fill:'#93a4c0','font-size':11,id:'lc'});lc.textContent='';
 requestAnimationFrame(anim)}
function route(e){const k=e.kind,r=e.src,d=e.dst;
 if(k=='DROP')return ['writer','mesh'];if(k=='RF')return ['mesh','van'];if(k=='UPLINK')return ['van','cp'];
 if(k=='DISPATCH')return ['cp','van',d];if(k=='STATION'||k=='STATE')return [r,'van','cp'];if(k=='WRITE')return [r,'MESH'];return null}
function spawn(e){const rt=route(e);if(!rt)return;cnt[e.kind]=(cnt[e.kind]||0)+1;let pts=[];
 for(const n of rt){if(n=='MESH'){const A=N[rt[0]];pts.push([A[0],A[1]+22],[A[0],283],[90,283],[90,175]);continue}pts.push([N[n][0],N[n][1]])}
 const col={DROP:'#40d0ff',RF:'#b48cff',UPLINK:'#ffd54d',DISPATCH:'#7fb0ff',STATION:'#c9d6ff',STATE:'#7dff9a',WRITE:'#7dff9a'}[e.kind]||'#fff';
 dots.push({pts,t0:performance.now(),col,el:el('circle',{r:6,fill:col,stroke:'#000'}),lab:e.label})}
function anim(now){for(const d of dots){const T=(now-d.t0)/1400;if(T>=1){d.el.remove();d.dead=true;continue}const n=d.pts.length-1,p=T*n,i=Math.min(n-1,Math.floor(p)),f=p-i,a=d.pts[i],b=d.pts[i+1];d.el.setAttribute('cx',a[0]+(b[0]-a[0])*f);d.el.setAttribute('cy',a[1]+(b[1]-a[1])*f)}
 dots=dots.filter(d=>!d.dead);if(dots.length>60)dots.splice(0,dots.length-60);requestAnimationFrame(anim)}
async function J(u){return (await fetch(u,{cache:'no-store'})).json()}
async function tick(){try{SIM=await J('/simstate');$('cplink').href=SIM.cp_url;try{CP=await J(SIM.cp_url.replace(/\/?$/,'/')+'state')}catch(e){CP=null}
  if(!started){started=true;build();seen=(SIM.flow.length?SIM.flow[SIM.flow.length-1].seq:0);seenCP=(CP&&CP.flow.length?CP.flow[CP.flow.length-1].seq:0)}
  const ev=[];SIM.flow.filter(e=>e.seq>seen).forEach(e=>{ev.push(Object.assign({src2:'sim'},e));seen=e.seq});if(CP)CP.flow.filter(e=>e.seq>seenCP).forEach(e=>{ev.push(Object.assign({src2:'cp'},e));seenCP=e.seq});
  ev.sort((a,b)=>a.t-b.t);ev.slice(-25).forEach((e,i)=>setTimeout(()=>spawn(e),i*90));
  if(ev.length){const h=ev.reverse().map(e=>'<div class="ev k-'+e.kind+'"><small>'+e.t+' s &middot; '+e.kind+'</small> '+e.src+' &rarr; '+e.dst+' <b>'+e.label+'</b></div>').join('');$('evs').innerHTML=h+$('evs').innerHTML;while($('evs').children.length>70)$('evs').lastChild.remove()}
  $('hud').textContent='sim time '+SIM.t+' s';$('cnt').innerHTML=Object.entries(cnt).map(([k,v])=>'<span>'+k+' '+v+'</span>').join('');table()}catch(e){}setTimeout(tick,330)}
function table(){const rows={};const ph=SIM.beacon_states||{};
 SIM.beacons.forEach(b=>rows[b.bid]={bid:b.bid,id:'BCN_'+String(b.bid).padStart(3,'0'),type:b.type,prio:b.prio,floor:b.f,cp:null,Q:null,conf:null,robot:null,where:'',upd:'',phys:(ph[b.bid]||['DROPPED'])[0],by:(ph[b.bid]||[])[1],pt:(ph[b.bid]||[])[2]});
 if(CP)CP.table.forEach(r=>{const o=rows[r.bid]||(rows[r.bid]={bid:r.bid,id:r.id,type:r.type,prio:r.prio,floor:r.floor,phys:'DROPPED'});o.cp=r.state;o.pi=r.pi;o.order=r.order;o.conf=r.conf;o.robot=r.robot;o.where=r.target||r.where;o.why=r.why;o.upd=r.by?(r.by+' @'+r.upd+' s'):''});
 let list=Object.values(rows);if($('f_rel').checked)list=list.filter(r=>r.type!='REPEATER'&&r.type!='STAIRS');
 list.sort((a,b)=>(a.pi==null?1e9:a.pi)-(b.pi==null?1e9:b.pi)||a.prio-b.prio||b.bid-a.bid);
 $('rows').innerHTML=list.map(r=>{const sig=r.cp+'|'+r.phys;const fl=prev[r.bid]!==undefined&&prev[r.bid]!==sig;prev[r.bid]=sig;
  const phys=(r.phys||'DROPPED')+(r.by&&r.by!='writer'?' <small>('+r.by+')</small>':'');
  return '<tr'+(fl?' class="flash"':'')+'><td>'+r.id+'</td><td>'+r.type+'</td><td>'+r.floor+'</td><td><span class="pr p'+r.prio+'">P'+r.prio+'</span></td><td><span class="st s-'+(r.cp||'none')+'">'+(r.cp||(CP?'not received yet':'Command Post unreachable'))+'</span>'+(r.why?'<br><small style="color:#93a4c0">'+r.why+'</small>':'')+'</td><td><span class="st s-'+(r.phys||'DROPPED')+'">'+phys+'</span></td><td>'+(r.robot||'')+(r.order?' <small>'+r.order+'</small>':'')+'</td><td>'+(r.conf==null?'':Math.round(r.conf*100)+'%')+'</td><td>'+(r.pi==null?'':r.pi.toFixed(2))+'</td><td>'+(r.upd||'')+'</td><td class="mut">'+(r.where||'')+'</td></tr>'}).join('');}
tick();
</script></body></html>"""
