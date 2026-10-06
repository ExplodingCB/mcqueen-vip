const $ = id => document.getElementById(id);
let track, snapshot, path = [], busy = false, aerial;
const keys = new Set();
async function api(url, body) {
  const response = await fetch(url, body ? {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(body)} : {});
  const data = await response.json();
  if (!response.ok) throw new Error(data.error || response.statusText);
  return data;
}
async function control(body) {
  try {
    if(['reset','mode','parameter'].includes(body.action)) path=[];
    update(await api('/api/control',body));
  } catch(error) { $('notice').textContent=error.message; }
}
$('play').onclick=()=>control({action:snapshot?.running?'pause':'play'});
$('step').onclick=()=>control({action:'step',manual:manualControls()});
$('reset').onclick=()=>{keys.clear();$('gas').value=$('pedal').value=$('steer').value=0;manualLabels();control({action:'reset'});};
$('mode').onchange=e=>control({action:'mode',mode:e.target.value});
$('speed').oninput=e=>$('cap').textContent=Number(e.target.value).toFixed(1)+' m/s';
$('speed').onchange=e=>control({action:'speed',value:Number(e.target.value)});
for(const [id,key,unit] of [['grip','friction',''],['mass','mass',' kg']]) {
  $(id).oninput=e=>$(id+'-value').textContent=Number(e.target.value).toFixed(id==='grip'?2:0)+unit;
  $(id).onchange=e=>control({action:'parameter',key,value:Number(e.target.value)});
}
function manualLabels() {
  $('steer-value').textContent=(Number($('steer').value)*180/Math.PI).toFixed(1)+' deg';
  $('gas-value').textContent=Math.round(Number($('gas').value)*100)+'%';
  $('brake-value').textContent=Math.round(Number($('pedal').value)*100)+'%';
}
for(const id of ['steer','gas','pedal']) $(id).oninput=manualLabels;
function manualControls() {
  const left=keys.has('a')||keys.has('arrowleft'),right=keys.has('d')||keys.has('arrowright');
  const gas=keys.has('w')||keys.has('arrowup'),brake=keys.has('s')||keys.has('arrowdown');
  return [left||right?(left?.35:0)-(right?.35:0):Number($('steer').value),gas?1:Number($('gas').value),brake?1:Number($('pedal').value)];
}
$('view').onchange=draw; $('aerial').onchange=draw; $('points').onchange=draw;
function syncSlider(id,value,label) {
  if(document.activeElement===$(id))return;
  $(id).value=value;$(id==='speed'?'cap':id+'-value').textContent=label;
}
function update(data) {
  snapshot=data;const s=data.state,r=data.report,p=r.vehicle_parameters;
  if(!$('mode').querySelector(`option[value="${CSS.escape(r.policy)}"]`)) {
    const option=document.createElement('option');option.value=r.policy;option.textContent=r.policy;$('mode').append(option);
  }
  $('mode').value=r.policy;$('manual').disabled=r.policy!=='manual';
  $('play').textContent=data.running?'Pause':'Run';$('play').disabled=Boolean(r.stop_reason);
  $('run-state').textContent=r.stop_reason?'Stopped':data.running?'Running':'Paused';
  $('notice').textContent=r.stop_reason||'';
  $('velocity').textContent=s.v.toFixed(2)+' m/s';$('time').textContent=s.t.toFixed(2)+' s';
  $('angle').textContent=(s.steer*180/Math.PI).toFixed(1)+' deg';
  $('throttle').textContent=Math.round(s.throttle*100)+'%';$('brake').textContent=Math.round(s.brake*100)+'%';
  $('lat').textContent=(s.a_lat/9.81).toFixed(2)+' g';$('yaw-rate').textContent=s.yaw_rate.toFixed(2)+' rad/s';
  $('clearance').textContent=r.minimum_body_clearance_m===null?'--':r.minimum_body_clearance_m.toFixed(2)+' m';
  $('progress').textContent=Math.max(0,100*r.progress_m/track.length).toFixed(1)+'%';
  $('laps').textContent=r.completed_laps;$('violations').textContent=r.boundary_violations;
  $('wheelbase').textContent=p.wheelbase.toFixed(2)+' m';$('body-width').textContent=(2*p.half_width).toFixed(2)+' m (estimate)';
  syncSlider('speed',data.speed_cap,data.speed_cap.toFixed(1)+' m/s');
  syncSlider('mass',p.mass,p.mass+' kg');syncSlider('grip',p.friction,p.friction.toFixed(2));
  const prev=path[path.length-1];if(!prev||Math.hypot(s.x-prev[0],s.y-prev[1])>.5)path.push([s.x,s.y]);
  if(path.length>6000)path.shift();draw();
}
function draw() {
  if(!track||!snapshot)return;
  const view=$('view').value;
  $('map').hidden=view!=='map';$('camera').hidden=view==='map';
  $('aerial-control').hidden=view!=='map'||!track.meta.aerial;
  $('points-control').hidden=view!=='map'||!track.meta.user_points_enu?.length;
  if(view!=='map') {
    $('camera').src=view==='camera'?snapshot.camera:view==='mask'?snapshot.mask:(snapshot.prediction||snapshot.mask);
    $('map-source').textContent=view==='prediction'&&!snapshot.prediction?'No prediction available. Showing true mask.':'Procedural camera, uncalibrated.';
    return;
  }
  const canvas=$('map'),rect=canvas.getBoundingClientRect(),ratio=window.devicePixelRatio||1;
  canvas.width=Math.round(rect.width*ratio);canvas.height=Math.round(rect.height*ratio);
  const ctx=canvas.getContext('2d');ctx.scale(ratio,ratio);const w=rect.width,h=rect.height;
  const points=[...track.left,...track.right];
  const xmin=Math.min(...points.map(p=>p[0]))-10,xmax=Math.max(...points.map(p=>p[0]))+10;
  const ymin=Math.min(...points.map(p=>p[1]))-10,ymax=Math.max(...points.map(p=>p[1]))+10;
  const scale=Math.min((w-20)/(xmax-xmin),(h-20)/(ymax-ymin));const cx=(xmin+xmax)/2,cy=(ymin+ymax)/2;
  const xy=p=>[w/2+(p[0]-cx)*scale,h/2-(p[1]-cy)*scale];
  function line(points,color,width,close=false){ctx.beginPath();points.forEach((p,i)=>i?ctx.lineTo(...xy(p)):ctx.moveTo(...xy(p)));if(close)ctx.closePath();ctx.strokeStyle=color;ctx.lineWidth=width;ctx.stroke();}
  ctx.fillStyle='#e8e9e4';ctx.fillRect(0,0,w,h);
  const showAerial=$('aerial').checked&&aerial?.complete&&aerial.naturalWidth>0&&track.meta.aerial;
  if(showAerial){const b=track.meta.aerial.bounds_enu,tl=xy([b[0],b[3]]),br=xy([b[2],b[1]]);ctx.drawImage(aerial,tl[0],tl[1],br[0]-tl[0],br[1]-tl[1]);}
  if(!showAerial)for(let i=0;i<track.left.length;i++){let j=(i+1)%track.left.length;ctx.beginPath();[track.left[i],track.left[j],track.right[j],track.right[i]].forEach((p,k)=>k?ctx.lineTo(...xy(p)):ctx.moveTo(...xy(p)));ctx.closePath();ctx.fillStyle='#777';ctx.fill();}
  line(track.left,showAerial?'#ffd342':'#444',showAerial?1:1.3,true);line(track.right,showAerial?'#ffd342':'#444',showAerial?1:1.3,true);
  if($('points').checked)for(const p of track.meta.user_points_enu||[]){ctx.beginPath();ctx.arc(...xy(p),2.5,0,Math.PI*2);ctx.fillStyle='#fff';ctx.fill();ctx.strokeStyle='#222';ctx.lineWidth=1;ctx.stroke();}
  for(const fraction of [.08,.38,.72]) {
    const i=Math.floor(track.center.length*fraction),a=xy(track.center[i]),b=xy(track.center[(i+3)%track.center.length]);
    ctx.save();ctx.translate(...a);ctx.rotate(Math.atan2(b[1]-a[1],b[0]-a[0]));
    ctx.beginPath();ctx.moveTo(5,0);ctx.lineTo(-4,-3);ctx.lineTo(-4,3);ctx.closePath();ctx.fillStyle='#fff';ctx.fill();ctx.strokeStyle='#333';ctx.lineWidth=.7;ctx.stroke();ctx.restore();
  }
  line(path,'#1595ff',2);line([track.left[0],track.right[0]],'#fff',3);
  const s=snapshot.state,p=snapshot.report.vehicle_parameters;ctx.save();ctx.translate(...xy([s.x,s.y]));ctx.rotate(-s.yaw);
  ctx.fillStyle='#e32424';ctx.fillRect(-p.rear_extent*scale,-p.half_width*scale,(p.front_extent+p.rear_extent)*scale,2*p.half_width*scale);
  ctx.beginPath();ctx.arc(0,0,7,0,Math.PI*2);ctx.strokeStyle='#e32424';ctx.lineWidth=1.5;ctx.stroke();ctx.beginPath();ctx.moveTo(8,0);ctx.lineTo(15,0);ctx.stroke();ctx.restore();
  ctx.fillStyle='#fff';ctx.fillRect(7,h-33,20*scale+10,26);ctx.fillStyle='#222';ctx.font='11px system-ui';ctx.fillText('20 m',12,h-16);ctx.fillRect(12,h-11,20*scale,1);ctx.fillText('N ↑',w-33,20);
  const source=track.meta.source?.attribution||'Provisional geometry';
  $('map-source').textContent=`${source}. ${showAerial?'Yellow: estimated':'Estimated'} track edges.`;
}
window.addEventListener('resize',draw);
const allowed=new Set(['w','a','s','d','arrowup','arrowdown','arrowleft','arrowright']);
window.addEventListener('keydown',e=>{const k=e.key.toLowerCase();if(allowed.has(k)&&snapshot?.report.policy==='manual'&&!['INPUT','SELECT'].includes(e.target.tagName)){keys.add(k);e.preventDefault();}});
window.addEventListener('keyup',e=>keys.delete(e.key.toLowerCase()));window.addEventListener('blur',()=>keys.clear());
document.addEventListener('visibilitychange',()=>{if(document.hidden){keys.clear();if(snapshot?.running)control({action:'pause'});}});
async function tick(){if(!busy&&snapshot?.running){busy=true;try{update(await api('/api/tick',{manual:manualControls()}));}catch(error){$('notice').textContent=error.message;snapshot.running=false;}finally{busy=false;}}setTimeout(tick,100);}
async function init(){try{track=await api('/api/track');if(track.meta.aerial){aerial=new Image();aerial.onload=draw;aerial.src='/api/aerial.png';}$('aerial').disabled=!track.meta.aerial;update(await api('/api/state'));tick();}catch(error){$('notice').textContent=error.message;}}
init();
