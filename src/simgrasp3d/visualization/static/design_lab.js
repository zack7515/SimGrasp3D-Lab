(() => {
  'use strict';
  const DATA = JSON.parse(document.getElementById('lab-data').textContent);
  const G = DATA.geometry, T = DATA.thresholds;
  const values = Object.assign({}, DATA.baselineValues);
  const baselineValues = Object.assign({}, DATA.baselineValues);
  const experimentLog = [];
  const add=(a,b)=>a.map((v,i)=>v+b[i]), sub=(a,b)=>a.map((v,i)=>v-b[i]), mul=(a,s)=>a.map(v=>v*s);
  const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0), norm=a=>Math.sqrt(dot(a,a));
  const unit=a=>{const n=norm(a);return n<1e-12?[0,0,0]:mul(a,1/n)};
  const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
  function segmentDistance(p1,q1,p2,q2) {
    const d1=sub(q1,p1), d2=sub(q2,p2), r=sub(p1,p2), a=dot(d1,d1), e=dot(d2,d2), eps=1e-12;
    let s=0,t=0;
    if(a<=eps&&e<=eps)return norm(r);
    if(a<=eps) t=Math.max(0,Math.min(1,dot(d2,r)/e));
    else {
      const c=dot(d1,r);
      if(e<=eps)s=Math.max(0,Math.min(1,-c/a));
      else {const b=dot(d1,d2),den=a*e-b*b;s=den!==0?Math.max(0,Math.min(1,(b*dot(d2,r)-c*e)/den)):0;t=(b*s+dot(d2,r))/e;if(t<0){t=0;s=Math.max(0,Math.min(1,-c/a))}else if(t>1){t=1;s=Math.max(0,Math.min(1,(b-c)/a))}}
    }
    return norm(sub(add(p1,mul(d1,s)),add(p2,mul(d2,t))));
  }
  const toolRadius=()=>Math.max(G.toolEnvelopeRadius,values.gripper_command_m/2+0.025);
  const obstacles=()=>G.obstacles.map(o=>({...o,radius:o.radius*values.obstacle_radius_scale}));
  function clearance(a,b) {return Math.min(...obstacles().map(o=>segmentDistance(a,b,o.start,o.end)-toolRadius()-o.radius))}
  function candidateOffsets() {const out=[];for(let d=G.detourStep;d<=G.maximumDetour+1e-10;d+=G.detourStep)out.push([0,0,d],[0,d,d*.5],[0,-d,d*.5],[d,0,d*.5],[-d,0,d*.5]);return out}
  function planPath(grasp,goal) {
    const lift=values.lift_height_m, approach=Math.max(.10,Math.min(.18,lift*.55));
    const raw=[G.startPoint,add(grasp,[0,0,approach]),grasp,grasp,add(grasp,[0,0,lift]),add(goal,[0,0,lift]),goal,goal,add(goal,[0,0,lift])];
    const planned=[raw[0]];let inserted=0,unresolved=0;
    for(let i=1;i<raw.length;i++){const a=planned[planned.length-1],b=raw[i];if(norm(sub(a,b))<1e-10){planned.push(b);continue}if(clearance(a,b)>=values.safety_margin_m){planned.push(b);continue}const mid=mul(add(a,b),.5), candidates=[];candidateOffsets().forEach(off=>{const p=add(mid,off),c=Math.min(clearance(a,p),clearance(p,b));if(c>=values.safety_margin_m)candidates.push([norm(sub(p,a))+norm(sub(b,p)),p])});if(!candidates.length)unresolved++;else{candidates.sort((x,y)=>x[0]-y[0]);planned.push(candidates[0][1]);inserted++}planned.push(b)}
    return {path:planned,inserted,unresolved};
  }
  function cameraCoverage(points,pos,look) {
    const f=unit(sub(look,pos)),r0=cross(f,[0,0,1]),r=norm(r0)<1e-9?[1,0,0]:unit(r0),u=unit(cross(r,f)),tan=Math.tan(values.camera_fov_deg*Math.PI/360);let frustum=0,visible=0;
    points.forEach(p=>{const rel=sub(p,pos),z=dot(rel,f),inside=z>=G.cameraNear&&z<=G.cameraFar&&Math.abs(dot(rel,u))<=z*tan&&Math.abs(dot(rel,r))<=z*tan*G.cameraAspect;if(!inside)return;frustum++;let blocked=false;for(const o of obstacles()){if(segmentDistance(pos,p,o.start,o.end)<=o.radius){blocked=true;break}}if(!blocked)visible++});
    return {frustum:frustum/points.length,visible:visible/points.length};
  }
  const fmt=(v,u)=>u==='ratio'?`${(v*100).toFixed(1)}%`:u==='m'?`${(v*1000).toFixed(1)} mm`:u==='deg'?`${v.toFixed(1)}°`:u==='x'?`${v.toFixed(2)}×`:v.toFixed(3);
  function evaluate() {
    const index=Math.round(values.grasp_fraction*(G.hosePoints.length-1)),grasp=[...G.hosePoints[index]],goal=[...G.goalPoint];grasp[2]=Math.max(grasp[2],G.tableTopZ+values.hose_radius_m);goal[2]=Math.max(goal[2],G.tableTopZ+values.hose_radius_m);
    const cam=[G.cameraX,values.camera_lateral_m,values.camera_height_m],coverage=cameraCoverage([...G.hosePoints,goal],cam,G.cameraLookAt),distance=norm(sub(grasp,cam)),n=G.noise;
    const axial=n.axial_noise_std_base_m+n.axial_noise_std_per_m2*distance*distance,rot=distance*n.extrinsic_rotation_std_deg*Math.PI/180,unc=3*Math.sqrt(axial*axial+n.extrinsic_translation_std_m**2+rot*rot+(n.depth_quantization_m/Math.sqrt(12))**2)*values.depth_noise_scale;
    const plan=planPath(grasp,goal),shoulder=G.shoulderPosition,maxReach=G.nominalReach*values.arm_reach_scale,maxRequest=Math.max(...plan.path.map(p=>norm(sub(p,shoulder)))),reserve=maxReach-maxRequest;
    const gripError=Math.abs(values.gripper_command_m-2*values.hose_radius_m);let minClear=Infinity;for(let i=0;i<plan.path.length-1;i++)if(norm(sub(plan.path[i],plan.path[i+1]))>1e-10)minClear=Math.min(minClear,clearance(plan.path[i],plan.path[i+1]));
    const meta=DATA.baselineGates.reduce((o,g)=>(o[g.key]=g,o),{}), gates=[
      {...meta.camera_coverage,value:coverage.visible,limit:T.minimum_visibility_ratio,passed:coverage.visible>=T.minimum_visibility_ratio},
      {...meta.depth_uncertainty,value:unc,limit:T.maximum_depth_uncertainty_m,passed:unc<=T.maximum_depth_uncertainty_m},
      {...meta.reach_reserve,value:reserve,limit:T.minimum_reach_reserve_m,passed:reserve>=T.minimum_reach_reserve_m},
      {...meta.gripper_match,value:gripError,limit:T.maximum_gripper_diameter_error_m,passed:gripError<=T.maximum_gripper_diameter_error_m},
      {...meta.bend_radius,value:G.minimumBendRadius,limit:values.hose_min_bend_radius_m,passed:G.minimumBendRadius>=values.hose_min_bend_radius_m},
      {...meta.path_clearance,value:minClear,limit:values.safety_margin_m,passed:minClear>=values.safety_margin_m&&plan.unresolved===0}
    ];
    const length=plan.path.slice(1).reduce((s,p,i)=>s+norm(sub(p,plan.path[i])),0);
    return {grasp,goal,cam,coverage,unc,reserve,maxReach,plan,minClear,length,gates};
  }
  function frustumLines(cam) {
    const f=unit(sub(G.cameraLookAt,cam)),r=unit(cross(f,[0,0,1])),u=unit(cross(r,f)),d=Math.min(G.cameraFar,1.55),hh=Math.tan(values.camera_fov_deg*Math.PI/360)*d,hw=hh*G.cameraAspect,c=add(cam,mul(f,d));
    const corners=[add(add(c,mul(r,-hw)),mul(u,-hh)),add(add(c,mul(r,hw)),mul(u,-hh)),add(add(c,mul(r,hw)),mul(u,hh)),add(add(c,mul(r,-hw)),mul(u,hh))],x=[],y=[],z=[];
    const seg=(a,b)=>{x.push(a[0],b[0],null);y.push(a[1],b[1],null);z.push(a[2],b[2],null)};corners.forEach((p,i)=>{seg(cam,p);seg(p,corners[(i+1)%4])});return {x,y,z}
  }
  function ring(center,radius,plane) {const pts=[];for(let i=0;i<=72;i++){const a=2*Math.PI*i/72;pts.push(plane==='xy'?[center[0]+radius*Math.cos(a),center[1]+radius*Math.sin(a),center[2]]:[center[0]+radius*Math.cos(a),center[1],center[2]+radius*Math.sin(a)])}return pts}
  function renderPlot(state) {
    const table=G.tableCenter,half=G.tableSize.map(v=>v/2),verts=[];[[-1,-1,-1],[1,-1,-1],[1,1,-1],[-1,1,-1],[-1,-1,1],[1,-1,1],[1,1,1],[-1,1,1]].forEach(s=>verts.push(s.map((v,i)=>table[i]+v*half[i])));
    const traces=[{type:'mesh3d',x:verts.map(p=>p[0]),y:verts.map(p=>p[1]),z:verts.map(p=>p[2]),i:[0,0,4,4,0,0,1,1,2,2,3,3],j:[1,2,5,6,1,5,2,6,3,7,0,4],k:[2,3,6,7,5,4,6,5,7,6,4,7],color:'#bccac3',opacity:.35,name:'工作桌',hoverinfo:'skip'},
      {type:'scatter3d',mode:'lines',x:G.hosePoints.map(p=>p[0]),y:G.hosePoints.map(p=>p[1]),z:G.hosePoints.map(p=>p[2]),line:{color:'#06999c',width:Math.max(5,values.hose_radius_m*300)},name:'軟管中心線'},
      {type:'scatter3d',mode:'lines+markers',x:state.plan.path.map(p=>p[0]),y:state.plan.path.map(p=>p[1]),z:state.plan.path.map(p=>p[2]),line:{color:state.gates[5].passed?'#d68d22':'#c64d3f',width:6},marker:{size:3,color:'#d68d22'},name:'規劃 TCP 路徑'},
      {type:'scatter3d',mode:'markers',x:[state.grasp[0]],y:[state.grasp[1]],z:[state.grasp[2]],marker:{size:8,color:'#f0ad31',symbol:'diamond',line:{color:'#fff',width:2}},name:'夾取點'},
      {type:'scatter3d',mode:'markers',x:[state.goal[0]],y:[state.goal[1]],z:[state.goal[2]],marker:{size:9,color:'#7656a5',symbol:'circle-open',line:{width:4}},name:'放置點'},
      {type:'scatter3d',mode:'markers+text',x:[state.cam[0]],y:[state.cam[1]],z:[state.cam[2]],text:['RGB-D'],textposition:'top center',marker:{size:7,color:'#058b91'},name:'相機'},
    ];
    obstacles().forEach(o=>traces.push({type:'scatter3d',mode:'lines',x:[o.start[0],o.end[0]],y:[o.start[1],o.end[1]],z:[o.start[2],o.end[2]],line:{color:'#52656c',width:Math.max(9,o.radius*260)},name:o.name,showlegend:false}));
    const fr=frustumLines(state.cam);traces.push({type:'scatter3d',mode:'lines',...fr,line:{color:'#058b91',width:2,dash:'dot'},name:'相機視錐',showlegend:false});
    ['xy','xz'].forEach(plane=>{const pts=ring(G.shoulderPosition,state.maxReach,plane);traces.push({type:'scatter3d',mode:'lines',x:pts.map(p=>p[0]),y:pts.map(p=>p[1]),z:pts.map(p=>p[2]),line:{color:'#7a8e94',width:2,dash:'dash'},opacity:.55,name:'工作空間包絡',showlegend:false})});
    const scaledJoints=G.robotJointPositions.map(p=>add(G.shoulderPosition,mul(sub(p,G.shoulderPosition),values.arm_reach_scale))),scaledTool=add(G.shoulderPosition,mul(sub(G.robotToolPosition,G.shoulderPosition),values.arm_reach_scale));scaledJoints.push(scaledTool);
    traces.push({type:'scatter3d',mode:'lines+markers',x:scaledJoints.map(p=>p[0]),y:scaledJoints.map(p=>p[1]),z:scaledJoints.map(p=>p[2]),line:{color:'#233c44',width:10},marker:{size:5,color:'#d68d22',line:{color:'#fff',width:1}},name:'六軸手臂結構'});
    traces.push({type:'scatter3d',mode:'lines+markers',x:[G.basePosition[0],G.shoulderPosition[0]],y:[G.basePosition[1],G.shoulderPosition[1]],z:[G.basePosition[2],G.shoulderPosition[2]],line:{color:'#233c44',width:14},marker:{size:7,color:'#d68d22'},name:'機械手基座',showlegend:false});
    const halfGrip=values.gripper_command_m/2,gx=state.grasp[0],gy=state.grasp[1],gz=state.grasp[2]+.018;traces.push({type:'scatter3d',mode:'lines',x:[gx-.07,gx,null,gx-.07,gx],y:[gy+halfGrip,gy+halfGrip,null,gy-halfGrip,gy-halfGrip],z:[gz,gz,gz,gz,gz],line:{color:state.gates[3].passed?'#d68d22':'#c64d3f',width:8},name:'閉爪開口示意'});
    Plotly.react('design-view',traces,{margin:{l:0,r:0,t:0,b:0},paper_bgcolor:'#fbfcfa',showlegend:true,legend:{orientation:'h',x:0,y:1.02,font:{size:9}},font:{family:'Aptos, Noto Sans TC, sans-serif',color:'#17262b',size:10},scene:{xaxis:{title:'X (m)',range:[-.9,.95],backgroundcolor:'#edf3f0',gridcolor:'#c8d5d1',showbackground:true},yaxis:{title:'Y (m)',range:[-.75,.75],backgroundcolor:'#edf3f0',gridcolor:'#c8d5d1',showbackground:true},zaxis:{title:'Z (m)',range:[0,1.65],backgroundcolor:'#edf3f0',gridcolor:'#c8d5d1',showbackground:true},aspectmode:'manual',aspectratio:{x:1.1,y:.9,z:1},camera:{eye:{x:1.5,y:-1.6,z:1.0}},uirevision:'system-design'},responsive:true},{displaylogo:false,responsive:true,scrollZoom:true});
  }
  function renderGates(state) {
    const passed=state.gates.filter(g=>g.passed).length;document.getElementById('gate-score').innerHTML=`${passed} <span>/ ${state.gates.length} PASS</span>`;
    document.getElementById('gate-grid').innerHTML=state.gates.map(g=>`<article class="gate ${g.passed?'':'fail'}"><span class="gate-layer">${g.layer}</span><span class="gate-state">${g.passed?'PASS':'STOP'}</span><strong class="gate-value">${fmt(g.value,g.unit)}</strong><h3>${g.label}</h3><div class="limit">TEACHING GATE ${g.relation} ${fmt(g.limit,g.unit)} · 未校準</div><p>${g.explanation}</p><p class="gate-action">調整方向：${g.action}</p></article>`).join('');
    const baseline=DATA.baselineGates.reduce((o,g)=>(o[g.key]=g,o),{});document.getElementById('compare-body').innerHTML=state.gates.map(g=>{const b=baseline[g.key],same=g.passed===b.passed;return `<tr><td>${g.label}</td><td>${fmt(b.value,b.unit)} / ${b.passed?'PASS':'STOP'}</td><td>${fmt(g.value,g.unit)}</td><td class="${g.passed?'delta-good':'delta-bad'}">${g.passed?'PASS':'STOP'}${same?'':' / 狀態改變'}</td><td>${g.action}</td></tr>`}).join('');
  }
  function update() {document.querySelectorAll('.preset').forEach(x=>x.classList.remove('active'));const state=evaluate();renderPlot(state);renderGates(state);return state}
  function displayParameter(p,value) {return p.unit==='ratio'?`${Math.round(value*100)}%`:p.unit==='m'?`${(value*1000).toFixed(0)} mm`:p.unit==='deg'?`${value.toFixed(0)}°`:p.unit==='x'?`${value.toFixed(2)}×`:String(value)}
  function buildControls() {
    const groups={};DATA.parameters.forEach(p=>(groups[p.group]??=[]).push(p));const host=document.getElementById('control-groups');
    Object.entries(groups).forEach(([name,params],groupIndex)=>{const details=document.createElement('details');details.className='control-group';details.open=groupIndex===0;const summary=document.createElement('summary');summary.textContent=`${String(groupIndex+1).padStart(2,'0')} / ${name}`;details.appendChild(summary);const list=document.createElement('div');list.className='control-list';params.forEach(p=>{const row=document.createElement('div');row.className='control';row.innerHTML=`<label class="control-title" for="control-${p.key}"><span>${p.label}</span><output class="control-output" id="value-${p.key}"></output></label><input id="control-${p.key}" type="range" min="${p.minimum}" max="${p.maximum}" step="${p.step}" value="${p.value}"><div class="control-scale"><span>${displayParameter(p,p.minimum)}</span><span>${displayParameter(p,p.maximum)}</span></div><p>${p.description}</p>`;list.appendChild(row);const input=row.querySelector('input'),output=row.querySelector('output');const sync=()=>{values[p.key]=Number(input.value);output.value=displayParameter(p,values[p.key]);update()};input.addEventListener('input',sync);output.value=displayParameter(p,p.value)});details.appendChild(list);host.appendChild(details)});
    const presets=document.getElementById('preset-list');DATA.presets.forEach((p,i)=>{const button=document.createElement('button');button.className='preset';button.textContent=p.name;button.title=p.description;button.addEventListener('click',()=>{Object.assign(values,baselineValues,p.values);syncInputs();update();button.classList.add('active')});presets.appendChild(button);if(i===0)button.classList.add('active')});
  }
  function syncInputs() {DATA.parameters.forEach(p=>{const input=document.getElementById(`control-${p.key}`),output=document.getElementById(`value-${p.key}`);input.value=values[p.key];output.value=displayParameter(p,values[p.key])})}
  document.getElementById('reset-design').addEventListener('click',()=>{Object.assign(values,baselineValues);syncInputs();update();document.querySelector('.preset')?.classList.add('active')});
  document.getElementById('record-design').addEventListener('click',()=>{const s=evaluate(),failed=s.gates.filter(g=>!g.passed);experimentLog.push({time:new Date().toLocaleTimeString('zh-TW'),pass:s.gates.length-failed.length,stop:failed.length,length:s.length,waypoint:s.plan.inserted,fail:failed.map(g=>g.label).join('、')||'無'});document.getElementById('log-empty').style.display='none';document.getElementById('log-table').style.display='table';document.getElementById('log-body').innerHTML=experimentLog.map((r,i)=>`<tr><td>${i+1}</td><td>${r.time}</td><td>${r.pass}</td><td>${r.stop}</td><td>${r.length.toFixed(3)} m</td><td>${r.waypoint}</td><td>${r.fail}</td></tr>`).join('')});
  function download(name,text,type) {const blob=new Blob([text],{type}),url=URL.createObjectURL(blob),a=document.createElement('a');a.href=url;a.download=name;a.click();setTimeout(()=>URL.revokeObjectURL(url),500)}
  document.getElementById('download-config').addEventListener('click',()=>{const config=structuredClone(DATA.rawSpec);config.parameters.forEach(p=>p.value=values[p.key]);download('system_design_lab.json',JSON.stringify(config,null,2),'application/json')});
  document.getElementById('export-log').addEventListener('click',()=>{const rows=[['index','time','pass','stop','path_length_m','waypoint_count','failed_gates'],...experimentLog.map((r,i)=>[i+1,r.time,r.pass,r.stop,r.length.toFixed(6),r.waypoint,r.fail])];download('system_design_experiments.csv',rows.map(r=>r.map(v=>`"${String(v).replaceAll('"','""')}"`).join(',')).join('\\n'),'text/csv;charset=utf-8')});
  buildControls();update();window.addEventListener('resize',()=>Plotly.Plots.resize('design-view'));
})();
