// SiliconForge — Production-Grade HTML5/Canvas Layout Visualizer
#include "viz/html_export.hpp"
#include <fstream>
#include <sstream>
#include <cmath>

namespace sf {

void HtmlVisualizer::set_congestion_map(const std::vector<std::vector<double>>& map) {
    congestion_map_ = map;
}
void HtmlVisualizer::set_power_map(const std::vector<std::vector<double>>& map) {
    power_map_ = map;
}
void HtmlVisualizer::set_timing_paths(const std::vector<VizTimingPath>& paths) {
    timing_paths_ = paths;
}
void HtmlVisualizer::set_drc_markers(const std::vector<VizDrcMarker>& markers) {
    drc_markers_ = markers;
}
void HtmlVisualizer::set_placement_history(const std::vector<PlacementSnapshot>& history) {
    placement_history_ = history;
}
void HtmlVisualizer::highlight_net(const std::string& net_name) {
    highlight_net_ = net_name;
}

// ═══════════════════════════════════════════════════════════════════════════════
// JSON DATA GENERATION
// ═══════════════════════════════════════════════════════════════════════════════

static std::string escape_js(const std::string& s) {
    std::string out;
    out.reserve(s.size());
    for (char c : s) {
        if (c == '\'' || c == '\\') out += '\\';
        out += c;
    }
    return out;
}

std::string HtmlVisualizer::generate_json_data() const {
    std::ostringstream ss;
    ss << std::fixed;
    ss.precision(3);
    ss << "const designData = {\n";

    // Die area
    ss << "die:{w:" << pd_.die_area.width() << ",h:" << pd_.die_area.height()
       << ",x0:" << pd_.die_area.x0 << ",y0:" << pd_.die_area.y0 << "},\n";

    // Routing layers
    ss << "layers:[";
    for (size_t i = 0; i < pd_.layers.size(); ++i) {
        auto& l = pd_.layers[i];
        ss << "{n:'" << escape_js(l.name) << "',hz:" << (l.horizontal ? 1 : 0)
           << ",p:" << l.pitch << ",w:" << l.width << ",s:" << l.spacing << "}";
        if (i + 1 < pd_.layers.size()) ss << ",";
    }
    ss << "],\n";

    // Cells
    ss << "cells:[\n";
    for (size_t i = 0; i < pd_.cells.size(); ++i) {
        auto& c = pd_.cells[i];
        ss << "[" << c.position.x << "," << c.position.y << "," << c.width << "," << c.height
           << ",'" << escape_js(c.cell_type) << "','" << escape_js(c.name) << "',"
           << c.orientation << "," << (c.is_macro ? 1 : 0) << "]";
        if (i + 1 < pd_.cells.size()) ss << ",";
        if (i % 10 == 9) ss << "\n";
    }
    ss << "],\n";

    // Wires
    ss << "wires:[\n";
    for (size_t i = 0; i < pd_.wires.size(); ++i) {
        auto& w = pd_.wires[i];
        ss << "[" << w.layer << "," << w.start.x << "," << w.start.y << ","
           << w.end.x << "," << w.end.y << "," << w.width << "," << w.net_id << "]";
        if (i + 1 < pd_.wires.size()) ss << ",";
        if (i % 10 == 9) ss << "\n";
    }
    ss << "],\n";

    // Vias
    ss << "vias:[\n";
    for (size_t i = 0; i < pd_.vias.size(); ++i) {
        auto& v = pd_.vias[i];
        ss << "[" << v.position.x << "," << v.position.y << "," << v.lower_layer << "," << v.upper_layer << "]";
        if (i + 1 < pd_.vias.size()) ss << ",";
    }
    ss << "],\n";

    // IO Pins
    ss << "pins:[\n";
    for (size_t i = 0; i < pd_.io_pins.size(); ++i) {
        auto& p = pd_.io_pins[i];
        ss << "{n:'" << escape_js(p.name) << "',d:'" << escape_js(p.direction)
           << "',x:" << p.position.x << ",y:" << p.position.y << ",l:" << p.layer << "}";
        if (i + 1 < pd_.io_pins.size()) ss << ",";
    }
    ss << "],\n";

    // Special nets (power/ground stripes)
    ss << "snets:[\n";
    for (size_t i = 0; i < pd_.special_nets.size(); ++i) {
        auto& sn = pd_.special_nets[i];
        ss << "{n:'" << escape_js(sn.name) << "',use:'" << escape_js(sn.use) << "',w:[";
        for (size_t j = 0; j < sn.wires.size(); ++j) {
            auto& sw = sn.wires[j];
            ss << "[" << sw.width << "," << sw.start.x << "," << sw.start.y
               << "," << sw.end.x << "," << sw.end.y << ",'" << escape_js(sw.shape) << "']";
            if (j + 1 < sn.wires.size()) ss << ",";
        }
        ss << "]}";
        if (i + 1 < pd_.special_nets.size()) ss << ",";
    }
    ss << "],\n";

    // Blockages
    ss << "blocks:[\n";
    for (size_t i = 0; i < pd_.blockages.size(); ++i) {
        auto& b = pd_.blockages[i];
        ss << "[" << b.layer << "," << b.rect.x0 << "," << b.rect.y0 << ","
           << b.rect.x1 << "," << b.rect.y1 << ",'" << escape_js(b.type) << "']";
        if (i + 1 < pd_.blockages.size()) ss << ",";
    }
    ss << "],\n";

    // Placement rows
    ss << "rows:[\n";
    for (size_t i = 0; i < pd_.placement_rows.size(); ++i) {
        auto& r = pd_.placement_rows[i];
        ss << "[" << r.origin_x << "," << r.origin_y << "," << r.num_x << ","
           << r.step_x << "," << r.num_y << "," << r.step_y << "]";
        if (i + 1 < pd_.placement_rows.size()) ss << ",";
    }
    ss << "],\n";

    // Nets (for net highlighting)
    ss << "nets:[\n";
    for (size_t i = 0; i < pd_.nets.size(); ++i) {
        auto& n = pd_.nets[i];
        ss << "{n:'" << escape_js(n.name) << "',c:[";
        for (size_t j = 0; j < n.cell_ids.size(); ++j) {
            ss << n.cell_ids[j];
            if (j + 1 < n.cell_ids.size()) ss << ",";
        }
        ss << "]}";
        if (i + 1 < pd_.nets.size()) ss << ",";
    }
    ss << "],\n";

    // Congestion map
    if (!congestion_map_.empty()) {
        ss << "cong:[\n";
        for (size_t y = 0; y < congestion_map_.size(); ++y) {
            ss << "[";
            for (size_t x = 0; x < congestion_map_[y].size(); ++x) {
                ss << congestion_map_[y][x];
                if (x + 1 < congestion_map_[y].size()) ss << ",";
            }
            ss << "]";
            if (y + 1 < congestion_map_.size()) ss << ",\n";
        }
        ss << "],\n";
    }

    // Power map
    if (!power_map_.empty()) {
        ss << "pwr:[\n";
        for (size_t y = 0; y < power_map_.size(); ++y) {
            ss << "[";
            for (size_t x = 0; x < power_map_[y].size(); ++x) {
                ss << power_map_[y][x];
                if (x + 1 < power_map_[y].size()) ss << ",";
            }
            ss << "]";
            if (y + 1 < power_map_.size()) ss << ",\n";
        }
        ss << "],\n";
    }

    // Timing paths
    if (!timing_paths_.empty()) {
        ss << "tpaths:[\n";
        for (size_t i = 0; i < timing_paths_.size(); ++i) {
            auto& tp = timing_paths_[i];
            ss << "{n:'" << escape_js(tp.name) << "',s:" << tp.slack << ",c:[";
            for (size_t j = 0; j < tp.cell_ids.size(); ++j) {
                ss << tp.cell_ids[j];
                if (j + 1 < tp.cell_ids.size()) ss << ",";
            }
            ss << "],nets:[";
            for (size_t j = 0; j < tp.net_ids.size(); ++j) {
                ss << tp.net_ids[j];
                if (j + 1 < tp.net_ids.size()) ss << ",";
            }
            ss << "]}";
            if (i + 1 < timing_paths_.size()) ss << ",";
        }
        ss << "],\n";
    }

    // DRC markers
    if (!drc_markers_.empty()) {
        ss << "drc:[\n";
        for (size_t i = 0; i < drc_markers_.size(); ++i) {
            auto& m = drc_markers_[i];
            ss << "{x:" << m.x << ",y:" << m.y << ",t:'" << escape_js(m.type)
               << "',m:'" << escape_js(m.message) << "',s:" << m.severity << "}";
            if (i + 1 < drc_markers_.size()) ss << ",";
        }
        ss << "],\n";
    }

    // Placement animation snapshots
    if (!placement_history_.empty()) {
        ss << "snaps:[\n";
        for (size_t i = 0; i < placement_history_.size(); ++i) {
            auto& snap = placement_history_[i];
            ss << "{it:" << snap.iteration << ",cost:" << snap.cost << ",p:[";
            for (size_t j = 0; j < snap.positions.size(); ++j) {
                ss << "[" << snap.positions[j].x << "," << snap.positions[j].y << "]";
                if (j + 1 < snap.positions.size()) ss << ",";
            }
            ss << "]}";
            if (i + 1 < placement_history_.size()) ss << ",\n";
        }
        ss << "],\n";
    }

    // Pre-highlighted net
    if (!highlight_net_.empty()) {
        ss << "hlNet:'" << escape_js(highlight_net_) << "',\n";
    }

    ss << "};\nconst D=designData;\n";
    return ss.str();
}

// ═══════════════════════════════════════════════════════════════════════════════
// CSS
// ═══════════════════════════════════════════════════════════════════════════════

std::string HtmlVisualizer::generate_css() const {
    return R"CSS(
@import url('https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600;700&family=JetBrains+Mono:wght@400;500&display=swap');
*{margin:0;padding:0;box-sizing:border-box;}
:root{
  --bg:#08080d;--bg2:#10101a;--bg3:#181828;--panel:rgba(12,12,22,0.92);
  --accent:#00d4ff;--accent2:#7c3aed;--green:#10b981;--red:#ef4444;
  --warn:#f59e0b;--pink:#ec4899;--text:#e0e0f0;--text2:#7878a0;
  --border:rgba(60,60,100,0.3);--glow:rgba(0,212,255,0.15);
  --mono:'JetBrains Mono',monospace;--sans:'Inter',sans-serif;
}
body{background:var(--bg);color:var(--text);font-family:var(--sans);overflow:hidden;height:100vh;}
#app{display:flex;flex-direction:column;height:100vh;}

/* Top bar */
#topbar{height:42px;background:var(--panel);border-bottom:1px solid var(--border);
  display:flex;align-items:center;padding:0 12px;gap:10px;z-index:50;
  backdrop-filter:blur(12px);}
#topbar .logo{font:600 13px var(--mono);color:var(--accent);letter-spacing:0.08em;
  text-transform:uppercase;white-space:nowrap;}
#topbar .sep{width:1px;height:20px;background:var(--border);}
#search{background:rgba(255,255,255,0.04);border:1px solid var(--border);
  border-radius:6px;padding:5px 10px;font:12px var(--mono);color:var(--text);
  width:220px;outline:none;transition:border-color 0.2s;}
#search:focus{border-color:var(--accent);}
#search-results{position:absolute;top:42px;left:180px;background:var(--panel);
  border:1px solid var(--border);border-radius:8px;max-height:240px;overflow-y:auto;
  z-index:100;min-width:250px;display:none;backdrop-filter:blur(12px);}
#search-results .sr{padding:6px 12px;font:12px var(--mono);color:var(--text2);
  cursor:pointer;border-bottom:1px solid rgba(60,60,100,0.15);}
#search-results .sr:hover{background:var(--glow);color:var(--accent);}
.tbtn{background:rgba(255,255,255,0.03);border:1px solid var(--border);
  border-radius:6px;padding:4px 10px;font:11px var(--mono);color:var(--text2);
  cursor:pointer;transition:all 0.15s;user-select:none;}
.tbtn:hover{border-color:var(--accent);color:var(--accent);}
.tbtn.on{background:var(--glow);border-color:var(--accent);color:var(--accent);}

/* Main area */
#main{display:flex;flex:1;overflow:hidden;position:relative;}

/* Left panel */
#lpanel{width:200px;background:var(--panel);border-right:1px solid var(--border);
  overflow-y:auto;padding:8px 0;font:11px var(--mono);flex-shrink:0;z-index:40;
  backdrop-filter:blur(12px);transition:margin-left 0.2s;}
#lpanel.collapsed{margin-left:-200px;}
.ptitle{padding:6px 12px;font-size:9px;font-weight:700;text-transform:uppercase;
  letter-spacing:1px;color:var(--text2);}
.ptitle:not(:first-child){margin-top:6px;border-top:1px solid var(--border);padding-top:8px;}
.lrow{display:flex;align-items:center;gap:6px;padding:3px 12px;cursor:pointer;
  transition:background 0.1s;user-select:none;}
.lrow:hover{background:rgba(255,255,255,0.03);}
.lrow input{accent-color:var(--accent);}
.lrow .sw{width:14px;height:4px;border-radius:2px;flex-shrink:0;}
.lrow span{color:var(--text2);font-size:11px;}
.lrow.off span{opacity:0.35;}
.slider-row{padding:4px 12px;display:flex;align-items:center;gap:6px;}
.slider-row input[type=range]{flex:1;height:3px;accent-color:var(--accent);}
.slider-row .sv{font-size:10px;color:var(--text2);min-width:28px;text-align:right;}

/* Canvas */
#cv{flex:1;display:block;cursor:crosshair;}

/* Right panel (inspector) */
#rpanel{width:240px;background:var(--panel);border-left:1px solid var(--border);
  overflow-y:auto;padding:10px;font:11px var(--mono);flex-shrink:0;z-index:40;
  display:none;backdrop-filter:blur(12px);}
#rpanel.show{display:block;}
.irow{display:flex;justify-content:space-between;padding:3px 0;border-bottom:1px solid rgba(60,60,100,0.12);}
.irow .ik{color:var(--text2);}
.irow .iv{color:var(--text);font-weight:500;text-align:right;max-width:140px;
  overflow:hidden;text-overflow:ellipsis;white-space:nowrap;}
.isec{font-size:9px;font-weight:700;text-transform:uppercase;letter-spacing:0.8px;
  color:var(--accent);margin:8px 0 4px;padding-top:6px;border-top:1px solid var(--border);}

/* Overlays */
#coords{position:absolute;bottom:8px;left:210px;background:var(--panel);
  border:1px solid var(--border);border-radius:6px;padding:4px 10px;
  font:11px var(--mono);color:var(--text2);z-index:30;backdrop-filter:blur(8px);}
#minimap{position:absolute;bottom:8px;right:8px;border:1px solid var(--border);
  border-radius:8px;z-index:30;cursor:pointer;background:var(--bg);}
#infobar{position:absolute;bottom:8px;left:50%;transform:translateX(-50%);
  background:var(--panel);border:1px solid var(--border);border-radius:6px;
  padding:4px 14px;font:11px var(--mono);color:var(--text2);z-index:30;
  display:flex;gap:14px;backdrop-filter:blur(8px);}
#infobar b{color:var(--accent);font-weight:600;}

/* Timeline */
#timeline{position:absolute;bottom:36px;left:50%;transform:translateX(-50%);
  background:var(--panel);border:1px solid var(--border);border-radius:8px;
  padding:6px 14px;z-index:30;display:none;align-items:center;gap:10px;
  backdrop-filter:blur(8px);font:11px var(--mono);}
#timeline.show{display:flex;}
#timeline button{background:none;border:1px solid var(--border);border-radius:4px;
  color:var(--accent);font:12px var(--mono);padding:2px 8px;cursor:pointer;}
#timeline input[type=range]{width:200px;accent-color:var(--accent);}
#timeline .tinfo{color:var(--text2);min-width:100px;}

/* Measure overlay */
#measure-info{position:absolute;top:50px;left:50%;transform:translateX(-50%);
  background:var(--panel);border:1px solid var(--accent);border-radius:8px;
  padding:6px 14px;z-index:30;display:none;font:12px var(--mono);color:var(--accent);
  backdrop-filter:blur(8px);}
#measure-info.show{display:block;}
)CSS";
}

// ═══════════════════════════════════════════════════════════════════════════════
// JAVASCRIPT LOGIC
// ═══════════════════════════════════════════════════════════════════════════════

std::string HtmlVisualizer::generate_js_logic() const {
    std::ostringstream js;

    // ── Config ──
    js << R"JS(
const LC=['#4a90d9','#50c878','#e74c3c','#f39c12','#9b59b6','#1abc9c','#e91e63','#ff9800','#00bcd4','#8bc34a'];
const LN=['M1','M2','M3','M4','M5','M6','M7','M8','M9','M10'];
const DPR=window.devicePixelRatio||1;
const cv=document.getElementById('cv');
const cx=cv.getContext('2d');
const mm=document.getElementById('minimap');
const mc=mm.getContext('2d');
let W,H;
)JS";

    // ── State ──
    js << R"JS(
let S={
  sc:1,ox:0,oy:0,drag:false,dx:0,dy:0,mx:0,my:0,
  selCell:-1,selNet:-1,tool:'pan',
  mstart:null,mend:null,
  layerVis:[],showCells:true,showWires:true,showVias:true,
  showCong:false,showPwr:false,showDrc:true,showPath:true,
  showRows:false,showBlocks:false,showSNets:false,showPins:true,
  congTh:0.1,animFrame:0,animPlay:false,animSpd:1,
};
for(let i=0;i<Math.max(D.layers.length,10);i++)S.layerVis.push(true);
)JS";

    // ── Viridis colormap ──
    js << R"JS(
const VIR=[
  [68,1,84],[72,36,117],[67,62,133],[56,88,140],[45,112,142],
  [31,150,139],[53,183,121],[109,205,89],[180,222,44],[253,231,37]
];
function viridis(t){
  t=Math.max(0,Math.min(1,t));
  const i=t*(VIR.length-1),lo=Math.floor(i),hi=Math.min(lo+1,VIR.length-1),f=i-lo;
  const r=VIR[lo][0]+(VIR[hi][0]-VIR[lo][0])*f;
  const g=VIR[lo][1]+(VIR[hi][1]-VIR[lo][1])*f;
  const b=VIR[lo][2]+(VIR[hi][2]-VIR[lo][2])*f;
  return 'rgba('+Math.round(r)+','+Math.round(g)+','+Math.round(b)+',0.65)';
}
)JS";

    // ── Cell type color ──
    js << R"JS(
const TC={INV:'#3498db',BUF:'#2ecc71',NAND:'#e74c3c',NOR:'#9b59b6',
  AND:'#f39c12',OR:'#1abc9c',XOR:'#e91e63',MUX:'#00bcd4',
  DFF:'#ff5722',LATCH:'#ff9800',AOI:'#795548',OAI:'#607d8b',
  FILL:'#333340',TAP:'#333340',ENDCAP:'#333340',CLKBUF:'#ff4081'};
function typeCol(t){
  const u=t.toUpperCase();
  for(const[k,v]of Object.entries(TC))if(u.includes(k))return v;
  let h=0;for(let i=0;i<t.length;i++)h=(h*31+t.charCodeAt(i))&0xffffff;
  return '#'+(h|0x404040).toString(16).padStart(6,'0');
}
)JS";

    // ── Canvas setup ──
    js << R"JS(
function resize(){
  W=cv.parentElement.clientWidth-(document.getElementById('lpanel').classList.contains('collapsed')?0:200)
    -(document.getElementById('rpanel').classList.contains('show')?240:0);
  H=window.innerHeight-42;
  cv.width=W*DPR;cv.height=H*DPR;
  cv.style.width=W+'px';cv.style.height=H+'px';
  if(S.sc===1)fitAll();
  draw();
}
function fitAll(){
  const dw=D.die.w,dh=D.die.h;
  if(dw<=0||dh<=0)return;
  const pad=40;
  S.sc=Math.min((W-pad*2)/dw,(H-pad*2)/dh);
  S.ox=(W-dw*S.sc)/2;
  S.oy=(H-dh*S.sc)/2;
}
)JS";

    // ── Viewport culling ──
    js << R"JS(
function inView(x,y,w,h){
  const sx=x*S.sc+S.ox,sy=y*S.sc+S.oy;
  return sx+w*S.sc>0&&sx<W&&sy+h*S.sc>0&&sy<H;
}
function worldXY(cx,cy){return{x:(cx-S.ox)/S.sc,y:(cy-S.oy)/S.sc};}
)JS";

    // ── Hit testing ──
    js << R"JS(
function hitCell(wx,wy){
  for(let i=D.cells.length-1;i>=0;i--){
    const c=D.cells[i];
    if(wx>=c[0]&&wx<=c[0]+c[2]&&wy>=c[1]&&wy<=c[1]+c[3])return i;
  }
  return -1;
}
function hitWireNet(wx,wy){
  const tol=Math.max(2/S.sc,0.5);
  for(let i=D.wires.length-1;i>=0;i--){
    const w=D.wires[i];
    if(!S.layerVis[w[0]])continue;
    const dx=w[3]-w[1],dy=w[4]-w[2],len2=dx*dx+dy*dy;
    if(len2<0.001)continue;
    const t=Math.max(0,Math.min(1,((wx-w[1])*dx+(wy-w[2])*dy)/len2));
    const px=w[1]+t*dx,py=w[2]+t*dy;
    const d=Math.sqrt((wx-px)*(wx-px)+(wy-py)*(wy-py));
    if(d<tol+w[5])return w[6];
  }
  return -1;
}
)JS";

    // ── Main draw ──
    js << R"JS(
function draw(){
  cx.save();
  cx.setTransform(DPR,0,0,DPR,0,0);
  cx.fillStyle='#08080d';
  cx.fillRect(0,0,W,H);

  cx.save();
  cx.translate(S.ox,S.oy);
  cx.scale(S.sc,S.sc);

  drawGrid();
  if(S.showRows)drawRows();
  if(S.showBlocks)drawBlocks();
  if(S.showCong&&D.cong)drawHeatmap(D.cong,'viridis');
  if(S.showPwr&&D.pwr)drawHeatmap(D.pwr,'thermal');
  if(S.showSNets)drawSpecialNets();
  if(S.showCells)drawCells();
  if(S.showWires)drawWires();
  if(S.showVias)drawVias();
  if(S.showPins)drawPins();
  if(S.showDrc&&D.drc)drawDrc();
  if(S.showPath&&D.tpaths)drawTimingPaths();
  drawSelection();
  drawMeasure();

  cx.restore();

  drawMinimap();
  updateCoords();

  cx.restore();
}
)JS";

    // ── Grid ──
    js << R"JS(
function drawGrid(){
  cx.strokeStyle='#00d4ff';cx.lineWidth=2/S.sc;
  cx.setLineDash([6/S.sc,4/S.sc]);
  cx.strokeRect(0,0,D.die.w,D.die.h);
  cx.setLineDash([]);
  if(S.sc>8){
    cx.strokeStyle='rgba(40,40,65,0.3)';cx.lineWidth=0.5/S.sc;
    const sp=10;
    for(let x=0;x<=D.die.w;x+=sp){cx.beginPath();cx.moveTo(x,0);cx.lineTo(x,D.die.h);cx.stroke();}
    for(let y=0;y<=D.die.h;y+=sp){cx.beginPath();cx.moveTo(0,y);cx.lineTo(D.die.w,y);cx.stroke();}
  }
}
)JS";

    // ── Placement rows ──
    js << R"JS(
function drawRows(){
  if(!D.rows||!D.rows.length)return;
  for(let i=0;i<D.rows.length;i++){
    const r=D.rows[i];
    const rw=r[2]*r[3],rh=r[4]*r[5]||10;
    if(!inView(r[0],r[1],rw,rh))continue;
    cx.fillStyle=i%2===0?'rgba(30,30,55,0.18)':'rgba(40,40,70,0.12)';
    cx.fillRect(r[0],r[1],rw,rh>0?rh:10);
  }
}
)JS";

    // ── Blockages ──
    js << R"JS(
function drawBlocks(){
  if(!D.blocks||!D.blocks.length)return;
  for(const b of D.blocks){
    const x0=b[1],y0=b[2],x1=b[3],y1=b[4];
    if(!inView(x0,y0,x1-x0,y1-y0))continue;
    cx.fillStyle='rgba(239,68,68,0.08)';
    cx.strokeStyle='rgba(239,68,68,0.25)';
    cx.lineWidth=1/S.sc;
    cx.fillRect(x0,y0,x1-x0,y1-y0);
    cx.setLineDash([3/S.sc,2/S.sc]);
    cx.strokeRect(x0,y0,x1-x0,y1-y0);
    cx.setLineDash([]);
  }
}
)JS";

    // ── Heatmap ──
    js << R"JS(
function drawHeatmap(map,mode){
  const gy=map.length,gx=map[0].length;
  const cw=D.die.w/gx,ch=D.die.h/gy;
  for(let y=0;y<gy;y++){
    for(let x=0;x<gx;x++){
      const v=map[y][x];
      if(v<=S.congTh)continue;
      if(!inView(x*cw,y*ch,cw,ch))continue;
      const t=(v-S.congTh)/(1.0-S.congTh);
      cx.fillStyle=mode==='viridis'?viridis(t):
        'rgba('+Math.round(255*t)+','+Math.round(80*(1-t))+','+Math.round(40*(1-t))+',0.55)';
      cx.fillRect(x*cw,y*ch,cw,ch);
    }
  }
}
)JS";

    // ── Special nets ──
    js << R"JS(
function drawSpecialNets(){
  if(!D.snets||!D.snets.length)return;
  for(const sn of D.snets){
    const isPwr=sn.use==='POWER';
    cx.strokeStyle=isPwr?'rgba(239,68,68,0.35)':'rgba(100,100,200,0.35)';
    cx.lineWidth=2/S.sc;
    for(const w of sn.w){
      cx.lineWidth=Math.max(w[0],1/S.sc);
      cx.beginPath();cx.moveTo(w[1],w[2]);cx.lineTo(w[3],w[4]);cx.stroke();
    }
  }
}
)JS";

    // ── Cells ──
    js << R"JS(
function drawCells(){
  const showLabel=S.sc>12;
  const showType=S.sc>20;
  for(let i=0;i<D.cells.length;i++){
    const c=D.cells[i];
    if(!inView(c[0],c[1],c[2],c[3]))continue;
    const isMacro=c[7]===1;
    const col=typeCol(c[4]);
    cx.fillStyle=isMacro?'rgba(124,58,237,0.18)':hexAlpha(col,0.12);
    cx.fillRect(c[0],c[1],c[2],c[3]);
    if(S.sc>0.5){
      cx.strokeStyle=isMacro?'#7c3aed':col;
      cx.lineWidth=(isMacro?1.5:0.8)/S.sc;
      cx.strokeRect(c[0],c[1],c[2],c[3]);
    }
    if(showLabel){
      cx.fillStyle='#e0e0f0';
      cx.font=600+' '+Math.max(6/S.sc,0.8)+'px JetBrains Mono';
      cx.textAlign='center';cx.textBaseline='middle';
      cx.fillText(c[5],c[0]+c[2]/2,c[1]+c[3]/2);
    }
    if(showType&&c[3]*S.sc>18){
      cx.fillStyle='rgba(120,120,160,0.7)';
      cx.font=Math.max(4/S.sc,0.5)+'px JetBrains Mono';
      cx.fillText(c[4],c[0]+c[2]/2,c[1]+c[3]/2+8/S.sc);
    }
  }
}
function hexAlpha(hex,a){
  const r=parseInt(hex.slice(1,3),16),g=parseInt(hex.slice(3,5),16),b=parseInt(hex.slice(5,7),16);
  return 'rgba('+r+','+g+','+b+','+a+')';
}
)JS";

    // ── Wires ──
    js << R"JS(
function drawWires(){
  cx.lineCap='round';
  for(const w of D.wires){
    const l=w[0];
    if(!S.layerVis[l])continue;
    if(!inView(Math.min(w[1],w[3]),Math.min(w[2],w[4]),
               Math.abs(w[3]-w[1])+1,Math.abs(w[4]-w[2])+1))continue;
    const col=LC[l%LC.length];
    cx.strokeStyle=col;
    cx.lineWidth=Math.max(w[5],1.2/S.sc);
    cx.globalAlpha=0.8;
    cx.shadowColor=col;cx.shadowBlur=3/S.sc;
    cx.beginPath();cx.moveTo(w[1],w[2]);cx.lineTo(w[3],w[4]);cx.stroke();
  }
  cx.shadowBlur=0;cx.globalAlpha=1;
}
)JS";

    // ── Vias ──
    js << R"JS(
function drawVias(){
  if(!D.vias||!D.vias.length)return;
  const sz=Math.max(1.5/S.sc,0.3);
  for(const v of D.vias){
    if(!inView(v[0]-sz,v[1]-sz,sz*2,sz*2))continue;
    const lo=v[2],hi=v[3];
    const col1=LC[lo%LC.length],col2=LC[hi%LC.length];
    if(!S.layerVis[lo]&&!S.layerVis[hi])continue;
    cx.fillStyle='#c0c0c0';
    cx.beginPath();
    cx.moveTo(v[0],v[1]-sz);cx.lineTo(v[0]+sz,v[1]);
    cx.lineTo(v[0],v[1]+sz);cx.lineTo(v[0]-sz,v[1]);
    cx.closePath();cx.fill();
    cx.strokeStyle=col2;cx.lineWidth=0.5/S.sc;cx.stroke();
  }
}
)JS";

    // ── IO Pins ──
    js << R"JS(
function drawPins(){
  if(!D.pins||!D.pins.length)return;
  const sz=Math.max(3/S.sc,0.8);
  for(const p of D.pins){
    if(!inView(p.x-sz,p.y-sz,sz*2,sz*2))continue;
    const isIn=p.d==='INPUT';
    cx.fillStyle=isIn?'rgba(34,211,238,0.3)':'rgba(248,113,113,0.3)';
    cx.strokeStyle=isIn?'#22d3ee':'#f87171';
    cx.lineWidth=1/S.sc;
    cx.beginPath();
    if(isIn){cx.moveTo(p.x,p.y-sz);cx.lineTo(p.x+sz,p.y);cx.lineTo(p.x,p.y+sz);cx.closePath();}
    else{cx.moveTo(p.x,p.y-sz);cx.lineTo(p.x-sz,p.y);cx.lineTo(p.x,p.y+sz);cx.closePath();}
    cx.fill();cx.stroke();
    if(S.sc>8){
      cx.fillStyle=isIn?'#22d3ee':'#f87171';
      cx.font=Math.max(5/S.sc,0.6)+'px JetBrains Mono';
      cx.textAlign=isIn?'left':'right';cx.textBaseline='middle';
      cx.fillText(p.n,p.x+(isIn?sz+2/S.sc:-sz-2/S.sc),p.y);
    }
  }
}
)JS";

    // ── DRC markers ──
    js << R"JS(
function drawDrc(){
  const sz=Math.max(4/S.sc,1);
  for(const d of D.drc){
    if(!inView(d.x-sz,d.y-sz,sz*2,sz*2))continue;
    const col=d.s>=1?'#ef4444':'#f59e0b';
    cx.strokeStyle=col;cx.lineWidth=1.5/S.sc;
    cx.beginPath();cx.moveTo(d.x-sz,d.y-sz);cx.lineTo(d.x+sz,d.y+sz);cx.stroke();
    cx.beginPath();cx.moveTo(d.x+sz,d.y-sz);cx.lineTo(d.x-sz,d.y+sz);cx.stroke();
    cx.beginPath();cx.arc(d.x,d.y,sz*1.3,0,Math.PI*2);
    cx.strokeStyle=hexAlpha(col,0.4);cx.stroke();
    if(S.sc>6){
      cx.fillStyle=col;
      cx.font='600 '+Math.max(4/S.sc,0.6)+'px JetBrains Mono';
      cx.textAlign='left';cx.textBaseline='bottom';
      cx.fillText(d.t,d.x+sz*1.5,d.y-sz*0.5);
    }
  }
}
)JS";

    // ── Timing paths ──
    js << R"JS(
function drawTimingPaths(){
  if(!D.tpaths||!D.tpaths.length)return;
  const tp=D.tpaths[0];
  cx.strokeStyle='rgba(239,68,68,0.7)';cx.lineWidth=2.5/S.sc;
  cx.setLineDash([4/S.sc,2/S.sc]);
  for(let i=0;i<tp.c.length-1;i++){
    const a=D.cells[tp.c[i]],b=D.cells[tp.c[i+1]];
    if(!a||!b)continue;
    cx.beginPath();
    cx.moveTo(a[0]+a[2]/2,a[1]+a[3]/2);
    cx.lineTo(b[0]+b[2]/2,b[1]+b[3]/2);
    cx.stroke();
  }
  cx.setLineDash([]);
  for(const ci of tp.c){
    const c=D.cells[ci];if(!c)continue;
    cx.strokeStyle='#ef4444';cx.lineWidth=2/S.sc;
    cx.strokeRect(c[0],c[1],c[2],c[3]);
    cx.fillStyle='rgba(239,68,68,0.15)';
    cx.fillRect(c[0],c[1],c[2],c[3]);
  }
}
)JS";

    // ── Selection highlight ──
    js << R"JS(
function drawSelection(){
  if(S.selCell>=0){
    const c=D.cells[S.selCell];
    cx.strokeStyle='#fbbf24';cx.lineWidth=2/S.sc;
    cx.strokeRect(c[0]-1/S.sc,c[1]-1/S.sc,c[2]+2/S.sc,c[3]+2/S.sc);
    cx.fillStyle='rgba(251,191,36,0.12)';
    cx.fillRect(c[0],c[1],c[2],c[3]);
  }
  if(S.selNet>=0){
    cx.strokeStyle='#00d4ff';cx.lineWidth=2/S.sc;
    cx.globalAlpha=0.9;
    cx.shadowColor='#00d4ff';cx.shadowBlur=6/S.sc;
    for(const w of D.wires){
      if(w[6]!==S.selNet)continue;
      cx.beginPath();cx.moveTo(w[1],w[2]);cx.lineTo(w[3],w[4]);cx.stroke();
    }
    cx.shadowBlur=0;cx.globalAlpha=1;
  }
}
)JS";

    // ── Measurement ──
    js << R"JS(
function drawMeasure(){
  if(!S.mstart)return;
  const a=S.mstart,b=S.mend||worldXY(S.mx,S.my);
  cx.strokeStyle='#fbbf24';cx.lineWidth=1.5/S.sc;
  cx.setLineDash([3/S.sc,2/S.sc]);
  cx.beginPath();cx.moveTo(a.x,a.y);cx.lineTo(b.x,b.y);cx.stroke();
  cx.setLineDash([]);
  const sz=2/S.sc;
  cx.fillStyle='#fbbf24';
  cx.fillRect(a.x-sz/2,a.y-sz/2,sz,sz);
  cx.fillRect(b.x-sz/2,b.y-sz/2,sz,sz);
  const manh=Math.abs(b.x-a.x)+Math.abs(b.y-a.y);
  const eucl=Math.sqrt((b.x-a.x)**2+(b.y-a.y)**2);
  const el=document.getElementById('measure-info');
  el.classList.add('show');
  el.innerHTML='Manhattan: <b>'+manh.toFixed(2)+'</b> \u00b5m &nbsp; Euclidean: <b>'+eucl.toFixed(2)+'</b> \u00b5m'+
    ' &nbsp; \u0394X: '+Math.abs(b.x-a.x).toFixed(2)+' \u0394Y: '+Math.abs(b.y-a.y).toFixed(2);
}
)JS";

    // ── Minimap ──
    js << R"JS(
function drawMinimap(){
  const mw=180,mh=130;
  mm.width=mw*DPR;mm.height=mh*DPR;
  mm.style.width=mw+'px';mm.style.height=mh+'px';
  mc.setTransform(DPR,0,0,DPR,0,0);
  mc.fillStyle='#0a0a14';mc.fillRect(0,0,mw,mh);
  if(D.die.w<=0||D.die.h<=0)return;
  const pad=6;
  const msc=Math.min((mw-pad*2)/D.die.w,(mh-pad*2)/D.die.h);
  const mox=(mw-D.die.w*msc)/2,moy=(mh-D.die.h*msc)/2;
  mc.strokeStyle='rgba(0,212,255,0.3)';mc.lineWidth=1;
  mc.strokeRect(mox,moy,D.die.w*msc,D.die.h*msc);
  mc.fillStyle='rgba(16,185,129,0.4)';
  for(const c of D.cells){
    mc.fillRect(mox+c[0]*msc,moy+c[1]*msc,Math.max(c[2]*msc,0.5),Math.max(c[3]*msc,0.5));
  }
  mc.globalAlpha=0.5;
  for(const w of D.wires){
    mc.strokeStyle=LC[w[0]%LC.length];mc.lineWidth=0.5;
    mc.beginPath();mc.moveTo(mox+w[1]*msc,moy+w[2]*msc);mc.lineTo(mox+w[3]*msc,moy+w[4]*msc);mc.stroke();
  }
  mc.globalAlpha=1;
  const vx0=(-S.ox/S.sc),vy0=(-S.oy/S.sc);
  const vx1=vx0+W/S.sc,vy1=vy0+H/S.sc;
  mc.strokeStyle='#fbbf24';mc.lineWidth=1.5;
  mc.strokeRect(mox+vx0*msc,moy+vy0*msc,(vx1-vx0)*msc,(vy1-vy0)*msc);
}
)JS";

    // ── Coordinates ──
    js << R"JS(
function updateCoords(){
  const w=worldXY(S.mx,S.my);
  document.getElementById('coords').textContent=w.x.toFixed(2)+', '+w.y.toFixed(2)+' \u00b5m  ['+S.sc.toFixed(1)+'x]';
}
)JS";

    // ── Inspector ──
    js << R"JS(
function updateInspector(){
  const rp=document.getElementById('rpanel');
  const ic=document.getElementById('insp-content');
  if(S.selCell<0&&S.selNet<0){rp.classList.remove('show');resize();return;}
  rp.classList.add('show');
  let h='';
  if(S.selCell>=0){
    const c=D.cells[S.selCell];
    h+='<div class="isec">Cell</div>';
    h+=irow('Name',c[5])+irow('Type',c[4])+irow('Size',c[2].toFixed(2)+' x '+c[3].toFixed(2));
    h+=irow('Position','('+c[0].toFixed(2)+', '+c[1].toFixed(2)+')');
    h+=irow('Orient',['N','S','W','E','FN','FS','FW','FE'][c[6]]||c[6]);
    h+=irow('Macro',c[7]?'Yes':'No');
    const nets=[];
    for(let i=0;i<D.nets.length;i++){
      if(D.nets[i].c.includes(S.selCell))nets.push({idx:i,name:D.nets[i].n});
    }
    if(nets.length){
      h+='<div class="isec">Connected Nets ('+nets.length+')</div>';
      for(const n of nets.slice(0,15)){
        h+='<div class="sr" onclick="selectNet('+n.idx+')">'+n.name+'</div>';
      }
      if(nets.length>15)h+='<div style="color:var(--text2);padding:4px">...+'+(nets.length-15)+' more</div>';
    }
  }
  if(S.selNet>=0&&D.nets[S.selNet]){
    const n=D.nets[S.selNet];
    h+='<div class="isec">Net</div>';
    h+=irow('Name',n.n)+irow('Cells',''+n.c.length);
    const wc=D.wires.filter(w=>w[6]===S.selNet).length;
    h+=irow('Wire Segs',''+wc);
  }
  ic.innerHTML=h;
  resize();
}
function irow(k,v){return '<div class="irow"><span class="ik">'+k+'</span><span class="iv" title="'+v+'">'+v+'</span></div>';}
function selectNet(idx){S.selNet=idx;updateInspector();draw();}
)JS";

    // ── Events ──
    js << R"JS(
cv.addEventListener('mousedown',e=>{
  const rect=cv.getBoundingClientRect();
  const cx_=e.clientX-rect.left,cy_=e.clientY-rect.top;
  if(S.tool==='measure'){
    const w=worldXY(cx_,cy_);
    if(!S.mstart){S.mstart=w;S.mend=null;}
    else{S.mend=w;S.tool='pan';document.querySelectorAll('.tbtn').forEach(b=>b.classList.remove('on'));
      document.getElementById('t-pan').classList.add('on');}
    draw();return;
  }
  S.drag=true;S.dx=cx_-S.ox;S.dy=cy_-S.oy;
});
cv.addEventListener('mousemove',e=>{
  const rect=cv.getBoundingClientRect();
  S.mx=e.clientX-rect.left;S.my=e.clientY-rect.top;
  if(S.drag){S.ox=S.mx-S.dx;S.oy=S.my-S.dy;draw();return;}
  if(S.tool==='measure'&&S.mstart&&!S.mend)draw();
  updateCoords();
});
window.addEventListener('mouseup',()=>{S.drag=false;});
cv.addEventListener('wheel',e=>{
  e.preventDefault();
  const rect=cv.getBoundingClientRect();
  const mx=e.clientX-rect.left,my=e.clientY-rect.top;
  const z=e.deltaY<0?1.15:0.87;
  const ns=S.sc*z;
  S.ox=mx-(mx-S.ox)*(ns/S.sc);
  S.oy=my-(my-S.oy)*(ns/S.sc);
  S.sc=ns;
  draw();
},{passive:false});
cv.addEventListener('click',e=>{
  if(S.tool==='measure')return;
  const rect=cv.getBoundingClientRect();
  const cx_=e.clientX-rect.left,cy_=e.clientY-rect.top;
  const w=worldXY(cx_,cy_);
  const ci=hitCell(w.x,w.y);
  const ni=ci<0?hitWireNet(w.x,w.y):-1;
  S.selCell=ci;S.selNet=ni;
  updateInspector();draw();
});
cv.addEventListener('dblclick',e=>{
  if(S.selCell>=0){
    const c=D.cells[S.selCell];
    const pad=Math.max(c[2],c[3])*3;
    S.sc=Math.min(W,H)/(pad*2);
    S.ox=W/2-(c[0]+c[2]/2)*S.sc;
    S.oy=H/2-(c[1]+c[3]/2)*S.sc;
    draw();
  }
});
mm.addEventListener('click',e=>{
  const rect=mm.getBoundingClientRect();
  const mx=e.clientX-rect.left,my=e.clientY-rect.top;
  const mw=180,mh=130,pad=6;
  const msc=Math.min((mw-pad*2)/D.die.w,(mh-pad*2)/D.die.h);
  const mox=(mw-D.die.w*msc)/2,moy=(mh-D.die.h*msc)/2;
  const wx=(mx-mox)/msc,wy=(my-moy)/msc;
  S.ox=W/2-wx*S.sc;S.oy=H/2-wy*S.sc;
  draw();
});
)JS";

    // ── Keyboard shortcuts ──
    js << R"JS(
document.addEventListener('keydown',e=>{
  if(e.target.tagName==='INPUT')return;
  switch(e.key){
    case 'f':case 'F':fitAll();draw();break;
    case 'Escape':
      S.selCell=-1;S.selNet=-1;S.mstart=null;S.mend=null;S.tool='pan';
      document.querySelectorAll('.tbtn').forEach(b=>b.classList.remove('on'));
      document.getElementById('t-pan').classList.add('on');
      document.getElementById('measure-info').classList.remove('show');
      updateInspector();draw();break;
    case 'r':case 'R':
      S.tool='measure';S.mstart=null;S.mend=null;
      document.querySelectorAll('.tbtn').forEach(b=>b.classList.remove('on'));
      document.getElementById('t-ruler').classList.add('on');
      break;
    case '/':e.preventDefault();document.getElementById('search').focus();break;
    case 'c':S.showCells=!S.showCells;draw();break;
    case 'w':S.showWires=!S.showWires;draw();break;
    case 'v':S.showVias=!S.showVias;draw();break;
  }
  if(e.key>='1'&&e.key<='9'){
    const l=parseInt(e.key)-1;
    if(l<S.layerVis.length){S.layerVis[l]=!S.layerVis[l];updateLayerUI();draw();}
  }
});
)JS";

    // ── Search ──
    js << R"JS(
const searchInput=document.getElementById('search');
const searchResults=document.getElementById('search-results');
searchInput.addEventListener('input',e=>{
  const q=e.target.value.toLowerCase().trim();
  if(q.length<1){searchResults.style.display='none';return;}
  let res=[];
  for(let i=0;i<D.cells.length&&res.length<20;i++){
    if(D.cells[i][5].toLowerCase().includes(q)||D.cells[i][4].toLowerCase().includes(q))
      res.push({type:'cell',idx:i,name:D.cells[i][5]+' ('+D.cells[i][4]+')'});
  }
  for(let i=0;i<D.nets.length&&res.length<20;i++){
    if(D.nets[i].n.toLowerCase().includes(q))
      res.push({type:'net',idx:i,name:D.nets[i].n});
  }
  if(!res.length){searchResults.style.display='none';return;}
  searchResults.style.display='block';
  searchResults.innerHTML=res.map(r=>
    '<div class="sr" onclick="searchSelect(\''+r.type+'\','+r.idx+')">'+
    '<span style="color:var(--accent);font-size:9px">'+r.type.toUpperCase()+'</span> '+r.name+'</div>'
  ).join('');
});
searchInput.addEventListener('blur',()=>setTimeout(()=>searchResults.style.display='none',200));
function searchSelect(type,idx){
  searchResults.style.display='none';searchInput.value='';
  if(type==='cell'){
    S.selCell=idx;S.selNet=-1;
    const c=D.cells[idx];
    S.sc=Math.min(W,H)/Math.max(c[2],c[3])/8;
    S.ox=W/2-(c[0]+c[2]/2)*S.sc;S.oy=H/2-(c[1]+c[3]/2)*S.sc;
  }else{
    S.selNet=idx;S.selCell=-1;
  }
  updateInspector();draw();
}
)JS";

    // ── Placement animation ──
    js << R"JS(
function animStep(){
  if(!S.animPlay||!D.snaps||!D.snaps.length)return;
  S.animFrame+=S.animSpd;
  if(S.animFrame>=D.snaps.length){S.animFrame=D.snaps.length-1;S.animPlay=false;}
  const snap=D.snaps[Math.floor(S.animFrame)];
  for(let i=0;i<D.cells.length&&i<snap.p.length;i++){
    D.cells[i][0]=snap.p[i][0];D.cells[i][1]=snap.p[i][1];
  }
  document.getElementById('anim-slider').value=S.animFrame;
  document.getElementById('anim-info').textContent='Iter '+snap.it+' | Cost: '+snap.cost.toFixed(1);
  draw();
  if(S.animPlay)requestAnimationFrame(animStep);
}
)JS";

    // ── Layer UI sync ──
    js << R"JS(
function updateLayerUI(){
  document.querySelectorAll('[data-ml]').forEach(el=>{
    const l=parseInt(el.dataset.ml);
    el.checked=S.layerVis[l];
    el.parentElement.classList.toggle('off',!S.layerVis[l]);
  });
}
)JS";

    // ── Toolbar handlers ──
    js << R"JS(
document.getElementById('t-pan').addEventListener('click',function(){
  S.tool='pan';document.querySelectorAll('.tbtn').forEach(b=>b.classList.remove('on'));this.classList.add('on');
});
document.getElementById('t-ruler').addEventListener('click',function(){
  S.tool='measure';S.mstart=null;S.mend=null;
  document.querySelectorAll('.tbtn').forEach(b=>b.classList.remove('on'));this.classList.add('on');
});
document.getElementById('t-fit').addEventListener('click',()=>{fitAll();draw();});
document.getElementById('t-layers').addEventListener('click',()=>{
  document.getElementById('lpanel').classList.toggle('collapsed');resize();
});
)JS";

    // ── Layer/overlay checkbox handlers ──
    js << R"JS(
document.querySelectorAll('[data-ov]').forEach(el=>{
  el.addEventListener('change',e=>{
    const k=e.target.dataset.ov;
    S[k]=e.target.checked;
    e.target.parentElement.classList.toggle('off',!e.target.checked);
    draw();
  });
});
document.querySelectorAll('[data-ml]').forEach(el=>{
  el.addEventListener('change',e=>{
    const l=parseInt(e.target.dataset.ml);
    S.layerVis[l]=e.target.checked;
    e.target.parentElement.classList.toggle('off',!e.target.checked);
    draw();
  });
});
const congSlider=document.getElementById('cong-slider');
if(congSlider){
  congSlider.addEventListener('input',e=>{
    S.congTh=parseInt(e.target.value)/100;
    document.getElementById('cong-val').textContent=S.congTh.toFixed(2);
    draw();
  });
}
)JS";

    // ── Timeline ──
    js << R"JS(
if(D.snaps&&D.snaps.length){
  document.getElementById('timeline').classList.add('show');
  const sl=document.getElementById('anim-slider');
  sl.max=D.snaps.length-1;
  sl.addEventListener('input',e=>{
    S.animFrame=parseInt(e.target.value);S.animPlay=false;
    const snap=D.snaps[S.animFrame];
    for(let i=0;i<D.cells.length&&i<snap.p.length;i++){
      D.cells[i][0]=snap.p[i][0];D.cells[i][1]=snap.p[i][1];
    }
    document.getElementById('anim-info').textContent='Iter '+snap.it+' | Cost: '+snap.cost.toFixed(1);
    draw();
  });
  document.getElementById('anim-play').addEventListener('click',()=>{
    S.animPlay=!S.animPlay;
    document.getElementById('anim-play').textContent=S.animPlay?'\u23f8':'\u25b6';
    if(S.animPlay){if(S.animFrame>=D.snaps.length-1)S.animFrame=0;animStep();}
  });
}
)JS";

    // ── SSE live connection (if served from studio) ──
    js << R"JS(
if(location.protocol!=='file:'){
  try{
    const es=new EventSource('/api/events');
    es.onmessage=e=>{
      const u=JSON.parse(e.data);
      if(u.type==='cell_move'&&u.id<D.cells.length){
        D.cells[u.id][0]=u.x;D.cells[u.id][1]=u.y;requestAnimationFrame(draw);
      }else if(u.type==='wire_add'){
        D.wires.push([u.l,u.x1,u.y1,u.x2,u.y2,u.w,u.net]);requestAnimationFrame(draw);
      }else if(u.type==='full_update'){
        location.reload();
      }
    };
    es.onerror=()=>{es.close();};
  }catch(e){}
}
)JS";

    // ── Pre-highlight ──
    js << R"JS(
if(D.hlNet){
  for(let i=0;i<D.nets.length;i++){
    if(D.nets[i].n===D.hlNet){S.selNet=i;break;}
  }
}
)JS";

    // ── Init ──
    js << R"JS(
resize();
window.addEventListener('resize',resize);
)JS";

    return js.str();
}

// ═══════════════════════════════════════════════════════════════════════════════
// HTML ASSEMBLY
// ═══════════════════════════════════════════════════════════════════════════════

static const char* layer_hex(int i) {
    static const char* colors[] = {
        "#4a90d9","#50c878","#e74c3c","#f39c12","#9b59b6",
        "#1abc9c","#e91e63","#ff9800","#00bcd4","#8bc34a"
    };
    return colors[i % 10];
}

std::string HtmlVisualizer::generate_html() const {
    std::ostringstream ss;
    ss << "<!DOCTYPE html>\n<html lang=\"en\">\n<head>\n<meta charset=\"UTF-8\">\n";
    ss << "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1.0\">\n";
    ss << "<title>SiliconForge Layout Viewer</title>\n";
    ss << "<style>\n" << generate_css() << "\n</style>\n</head>\n<body>\n";

    ss << "<div id=\"app\">\n";

    // Top bar
    ss << "<div id=\"topbar\">\n";
    ss << "  <div class=\"logo\">SiliconForge</div>\n";
    ss << "  <div class=\"sep\"></div>\n";
    ss << "  <input id=\"search\" placeholder=\"Search cell / net...  (/)\" autocomplete=\"off\">\n";
    ss << "  <div id=\"search-results\"></div>\n";
    ss << "  <div class=\"sep\"></div>\n";
    ss << "  <button class=\"tbtn on\" id=\"t-pan\">Pan</button>\n";
    ss << "  <button class=\"tbtn\" id=\"t-ruler\">Ruler</button>\n";
    ss << "  <button class=\"tbtn\" id=\"t-fit\">Fit (F)</button>\n";
    ss << "  <button class=\"tbtn\" id=\"t-layers\">Layers</button>\n";
    ss << "  <div style=\"flex:1\"></div>\n";
    ss << "  <span style=\"font:11px var(--mono);color:var(--text2)\">"
       << pd_.cells.size() << " cells &middot; " << pd_.wires.size() << " wires &middot; "
       << pd_.vias.size() << " vias</span>\n";
    ss << "</div>\n";

    // Main area
    ss << "<div id=\"main\">\n";

    // Left panel
    ss << "<div id=\"lpanel\">\n";
    ss << "  <div class=\"ptitle\">Display</div>\n";

    auto ov_check = [&](const char* key, const char* label, bool def) {
        ss << "  <label class=\"lrow" << (def ? "" : " off") << "\"><input type=\"checkbox\" data-ov=\"" << key << "\""
           << (def ? " checked" : "") << "> <span>" << label << "</span></label>\n";
    };
    ov_check("showCells", "Cells", true);
    ov_check("showWires", "Wires", true);
    ov_check("showVias", "Vias", true);
    ov_check("showPins", "IO Pins", true);
    ov_check("showDrc", "DRC Markers", true);
    ov_check("showPath", "Critical Path", true);
    ov_check("showRows", "Placement Rows", false);
    ov_check("showBlocks", "Blockages", false);
    ov_check("showSNets", "Power/Ground", false);

    if (!congestion_map_.empty()) {
        ov_check("showCong", "Congestion", false);
        ss << "  <div class=\"slider-row\"><input type=\"range\" id=\"cong-slider\" min=\"0\" max=\"100\" value=\"10\">"
           << "<span class=\"sv\" id=\"cong-val\">0.10</span></div>\n";
    }
    if (!power_map_.empty()) {
        ov_check("showPwr", "Power Map", false);
    }

    // Per-layer toggles
    ss << "  <div class=\"ptitle\">Metal Layers</div>\n";
    size_t nl = pd_.layers.size();
    if (nl == 0) nl = 6; // default if no layers defined
    for (size_t i = 0; i < nl; ++i) {
        std::string name = i < pd_.layers.size() ? pd_.layers[i].name : ("M" + std::to_string(i + 1));
        ss << "  <label class=\"lrow\"><input type=\"checkbox\" data-ml=\"" << i << "\" checked>"
           << "<span class=\"sw\" style=\"background:" << layer_hex(i) << "\"></span>"
           << "<span>" << name << "</span></label>\n";
    }

    ss << "</div>\n"; // end lpanel

    // Canvas
    ss << "<canvas id=\"cv\"></canvas>\n";

    // Right panel (inspector)
    ss << "<div id=\"rpanel\"><div class=\"ptitle\">Inspector</div><div id=\"insp-content\"></div></div>\n";

    // Overlays
    ss << "<div id=\"coords\">0, 0 \xC2\xB5m</div>\n";
    ss << "<canvas id=\"minimap\" width=\"180\" height=\"130\"></canvas>\n";

    ss << "<div id=\"infobar\">\n";
    ss << "  <span>Die: <b>" << pd_.die_area.width() << " x " << pd_.die_area.height() << " \xC2\xB5m</b></span>\n";
    ss << "  <span>Util: <b>" << (pd_.utilization() * 100) << "%</b></span>\n";
    ss << "  <span>WL: <b>" << pd_.total_wirelength() << " \xC2\xB5m</b></span>\n";
    ss << "</div>\n";

    ss << "<div id=\"measure-info\"></div>\n";

    // Timeline (animation)
    ss << "<div id=\"timeline\">\n";
    ss << "  <button id=\"anim-play\">\xE2\x96\xB6</button>\n";
    ss << "  <input type=\"range\" id=\"anim-slider\" min=\"0\" max=\"100\" value=\"0\">\n";
    ss << "  <span class=\"tinfo\" id=\"anim-info\">Frame 0</span>\n";
    ss << "</div>\n";

    ss << "</div>\n"; // end main
    ss << "</div>\n"; // end app

    // Script
    ss << "<script>\n";
    ss << generate_json_data();
    ss << generate_js_logic();
    ss << "</script>\n";

    ss << "</body>\n</html>";
    return ss.str();
}

bool HtmlVisualizer::export_to_file(const std::string& filename) const {
    std::ofstream out(filename);
    if (!out) return false;
    out << generate_html();
    return true;
}

} // namespace sf
