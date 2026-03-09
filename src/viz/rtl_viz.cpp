// SiliconForge — RTL Circuit Graph Visualizer
#include "viz/rtl_viz.hpp"
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <algorithm>

namespace sf {

std::string RtlVisualizer::generate_json_data() const {
    std::ostringstream ss;
    ss << "const designData = {\n";
    
    // Gather nodes
    ss << "  nodes: [\n";
    std::unordered_map<int, int> nl_id_to_idx; // Map Netlist GateId/NetId to JS array index
    int idx = 0;
    
    // Create nodes for PI/PO (nets marked input/output)
    for (auto& n : nl_.nets()) {
        bool is_input = std::find(nl_.primary_inputs().begin(), nl_.primary_inputs().end(), n.id) != nl_.primary_inputs().end();
        if (is_input) {
            ss << "    { id: " << idx << ", type: 'IN', label: '" << n.name << "', r: 14 }";
            nl_id_to_idx[nl_.gates().size() + n.id] = idx++;
            if (idx > 1 || nl_.gates().size() > 0) ss << ",\n";
        }
    }
    
    for (size_t g = 0; g < nl_.gates().size(); ++g) {
        auto& gate = nl_.gates()[g];
        std::string type_str;
        switch(gate.type) {
            case GateType::AND: type_str = "AND"; break;
            case GateType::OR: type_str = "OR"; break;
            case GateType::NOT: type_str = "NOT"; break;
            case GateType::XOR: type_str = "XOR"; break;
            case GateType::BUF: type_str = "BUF"; break;
            case GateType::MUX: type_str = "MUX"; break;
            case GateType::DFF: type_str = "DFF"; break;
            default: type_str = "GATE"; break;
        }
        
        ss << "    { id: " << idx << ", type: '" << type_str << "', label: '" << gate.name << "', r: 18 }";
        nl_id_to_idx[g] = idx++;
        if (g < nl_.gates().size() - 1 || nl_.nets().size() > 0) ss << ",\n";
    }
    
    for (auto& n : nl_.nets()) {
        bool is_output = std::find(nl_.primary_outputs().begin(), nl_.primary_outputs().end(), n.id) != nl_.primary_outputs().end();
        if (is_output) {
            ss << "    { id: " << idx << ", type: 'OUT', label: '" << n.name << "', r: 14 }";
            // Map the net's driver to this output node later via an edge
            nl_id_to_idx[nl_.gates().size() + n.id] = idx++;
            ss << ",\n";
        }
    }
    // Remove trailing comma (simplistic for now)
    std::string nodes_str = ss.str();
    if (nodes_str.length() > 2 && nodes_str[nodes_str.length()-2] == ',') {
        nodes_str.erase(nodes_str.length()-2, 1);
    }
    ss.str(""); ss << nodes_str;
    ss << "  ],\n";

    // Gather edges
    ss << "  edges: [\n";
    bool first_edge = true;
    for (size_t g = 0; g < nl_.gates().size(); ++g) {
        auto& gate = nl_.gates()[g];
        int to_idx = nl_id_to_idx[g];
        
        for (auto in_net : gate.inputs) {
            // Find driver of in_net
            int from_idx = -1;
            bool is_pi = std::find(nl_.primary_inputs().begin(), nl_.primary_inputs().end(), in_net) != nl_.primary_inputs().end();
            if (is_pi) {
                from_idx = nl_id_to_idx[nl_.gates().size() + in_net];
            } else {
                for (size_t src = 0; src < nl_.gates().size(); ++src) {
                    if (nl_.gates()[src].output == in_net) {
                        from_idx = nl_id_to_idx[src];
                        break;
                    }
                }
            }
            if (from_idx != -1) {
                if (!first_edge) ss << ",\n";
                ss << "    { from: " << from_idx << ", to: " << to_idx << " }";
                first_edge = false;
            }
        }
        
        // Also map clock if DFF
        if (gate.type == GateType::DFF && gate.clk != -1) {
             int from_idx = -1;
             bool is_pi = std::find(nl_.primary_inputs().begin(), nl_.primary_inputs().end(), gate.clk) != nl_.primary_inputs().end();
             if (is_pi) {
                 from_idx = nl_id_to_idx[nl_.gates().size() + gate.clk];
             } else {
                 for (size_t src = 0; src < nl_.gates().size(); ++src) {
                    if (nl_.gates()[src].output == gate.clk) {
                        from_idx = nl_id_to_idx[src];
                        break;
                    }
                }
             }
             if (from_idx != -1) {
                if (!first_edge) ss << ",\n";
                ss << "    { from: " << from_idx << ", to: " << to_idx << " }";
                first_edge = false;
            }
        }
    }
    
    // Connect to POs
    for (auto& n : nl_.nets()) {
        bool is_output = std::find(nl_.primary_outputs().begin(), nl_.primary_outputs().end(), n.id) != nl_.primary_outputs().end();
        if (is_output) {
            int to_idx = nl_id_to_idx[nl_.gates().size() + n.id];
            int from_idx = -1;
            for (size_t src = 0; src < nl_.gates().size(); ++src) {
                if (nl_.gates()[src].output == n.id) {
                    from_idx = nl_id_to_idx[src];
                    break;
                }
            }
            bool is_pi = std::find(nl_.primary_inputs().begin(), nl_.primary_inputs().end(), n.id) != nl_.primary_inputs().end();
            if (from_idx == -1 && is_pi) {
                 from_idx = nl_id_to_idx[nl_.gates().size() + n.id]; // Feedthrough PI->PO
            }
            if (from_idx != -1 && from_idx != to_idx) {
                if (!first_edge) ss << ",\n";
                ss << "    { from: " << from_idx << ", to: " << to_idx << " }";
                first_edge = false;
            }
        }
    }
    
    ss << "\n  ],\n";
    ss << "  metrics: { nodes: " << idx << ", nets: " << nl_.nets().size() 
       << ", gates: " << nl_.gates().size() << " }\n";
    ss << "};\n";
    return ss.str();
}

std::string RtlVisualizer::generate_css() const {
    return R"(
        @import url('https://fonts.googleapis.com/css2?family=Inter:wght@400;600;800&family=JetBrains+Mono:wght@500;700&display=swap');
        * { margin: 0; padding: 0; box-sizing: border-box; }
        :root { --bg: #0a0a0f; --text: #e8e8f0; --accent: #00d4ff; --border: #2a2a3e; }
        body { background: var(--bg); color: var(--text); font-family: 'Inter', sans-serif; overflow: hidden; }
        .viz-container { width: 100vw; height: 100vh; position: relative; }
        canvas { display: block; width: 100%; height: 100%; cursor: grab; }
        canvas:active { cursor: grabbing; }
        .viz-label { position: absolute; top: 16px; left: 16px; background: rgba(10,10,15,0.9); border: 1px solid var(--border); border-radius: 8px; padding: 8px 14px; font-family: 'JetBrains Mono', monospace; font-size: 0.75rem; color: var(--accent); letter-spacing: 0.1em; text-transform: uppercase; backdrop-filter: blur(10px); z-index: 10; }
        .viz-info { position: absolute; bottom: 16px; left: 16px; background: rgba(10,10,15,0.9); border: 1px solid var(--border); border-radius: 10px; padding: 12px 18px; font-family: 'JetBrains Mono', monospace; font-size: 0.75rem; color: #8888a0; backdrop-filter: blur(10px); z-index: 10; display: flex; gap: 16px; }
        .viz-info .metric { color: var(--accent); font-weight: 600; }
    )";
}

std::string RtlVisualizer::generate_js_logic() const {
    return R"(
        const DPR = window.devicePixelRatio || 1;
        const canvas = document.getElementById('canvas');
        const ctx = canvas.getContext('2d');
        let width, height;
        
        let offset = { x: 0, y: 0 };
        let scale = 1.0;
        let isDragging = false;
        let dragStart = { x: 0, y: 0 };

        function resize() {
            width = window.innerWidth; height = window.innerHeight;
            canvas.width = width * DPR; canvas.height = height * DPR;
            ctx.scale(DPR, DPR);
            if (offset.x === 0 && offset.y === 0) {
               offset = { x: width/2, y: height/2 };
            }
        }
        window.addEventListener('resize', resize);
        
        canvas.addEventListener('mousedown', e => { isDragging = true; dragStart = { x: e.clientX - offset.x, y: e.clientY - offset.y }; });
        window.addEventListener('mouseup', () => isDragging = false);
        window.addEventListener('mousemove', e => { if (isDragging) { offset.x = e.clientX - dragStart.x; offset.y = e.clientY - dragStart.y; } });
        canvas.addEventListener('wheel', e => {
            const wheel = e.deltaY < 0 ? 1.1 : 0.9;
            const newScale = scale * wheel;
            offset.x = e.clientX - (e.clientX - offset.x) * (newScale / scale);
            offset.y = e.clientY - (e.clientY - offset.y) * (newScale / scale);
            scale = newScale;
        });

        // Setup Force Layout nodes
        let nodes = designData.nodes;
        let edges = designData.edges;
        
        // Initial random placement
        nodes.forEach(n => {
            n.x = (Math.random() - 0.5) * 800;
            n.y = (Math.random() - 0.5) * 600;
            n.vx = 0; n.vy = 0;
            // Pin IN points left, OUT points right
            if (n.type === 'IN') n.x = -800;
            if (n.type === 'OUT') n.x = 800;
        });

        const typeColors = {
            'AND': '#00d4ff', 'OR': '#7c3aed', 'NOT': '#f59e0b',
            'MUX': '#ec4899', 'DFF': '#10b981', 'XOR': '#ef4444',
            'BUF': '#6366f1', 'NAND': '#14b8a6', 'IN': '#10b981', 'OUT': '#ef4444'
        };

        let time = 0;
        let animPhase = 0;

        function simulateForces() {
            const k = 0.05, repulsion = 80000, friction = 0.85;
            
            for (let i = 0; i < nodes.length; i++) {
                for (let j = i + 1; j < nodes.length; j++) {
                    let dx = nodes[j].x - nodes[i].x;
                    let dy = nodes[j].y - nodes[i].y;
                    let d2 = dx*dx + dy*dy;
                    if (d2 < 0.1 || d2 > repulsion * 2) continue;
                    let d = Math.sqrt(d2);
                    let f = -repulsion / d2;
                    let fx = f * (dx / d);
                    let fy = f * (dy / d);
                    nodes[i].vx -= fx; nodes[i].vy -= fy;
                    nodes[j].vx += fx; nodes[j].vy += fy;
                }
            }
            
            for (let e of edges) {
                let n1 = nodes[e.from], n2 = nodes[e.to];
                if (!n1 || !n2) continue;
                let dx = n2.x - n1.x;
                let dy = n2.y - n1.y;
                let d = Math.max(0.1, Math.sqrt(dx*dx + dy*dy));
                let f = (d - 100) * k;
                let fx = f * (dx/d);
                let fy = f * (dy/d);
                
                // Keep flow left to right based on edges
                fx += 1.5; 
                
                n1.vx += fx; n1.vy += fy;
                n2.vx -= fx; n2.vy -= fy;
            }
            
            for (let n of nodes) {
                // Pin logic
                if (n.type === 'IN') { n.x = -600; n.vx = 0; n.vy *= friction; }
                else if (n.type === 'OUT') { n.x = 600; n.vx = 0; n.vy *= friction; }
                
                n.x += n.vx;
                n.y += n.vy;
                n.vx *= friction;
                n.vy *= friction;
            }
        }

        function draw() {
            time++; animPhase += 0.02;
            ctx.fillStyle = '#0a0a0f';
            ctx.fillRect(0, 0, width, height);

            ctx.save();
            ctx.translate(offset.x, offset.y);
            ctx.scale(scale, scale);

            if (time < 300) simulateForces();

            // Grid
            if (scale > 0.5) {
                ctx.strokeStyle = 'rgba(40,40,65,0.4)';
                ctx.lineWidth = 1/scale;
                const bs = 50;
                let startX = -Math.floor((offset.x/scale)/bs)*bs - bs*10;
                let endX = startX + (width/scale) + bs*20;
                let startY = -Math.floor((offset.y/scale)/bs)*bs - bs*10;
                let endY = startY + (height/scale) + bs*20;
                
                for(let x=startX; x<endX; x+=bs) { ctx.beginPath(); ctx.moveTo(x, startY); ctx.lineTo(x, endY); ctx.stroke(); }
                for(let y=startY; y<endY; y+=bs) { ctx.beginPath(); ctx.moveTo(startX, y); ctx.lineTo(endX, y); ctx.stroke(); }
            }

            // Edges
            for (let e of edges) {
                let from = nodes[e.from], to = nodes[e.to];
                if (!from || !to) continue;
                ctx.beginPath();
                ctx.strokeStyle = 'rgba(0,212,255,0.15)';
                ctx.lineWidth = 1.5 / scale;
                let cx = (from.x + to.x) / 2;
                ctx.moveTo(from.x, from.y);
                ctx.bezierCurveTo(cx, from.y, cx, to.y, to.x, to.y);
                ctx.stroke();

                // Signal points
                if (scale > 0.3) {
                    let t = ((animPhase + e.from * 0.1) % 1);
                    // Bezier interp
                    let imC1X = from.x + (cx - from.x) * t;
                    let imC1Y = from.y;
                    let imC2X = cx + (to.x - cx) * t;
                    let imC2Y = from.y + (to.y - from.y) * t;
                    let imC3X = to.x;
                    let imC3Y = to.y;
                    let q1X = imC1X + (imC2X - imC1X) * t;
                    let q1Y = imC1Y + (imC2Y - imC1Y) * t;
                    let q2X = imC2X + (imC3X - imC2X) * t;
                    let q2Y = imC2Y + (imC3Y - imC2Y) * t;
                    let sx = q1X + (q2X - q1X) * t;
                    let sy = q1Y + (q2Y - q1Y) * t;

                    ctx.beginPath();
                    ctx.fillStyle = '#00d4ff';
                    ctx.shadowColor = '#00d4ff'; ctx.shadowBlur = 8 / scale;
                    ctx.arc(sx, sy, 2 / scale, 0, Math.PI * 2);
                    ctx.fill();
                    ctx.shadowBlur = 0;
                }
            }

            // Nodes
            for (let n of nodes) {
                let color = typeColors[n.type] || '#00d4ff';
                
                // Glow body
                ctx.beginPath();
                ctx.fillStyle = color + '20';
                ctx.arc(n.x, n.y, n.r*1.3, 0, Math.PI * 2);
                ctx.fill();

                ctx.beginPath();
                ctx.fillStyle = '#1a1a2e';
                ctx.strokeStyle = color;
                ctx.lineWidth = 2 / scale;
                ctx.arc(n.x, n.y, n.r, 0, Math.PI * 2);
                ctx.fill(); ctx.stroke();

                if (scale > 0.4) {
                    ctx.fillStyle = color;
                    ctx.font = `600 ${9}px JetBrains Mono`;
                    ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
                    ctx.fillText(n.type, n.x, n.y);
                }
            }

            ctx.restore();
            requestAnimationFrame(draw);
        }

        resize();
        requestAnimationFrame(draw);
    )";
}

std::string RtlVisualizer::generate_html() const {
    std::ostringstream ss;
    ss << "<!DOCTYPE html>\n<html lang=\"en\">\n<head>\n<meta charset=\"UTF-8\">\n";
    ss << "<title>SiliconForge RTL Graph</title>\n";
    ss << "<style>\n" << generate_css() << "</style>\n";
    ss << "</head>\n<body>\n";

    ss << "<div class=\"viz-container\">\n";
    ss << "  <div class=\"viz-label\">Stage 01 — Circuit Graph</div>\n";
    ss << "  <div class=\"viz-info\">\n";
    ss << "    <div>Nodes: <span class=\"metric\">" << nl_.gates().size() << "</span></div>\n";
    ss << "    <div>Nets: <span class=\"metric\">" << nl_.nets().size() << "</span></div>\n";
    ss << "  </div>\n";
    ss << "  <canvas id=\"canvas\"></canvas>\n";
    ss << "</div>\n";

    ss << "<script>\n";
    ss << generate_json_data();
    ss << generate_js_logic();
    ss << "</script>\n";

    ss << "</body>\n</html>";
    return ss.str();
}

bool RtlVisualizer::export_to_file(const std::string& filename) const {
    std::ofstream out(filename);
    if (!out) return false;
    out << generate_html();
    return true;
}

} // namespace sf
