// SiliconForge — Simulation Waveform Visualizer
#include "viz/sim_viz.hpp"
#include <fstream>
#include <sstream>
#include <iomanip>
#include <algorithm>

namespace sf {

std::string SimVisualizer::generate_json_data() const {
    std::ostringstream ss;
    ss << "const designData = {\n";
    
    // Times
    ss << "  times: [";
    for (size_t i = 0; i < trace_.times.size(); ++i) {
        ss << trace_.times[i] << (i < trace_.times.size() - 1 ? "," : "");
    }
    ss << "],\n";

    // Signals
    ss << "  signals: [\n";
    size_t sig_idx = 0;
    for (auto& [net_id, values] : trace_.traces) {
        if (net_id >= nl_.nets().size()) continue;
        const auto& net = nl_.nets()[net_id];
        
        bool is_pi = std::find(nl_.primary_inputs().begin(), nl_.primary_inputs().end(), net_id) != nl_.primary_inputs().end();
        bool is_po = std::find(nl_.primary_outputs().begin(), nl_.primary_outputs().end(), net_id) != nl_.primary_outputs().end();
        
        // Try to color code typical names
        std::string type = "bit";
        std::string color = "#00d4ff";
        if (net.name.find("clk") != std::string::npos || net.name.find("clock") != std::string::npos) {
            type = "clock"; color = "#10b981";
        } else if (net.name.find("rst") != std::string::npos || net.name.find("reset") != std::string::npos) {
            color = "#f59e0b";
        } else if (net.name.find("d_") != std::string::npos || net.name.find("data") != std::string::npos) {
            color = "#a78bfa";
        } else if (is_po) { // PO
            color = "#f97316";
        } else if (is_pi) { // PI
            color = "#6366f1";
        }

        ss << "    { name: '" << net.name << "', type: '" << type << "', color: '" << color << "', values: [";
        for (size_t i = 0; i < values.size(); ++i) {
            int v = 0;
            switch(values[i]) {
                case Logic4::ZERO: v = 0; break;
                case Logic4::ONE: v = 1; break;
                case Logic4::X: v = 2; break; // X
                case Logic4::Z: v = 3; break; // Z
            }
            ss << v << (i < values.size() - 1 ? "," : "");
        }
        ss << "] }" << (++sig_idx < trace_.traces.size() ? "," : "") << "\n";
    }
    ss << "  ]\n};\n";
    return ss.str();
}

std::string SimVisualizer::generate_css() const {
    return R"(
        @import url('https://fonts.googleapis.com/css2?family=Inter:wght@400;600;800&family=JetBrains+Mono:wght@500;700&display=swap');
        * { margin: 0; padding: 0; box-sizing: border-box; }
        :root { --bg: #0a0a0f; --text: #e8e8f0; --accent: #00d4ff; --border: #2a2a3e; }
        body { background: var(--bg); color: var(--text); font-family: 'Inter', sans-serif; overflow: hidden; }
        .viz-container { width: 100vw; height: 100vh; position: relative; }
        canvas { display: block; width: 100%; height: 100%; cursor: pointer; }
        .viz-label { position: absolute; top: 16px; left: 16px; background: rgba(10,10,15,0.9); border: 1px solid var(--border); border-radius: 8px; padding: 8px 14px; font-family: 'JetBrains Mono', monospace; font-size: 0.75rem; color: var(--accent); letter-spacing: 0.1em; text-transform: uppercase; backdrop-filter: blur(10px); z-index: 10; }
        .viz-info { position: absolute; bottom: 16px; left: 16px; background: rgba(10,10,15,0.9); border: 1px solid var(--border); border-radius: 10px; padding: 12px 18px; font-family: 'JetBrains Mono', monospace; font-size: 0.75rem; color: #8888a0; backdrop-filter: blur(10px); z-index: 10; display: flex; gap: 16px; }
        .viz-info .metric { color: var(--accent); font-weight: 600; }
    )";
}

std::string SimVisualizer::generate_js_logic() const {
    return R"(
        const DPR = window.devicePixelRatio || 1;
        const canvas = document.getElementById('canvas');
        const ctx = canvas.getContext('2d');
        let width, height;
        
        let cursorX = null;
        let pannedX = 0;
        let isDragging = false;
        let dragStartX = 0;
        let scale = 1.0;
        
        const sigHeight = 42;
        const sigGap = 6;
        const leftMargin = 120;
        
        function resize() {
            width = window.innerWidth; height = window.innerHeight;
            canvas.width = width * DPR; canvas.height = height * DPR;
            ctx.scale(DPR, DPR);
            draw();
        }
        window.addEventListener('resize', resize);
        
        canvas.addEventListener('mousedown', e => { 
            if (e.clientX > leftMargin) {
                isDragging = true; dragStartX = e.clientX - pannedX; 
            }
        });
        window.addEventListener('mouseup', () => isDragging = false);
        window.addEventListener('mousemove', e => { 
            if (isDragging) { pannedX = e.clientX - dragStartX; }
            if (e.clientX > leftMargin) { cursorX = e.clientX; } else { cursorX = null; }
            draw();
        });
        canvas.addEventListener('wheel', e => {
            const wheel = e.deltaY < 0 ? 1.1 : 0.9;
            const newScale = scale * wheel;
            if (e.clientX > leftMargin) {
                // Zoom towards mouse
                pannedX = e.clientX - (e.clientX - pannedX) * (newScale / scale);
            }
            scale = newScale;
            draw();
        });

        // Compute max time
        const times = designData.times;
        const maxTime = times.length > 0 ? times[times.length-1] : 100;
        document.getElementById('m-time').innerText = `0\u2013${maxTime}ns`;

        // Pre-process transition points to optimize rendering
        const signals = designData.signals.sort((a,b) => {
            if (a.type === 'clock' && b.type !== 'clock') return -1;
            if (a.type !== 'clock' && b.type === 'clock') return 1;
            return a.name.localeCompare(b.name);
        });

        function draw() {
            ctx.fillStyle = '#0a0a0f';
            ctx.fillRect(0, 0, width, height);

            const totalW = (width - leftMargin - 40) * scale;
            
            // Time Ruler
            ctx.fillStyle = '#1a1a2e';
            ctx.fillRect(leftMargin, 0, width - leftMargin, 35);
            ctx.strokeStyle = '#2a2a3e';
            ctx.beginPath(); ctx.moveTo(leftMargin, 35); ctx.lineTo(width, 35); ctx.stroke();

            // Ruler ticks
            ctx.strokeStyle = 'rgba(40,40,65,0.4)';
            ctx.fillStyle = '#8888a0';
            ctx.font = '500 10px JetBrains Mono';
            ctx.textAlign = 'center';
            for (let i = 0; i <= 20; i++) {
                let x = leftMargin + pannedX + (i / 20) * totalW;
                if (x >= leftMargin && x <= width) {
                    ctx.beginPath(); ctx.moveTo(x, 25); ctx.lineTo(x, height); ctx.stroke();
                    let t = Math.floor((i / 20) * maxTime);
                    ctx.fillText(t + 'ns', x, 20);
                }
            }

            // Signals
            signals.forEach((sig, si) => {
                let y = 50 + si * (sigHeight + sigGap);
                
                // Name plate
                ctx.fillStyle = '#0a0a0f'; // opaque behind text
                ctx.fillRect(0, y - 10, leftMargin, sigHeight + 10);
                
                ctx.fillStyle = sig.color;
                ctx.font = '600 11px JetBrains Mono';
                ctx.textAlign = 'right';
                ctx.fillText(sig.name, leftMargin - 15, y + sigHeight / 2 + 4);

                // Grid seperator
                ctx.strokeStyle = '#2a2a3e80';
                ctx.beginPath(); ctx.moveTo(leftMargin, y + sigHeight + sigGap/2); ctx.lineTo(width, y + sigHeight + sigGap/2); ctx.stroke();

                // Plot wave
                ctx.strokeStyle = sig.color;
                ctx.lineWidth = 1.5;
                
                ctx.save();
                ctx.beginPath();
                ctx.rect(leftMargin, y, width - leftMargin, sigHeight);
                ctx.clip(); // don't draw under nametags

                ctx.beginPath();
                for (let i = 0; i < times.length; i++) {
                    let tv = times[i];
                    let val = sig.values[i];
                    let x = leftMargin + pannedX + (tv / maxTime) * totalW;
                    
                    let isX = val === 2;
                    let yVal = (val === 1) ? y + 5 : (isX ? y + sigHeight/2 : y + sigHeight - 5);
                    
                    if (i === 0) {
                        ctx.moveTo(x, yVal);
                    } else {
                        let prevX = leftMargin + pannedX + (times[i-1] / maxTime) * totalW;
                        let prevVal = sig.values[i-1];
                        let prevY = (prevVal === 1) ? y + 5 : (prevVal === 2 ? y + sigHeight/2 : y + sigHeight - 5);
                        
                        // Extend to current time
                        ctx.lineTo(x, prevY);
                        // Vertical transition
                        if (prevY !== yVal) {
                             ctx.lineTo(x, yVal);
                        }
                    }
                }
                ctx.stroke();

                // Fill logic
                ctx.globalAlpha = 0.08;
                ctx.fillStyle = sig.color;
                for (let i = 0; i < times.length - 1; i++) {
                    if (sig.values[i] === 1) { // Fill under HIGH
                        let x1 = leftMargin + pannedX + (times[i] / maxTime) * totalW;
                        let x2 = leftMargin + pannedX + (times[i+1] / maxTime) * totalW;
                        ctx.fillRect(x1, y + 5, x2 - x1, sigHeight - 10);
                    } else if (sig.values[i] === 2) { // Fill Red for X
                         let x1 = leftMargin + pannedX + (times[i] / maxTime) * totalW;
                         let x2 = leftMargin + pannedX + (times[i+1] / maxTime) * totalW;
                         ctx.fillStyle = '#ef4444';
                         ctx.fillRect(x1, y + 5, x2 - x1, sigHeight - 10);
                         ctx.fillStyle = sig.color;
                    }
                }
                ctx.globalAlpha = 1.0;
                ctx.restore();
            });

            // Cursor
            if (cursorX !== null) {
                ctx.strokeStyle = '#ffffffaa';
                ctx.lineWidth = 1;
                ctx.setLineDash([4, 4]);
                ctx.beginPath(); ctx.moveTo(cursorX, 0); ctx.lineTo(cursorX, height); ctx.stroke();
                ctx.setLineDash([]);
                
                let curTime = ((cursorX - leftMargin - pannedX) / totalW) * maxTime;
                if (curTime >= 0 && curTime <= maxTime) {
                    ctx.fillStyle = '#fff';
                    ctx.font = '700 11px JetBrains Mono';
                    ctx.textAlign = 'center';
                    ctx.fillText(curTime.toFixed(1) + 'ns', cursorX, 15);
                }
            }
        }

        resize();
        
        // Setup render loop only when interacting to save CPU, but draw once now
        let renderRequested = false;
        function requestDraw() {
            if (!renderRequested) {
                renderRequested = true;
                requestAnimationFrame(() => {
                    draw();
                    renderRequested = false;
                });
            }
        }
        canvas.addEventListener('mousemove', requestDraw);
        canvas.addEventListener('wheel', requestDraw);
        // Force initial draw
        setTimeout(draw, 100);
    )";
}

std::string SimVisualizer::generate_html() const {
    std::ostringstream ss;
    ss << "<!DOCTYPE html>\n<html lang=\"en\">\n<head>\n<meta charset=\"UTF-8\">\n";
    ss << "<title>SiliconForge Simulation Waveforms</title>\n";
    ss << "<style>\n" << generate_css() << "</style>\n";
    ss << "</head>\n<body>\n";

    ss << "<div class=\"viz-container\">\n";
    ss << "  <div class=\"viz-label\">Stage 02 — Waveform Viewer</div>\n";
    ss << "  <div class=\"viz-info\">\n";
    ss << "    <div>Time: <span class=\"metric\" id=\"m-time\">...</span></div>\n";
    ss << "    <div>Signals: <span class=\"metric\">" << trace_.traces.size() << "</span></div>\n";
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

bool SimVisualizer::export_to_file(const std::string& filename) const {
    std::ofstream out(filename);
    if (!out) return false;
    out << generate_html();
    return true;
}

} // namespace sf
