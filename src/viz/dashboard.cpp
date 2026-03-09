// SiliconForge — Ultimate GUI Dashboard
#include "viz/dashboard.hpp"
#include <sstream>

namespace sf {

DashboardVisualizer::DashboardVisualizer(const Netlist& nl, const PhysicalDesign& pd)
    : nl_(nl), pd_(pd) {}

std::string DashboardVisualizer::generate_css() const {
    return R"HTML(
        :root { --bg: #09090b; --fg: #fafafa; --accent: #06b6d4; --panel: #18181b; }
        body { margin: 0; padding: 0; background: var(--bg); color: var(--fg); font-family: 'Inter', sans-serif; display: flex; flex-direction: column; height: 100vh; }
        header { padding: 1rem 2rem; background: var(--panel); border-bottom: 1px solid #27272a; display: flex; justify-content: space-between; align-items: center; }
        header h1 { margin: 0; font-size: 1.5rem; font-weight: 700; background: linear-gradient(90deg, #06b6d4, #3b82f6); -webkit-background-clip: text; -webkit-text-fill-color: transparent; }
        .main-container { display: flex; flex: 1; overflow: hidden; }
        .sidebar { width: 300px; background: var(--panel); border-right: 1px solid #27272a; padding: 1.5rem; overflow-y: auto; }
        .content { flex: 1; position: relative; display: flex; flex-direction: column; }
        .metric-card { background: #27272a; padding: 1rem; border-radius: 8px; margin-bottom: 1rem; border: 1px solid #3f3f46; }
        .metric-card h3 { margin: 0 0 0.5rem 0; font-size: 0.875rem; color: #a1a1aa; text-transform: uppercase; letter-spacing: 0.05em; }
        .metric-card .val { font-size: 1.5rem; font-weight: 600; color: var(--accent); }
        .tabs { display: flex; background: var(--panel); border-bottom: 1px solid #27272a; }
        .tab { padding: 1rem 2rem; cursor: pointer; color: #a1a1aa; font-weight: 500; transition: all 0.2s; border-bottom: 2px solid transparent; }
        .tab:hover { color: var(--fg); }
        .tab.active { color: var(--accent); border-bottom-color: var(--accent); }
        .tab-content { flex: 1; position: relative; display: none; }
        .tab-content.active { display: block; }
        canvas { width: 100%; height: 100%; display: block; }
    )HTML";
}

std::string DashboardVisualizer::generate_js() const {
    std::ostringstream js;
    js << "const pd = {\n";
    js << "  width: " << pd_.die_area.width() << ",\n";
    js << "  height: " << pd_.die_area.height() << ",\n";
    js << "  cells: [\n";
    for (const auto& c : pd_.cells) {
        js << "    { x: " << c.position.x << ", y: " << c.position.y << ", w: " << c.width << ", h: " << c.height << ", type: '" << c.cell_type << "' },\n";
    }
    js << "  ],\n";
    js << "  wires: [\n";
    for (const auto& w : pd_.wires) {
        js << "    { x1: " << w.start.x << ", y1: " << w.start.y << ", x2: " << w.end.x << ", y2: " << w.end.y << ", layer: " << w.layer << " },\n";
    }
    js << "  ]\n";
    js << "};\n";

    js << R"HTML(
        function switchTab(id) {
            document.querySelectorAll('.tab').forEach(t => t.classList.remove('active'));
            document.querySelectorAll('.tab-content').forEach(c => c.classList.remove('active'));
            document.querySelector(`[onclick="switchTab('${id}')"]`).classList.add('active');
            document.getElementById(id).classList.add('active');
            if (id === 'layout') drawLayout();
        }

        function drawLayout() {
            const canvas = document.getElementById('layoutCanvas');
            const ctx = canvas.getContext('2d');
            canvas.width = canvas.parentElement.clientWidth;
            canvas.height = canvas.parentElement.clientHeight;
            
            ctx.fillStyle = '#09090b';
            ctx.fillRect(0, 0, canvas.width, canvas.height);
            
            // Transform
            const padding = 50;
            let scaleZoom = Math.min((canvas.width - padding*2) / (pd.width || 100), (canvas.height - padding*2) / (pd.height || 100));
            if (!isFinite(scaleZoom) || scaleZoom <= 0) scaleZoom = 1;
            
            ctx.save();
            ctx.translate(padding, canvas.height - padding); // flip Y
            ctx.scale(scaleZoom, -scaleZoom);
            
            // Draw Die Bounds
            ctx.strokeStyle = '#3f3f46';
            ctx.lineWidth = 2/scaleZoom;
            ctx.strokeRect(0, 0, pd.width, pd.height);

            // Draw Cells
            ctx.fillStyle = 'rgba(6, 182, 212, 0.2)';
            ctx.strokeStyle = '#06b6d4';
            ctx.lineWidth = 1/scaleZoom;
            pd.cells.forEach(c => {
                ctx.fillRect(c.x, c.y, c.w, c.h);
                ctx.strokeRect(c.x, c.y, c.w, c.h);
            });

            // Draw Wires
            ctx.strokeStyle = 'rgba(236, 72, 153, 0.8)'; // Pinkish routed wires
            ctx.lineWidth = 2/scaleZoom;
            ctx.lineCap = 'round';
            ctx.lineJoin = 'round';
            ctx.beginPath();
            pd.wires.forEach(w => {
                ctx.moveTo(w.x1, w.y1);
                ctx.lineTo(w.x2, w.y2);
            });
            ctx.stroke();
            
            ctx.restore();
        }
        
        window.addEventListener('resize', () => {
            if (document.getElementById('layout').classList.contains('active')) drawLayout();
        });
        
        // Init
        switchTab('layout');
    )HTML";
    return js.str();
}

std::string DashboardVisualizer::render_nav() const {
    return R"HTML(
        <header>
            <h1>SiliconForge EDA</h1>
            <div style="color: #a1a1aa; font-size: 0.875rem;">Status: <span style="color: #10b981;">Signoff Complete</span></div>
        </header>
    )HTML";
}

std::string DashboardVisualizer::render_metrics_view() const {
    std::ostringstream oss;
    oss << R"HTML(
        <div class="sidebar">
            <h2 style="margin-top: 0; font-size: 1.1rem;">Design Metrics</h2>
            
            <div class="metric-card">
                <h3>Total Nets</h3>
                <div class="val">)HTML" << nl_.num_nets() << R"HTML(</div>
            </div>
            
            <div class="metric-card">
                <h3>Total Logic Gates</h3>
                <div class="val">)HTML" << nl_.num_gates() << R"HTML(</div>
            </div>

            <div class="metric-card">
                <h3>Total Std Cells</h3>
                <div class="val">)HTML" << pd_.cells.size() << R"HTML(</div>
            </div>

            <div class="metric-card">
                <h3>Routed Wire Segments</h3>
                <div class="val">)HTML" << pd_.wires.size() << R"HTML(</div>
            </div>
            
            <div class="metric-card">
                <h3>Total Wirelength</h3>
                <div class="val">)HTML" << pd_.total_wirelength() << R"HTML( &#181;m</div>
            </div>
        </div>
    )HTML";
    return oss.str();
}

std::string DashboardVisualizer::render_layout_view() const {
    return R"HTML(
        <div id="layout" class="tab-content">
            <canvas id="layoutCanvas"></canvas>
        </div>
        <div id="rtl" class="tab-content" style="padding: 2rem;">
            <h2 style="color: #3b82f6;">RTL Netlist Details</h2>
            <p style="color: #a1a1aa;">The RTL graph renderer is integrated into a separate view for performance. Use the <code>rtl_out.html</code> for full 3D interactive netlist exploration.</p>
        </div>
    )HTML";
}

std::string DashboardVisualizer::generate_html() const {
    std::ostringstream oss;
    oss << "<!DOCTYPE html>\n<html lang=\"en\">\n<head>\n";
    oss << "    <meta charset=\"UTF-8\">\n";
    oss << "    <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">\n";
    oss << "    <title>SiliconForge Ultimate Dashboard</title>\n";
    oss << "    <link href=\"https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600;700&display=swap\" rel=\"stylesheet\">\n";
    oss << "    <style>\n" << generate_css() << "    </style>\n";
    oss << "</head>\n<body>\n";
    
    oss << render_nav();
    
    oss << "    <div class=\"main-container\">\n";
    oss << render_metrics_view();
    oss << "        <div class=\"content\">\n";
    oss << "            <div class=\"tabs\">\n";
    oss << "                <div class=\"tab active\" onclick=\"switchTab('layout')\">Physical Layout</div>\n";
    oss << "                <div class=\"tab\" onclick=\"switchTab('rtl')\">RTL Data</div>\n";
    oss << "            </div>\n";
    oss << render_layout_view();
    oss << "        </div>\n";
    oss << "    </div>\n";
    
    oss << "    <script>\n" << generate_js() << "    </script>\n";
    oss << "</body>\n</html>";
    return oss.str();
}

} // namespace sf
