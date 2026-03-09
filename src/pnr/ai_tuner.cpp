// SiliconForge — AI-Driven PnR Tuner
#include "pnr/ai_tuner.hpp"
#include "pnr/placer.hpp"
#include "ml/congestion_model.hpp"
#include "ml/timing_model.hpp"
#include <chrono>
#include <iostream>

namespace sf {

TunerResult AiTuner::optimize(int max_iterations, double clock_period) {
    auto t0 = std::chrono::high_resolution_clock::now();
    TunerResult r;
    
    // Save original placement
    auto original_cells = pd_.cells;
    
    // Simple Grid Search / Random Search loop
    // In a full production system this would use BayesOpt or RL.
    
    double best_score = -1e18; // We want to maximize: alpha*WNS - beta*Congestion
    
    for (int i = 0; i < max_iterations; ++i) {
        // 1. Propose hyperparameters
        // Grid search on density vs wirelength
        double density_wt = 0.5 + (i * 0.1); 
        double wl_wt = 1.5 - (i * 0.1);
        
        // 2. Run Placer
        sf::AnalyticalPlacer placer(pd_);
        placer.set_hyperparams(wl_wt, density_wt);
        placer.place();
        
        // 3. Evaluate ML metrics
        MlCongestionPredictor cong(pd_);
        auto c_res = cong.predict();
        
        MlTimingPredictor timer(nl_, pd_, nullptr);
        auto t_res = timer.predict(clock_period);
        
        // 4. Reward function
        // Penalize congestion overflows heavily, reward positive slack
        double score = (t_res.predicted_wns * 100) - (c_res.peak_congestion * 50);
        
        if (score > best_score) {
            best_score = score;
            r.best_wns = t_res.predicted_wns;
            r.best_congestion = c_res.peak_congestion;
            r.optimal_density_weight = density_wt;
            r.optimal_wirelength_weight = wl_wt;
            original_cells = pd_.cells; // Save best state
            
            std::cout << "AI Tuner: [Iter " << i << "] New Best! WNS=" 
                      << r.best_wns << " ns, Cong=" << r.best_congestion 
                      << ", D=" << density_wt << ", W=" << wl_wt << "\n";
        }
    }
    
    // Restore best placement
    pd_.cells = original_cells;
    r.iterations = max_iterations;

    auto t1 = std::chrono::high_resolution_clock::now();
    r.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    return r;
}

} // namespace sf
