#pragma once
// SiliconForge — Spectral (DCT) Density Smoothing for ePlace
// Replaces naive Gaussian kernel with 2D Discrete Cosine Transform (DCT-II).
// Spectral smoothing produces globally smooth density potential fields,
// enabling better Nesterov convergence in analytical placement.
//
// Algorithm:
//   1. Build density map ρ(bx, by) — cell overlap per bin
//   2. Forward DCT-II: Φ = DCT(ρ - ρ_target)  [O(NxNy log(NxNy))]
//   3. Apply spectral filter: Φ'(u,v) = Φ(u,v) * W(u,v)
//      where W(u,v) = 1 / (1 + α(π²u²/Nx² + π²v²/Ny²))
//      (Poisson-like smoothing kernel in frequency domain)
//   4. Inverse DCT-II: ψ = IDCT(Φ')
//   5. Density gradient: ∂ψ/∂x, ∂ψ/∂y via spectral differentiation
//
// Reference:
//   Lu et al., "ePlace: Electrostatics-Based Placement Using Fast Fourier
//              Transform and Nesterov's Method", ACM TODAES 2015
//   Naylor et al., "Non-Linear Optimization System and Methods for Wire
//                    Length and Delay Optimization", US Patent 6301693

#include <vector>
#include <cmath>
#include <algorithm>

namespace sf {

class SpectralDensity {
public:
    SpectralDensity() = default;

    // Initialize grid parameters
    void init(int nx, int ny, double die_w, double die_h,
              double target_density = 0.7, double alpha = 8.0) {
        nx_ = nx; ny_ = ny;
        die_w_ = die_w; die_h_ = die_h;
        target_ = target_density;
        alpha_ = alpha;
        bw_ = die_w / nx; bh_ = die_h / ny;

        // Pre-compute spectral filter weights
        filter_.assign(ny_, std::vector<double>(nx_, 0.0));
        for (int v = 0; v < ny_; v++) {
            for (int u = 0; u < nx_; u++) {
                double fu = (u > 0) ? M_PI * u / nx_ : 0.0;
                double fv = (v > 0) ? M_PI * v / ny_ : 0.0;
                double freq2 = fu * fu + fv * fv;
                filter_[v][u] = 1.0 / (1.0 + alpha_ * freq2);
            }
        }

        // Pre-compute DCT cosine tables
        cos_x_.assign(nx_, std::vector<double>(nx_));
        cos_y_.assign(ny_, std::vector<double>(ny_));
        for (int u = 0; u < nx_; u++)
            for (int j = 0; j < nx_; j++)
                cos_x_[u][j] = std::cos(M_PI * u * (2.0 * j + 1.0) / (2.0 * nx_));
        for (int v = 0; v < ny_; v++)
            for (int i = 0; i < ny_; i++)
                cos_y_[v][i] = std::cos(M_PI * v * (2.0 * i + 1.0) / (2.0 * ny_));
    }

    // Compute smoothed potential field from density map
    // Input: density[ny][nx] — cell density per bin (excess above target)
    // Output: potential[ny][nx] — smoothed electrostatic potential
    std::vector<std::vector<double>> smooth(
            const std::vector<std::vector<double>>& density) const {
        int ny = ny_, nx = nx_;
        // Step 1: excess density
        std::vector<std::vector<double>> excess(ny, std::vector<double>(nx, 0.0));
        for (int r = 0; r < ny; r++)
            for (int c = 0; c < nx; c++)
                excess[r][c] = density[r][c] - target_;

        // Step 2: Forward DCT-II
        auto dct = forward_dct(excess);

        // Step 3: Apply spectral filter (low-pass Poisson kernel)
        for (int v = 0; v < ny; v++)
            for (int u = 0; u < nx; u++)
                dct[v][u] *= filter_[v][u];

        // Step 4: Inverse DCT-II
        return inverse_dct(dct);
    }

    // Compute density gradients via spectral differentiation
    // Returns {grad_x[ny][nx], grad_y[ny][nx]}
    struct DensityField {
        std::vector<std::vector<double>> potential;
        std::vector<std::vector<double>> grad_x;
        std::vector<std::vector<double>> grad_y;
        double overflow = 0.0;
    };

    DensityField compute_field(
            const std::vector<std::vector<double>>& density) const {
        DensityField field;
        int ny = ny_, nx = nx_;

        // Excess density
        std::vector<std::vector<double>> excess(ny, std::vector<double>(nx, 0.0));
        for (int r = 0; r < ny; r++)
            for (int c = 0; c < nx; c++) {
                excess[r][c] = density[r][c] - target_;
                if (excess[r][c] > 0) field.overflow += excess[r][c];
            }

        // Forward DCT
        auto dct = forward_dct(excess);

        // Smoothed potential: apply filter + IDCT
        auto filtered = dct;
        for (int v = 0; v < ny; v++)
            for (int u = 0; u < nx; u++)
                filtered[v][u] *= filter_[v][u];
        field.potential = inverse_dct(filtered);

        // Gradient X: multiply spectral coefficients by i*u*π/Nx (DST for x-derivative)
        // d/dx in spectral domain: Φ_x(u,v) = -u*π/Nx * Φ(u,v)
        // The derivative of DCT becomes a DST
        field.grad_x.assign(ny, std::vector<double>(nx, 0.0));
        field.grad_y.assign(ny, std::vector<double>(nx, 0.0));

        // Numerical gradient from potential (central differences) — more robust
        for (int r = 0; r < ny; r++) {
            for (int c = 0; c < nx; c++) {
                double pl = (c > 0)      ? field.potential[r][c-1] : field.potential[r][c];
                double pr = (c < nx - 1) ? field.potential[r][c+1] : field.potential[r][c];
                double pd = (r > 0)      ? field.potential[r-1][c] : field.potential[r][c];
                double pu = (r < ny - 1) ? field.potential[r+1][c] : field.potential[r][c];
                field.grad_x[r][c] = (pr - pl) / (2.0 * bw_);
                field.grad_y[r][c] = (pu - pd) / (2.0 * bh_);
            }
        }

        return field;
    }

    int nx() const { return nx_; }
    int ny() const { return ny_; }
    double bin_w() const { return bw_; }
    double bin_h() const { return bh_; }

private:
    int nx_ = 0, ny_ = 0;
    double die_w_ = 0, die_h_ = 0;
    double target_ = 0.7;
    double alpha_ = 8.0;
    double bw_ = 0, bh_ = 0;
    std::vector<std::vector<double>> filter_;
    std::vector<std::vector<double>> cos_x_, cos_y_;

    // Forward DCT-II: F(u,v) = sum_j sum_i f(i,j) * cos(...) * cos(...)
    // Using pre-computed cosine tables for O(N²) per coefficient
    // Total: O(Nx² * Ny²) — acceptable for typical 64x64 grids
    // For larger grids (512+), would use FFT-based DCT
    std::vector<std::vector<double>> forward_dct(
            const std::vector<std::vector<double>>& f) const {
        int ny = ny_, nx = nx_;
        std::vector<std::vector<double>> F(ny, std::vector<double>(nx, 0.0));

        // Separable: DCT rows then DCT columns
        // Step A: DCT along columns (x-direction) for each row
        std::vector<std::vector<double>> temp(ny, std::vector<double>(nx, 0.0));
        for (int i = 0; i < ny; i++) {
            for (int u = 0; u < nx; u++) {
                double sum = 0.0;
                for (int j = 0; j < nx; j++)
                    sum += f[i][j] * cos_x_[u][j];
                double cu = (u == 0) ? 1.0 / std::sqrt(nx) : std::sqrt(2.0 / nx);
                temp[i][u] = cu * sum;
            }
        }
        // Step B: DCT along rows (y-direction) for each column
        for (int u = 0; u < nx; u++) {
            for (int v = 0; v < ny; v++) {
                double sum = 0.0;
                for (int i = 0; i < ny; i++)
                    sum += temp[i][u] * cos_y_[v][i];
                double cv = (v == 0) ? 1.0 / std::sqrt(ny) : std::sqrt(2.0 / ny);
                F[v][u] = cv * sum;
            }
        }
        return F;
    }

    // Inverse DCT-II (orthogonal inverse = DCT-III)
    std::vector<std::vector<double>> inverse_dct(
            const std::vector<std::vector<double>>& F) const {
        int ny = ny_, nx = nx_;
        std::vector<std::vector<double>> f(ny, std::vector<double>(nx, 0.0));

        // Separable: IDCT columns then IDCT rows
        // Step A: IDCT along rows (y-direction)
        std::vector<std::vector<double>> temp(ny, std::vector<double>(nx, 0.0));
        for (int u = 0; u < nx; u++) {
            for (int i = 0; i < ny; i++) {
                double sum = 0.0;
                for (int v = 0; v < ny; v++) {
                    double cv = (v == 0) ? 1.0 / std::sqrt(ny) : std::sqrt(2.0 / ny);
                    sum += cv * F[v][u] * cos_y_[v][i];
                }
                temp[i][u] = sum;
            }
        }
        // Step B: IDCT along columns (x-direction)
        for (int i = 0; i < ny; i++) {
            for (int j = 0; j < nx; j++) {
                double sum = 0.0;
                for (int u = 0; u < nx; u++) {
                    double cu = (u == 0) ? 1.0 / std::sqrt(nx) : std::sqrt(2.0 / nx);
                    sum += cu * temp[i][u] * cos_x_[u][j];
                }
                f[i][j] = sum;
            }
        }
        return f;
    }
};

} // namespace sf
