/*
    Copyright © 2024–2026 rzrn

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include <Milsim/Engine.hxx>

// Makes sense only for points in the voxel interior.
inline auto voxelOf(const Vector3d & r)
{ return Vector3i(std::floor(r.x), std::floor(r.y), std::ceil(r.z)); }

template<typename T> Vector3<T> cone(const Vector3<T> & v, const T σ) {
    static std::random_device rd;
    static std::mt19937 randgen(rd());

    std::normal_distribution gauss(0.0, σ);
    std::uniform_real_distribution uniform(-std::numbers::pi_v<T>, std::numbers::pi_v<T>);

    auto n = v.normal(); auto left = Vector3<T>(n.y, -n.x, 0).normal();
    auto α = std::fabs(gauss(randgen)), β = uniform(randgen);

    return v.rot(left, α).rot(n, β);
}

template<typename T> inline Vector3<T> reflect(const Vector3<T> & v, const Vector3<T> & n)
{ return v - n * (2 * (v, n)); }

Voxel & VoxelData::set(int i, PyObject * o) {
    int z = i % MAP_Z;

    if (63 <= z) return water; // ignore z = 63

    auto d = z < 62 ? 1.0 : std::numeric_limits<double>::infinity();

    if (o == nullptr) o = defaultMaterial;

    auto [iter, _] = data.insert_or_assign(i, Voxel(o, d));
    return iter->second;
}

Voxel & VoxelData::get(int x, int y, int z) {
    if (63 <= z) return water;

    auto i = map_to_xy(x, y) * MAP_Z + z; auto iter = data.find(i);
    return iter == data.end() ? set(i, defaultMaterial) : iter->second;
}

uint64_t Object::gidx = 0;

void Engine::clear() {
    temperature = 0;
    pressure    = 101325;
    humidity    = 0.3;

    wind = {};

    update();

    _lag = _peak = 0.0;

    objects.clear();
    Object::flush();

    vxlData.clear();
}

void Engine::update() {
    using namespace Fundamentals;

    const auto & t = temperature, & p = pressure, & φ = humidity; const auto & w = wind;

    // 1) Here we assume Dalton’s law.

    // https://en.wikipedia.org/wiki/Density_of_air#Humid_air
    auto p₁ = φ * vaporPressureOfWater<double>(t), p₂ = p - p₁;

    auto ε = gasConstant<double> * (t - absoluteZero<double>);

    _density = (p₁ * molarMassWaterVapor<double> + p₂ * molarMassDryAir<double>) / ε;
    _ppo2    = 0.20946 * p₂;

    // 2) Here we assume Amagat’s law.

    // https://en.wikipedia.org/wiki/Heat_capacity_ratio#Relation_with_degrees_of_freedom
    constexpr double γ₁ = 1.333333, γ₂ = 1.4;

    // https://physicspages.com/pdf/Thermal%20physics/Bulk%20modulus%20and%20the%20speed%20of%20sound.pdf
    // pV^γ = A, p = AV^−γ, K = −VdP/V = −Vd(AV^−γ)/V = −VA(−γ)V^(−γ + 1) = γAV^−γ = γp
    auto K₁ = γ₁ * p, K₂ = γ₂ * p;

    // https://en.wikipedia.org/wiki/Partial_pressure#Partial_volume_(Amagat's_law_of_additive_volume)
    auto x₁ = p₁ / p, x₂ = p₂ / p;

    /*
        https://eng.libretexts.org/Bookshelves/Civil_Engineering/Book%3A_Fluid_Mechanics_(Bar-Meir)/00%3A_Introduction/1.6%3A_Fluid_Properties/1.6.2%3A_Bulk_Modulus/1.6.2.1%3A_Bulk_Modulus_of_Mixtures

        Kᵢ = −VᵢdP/dVᵢ,
        dV = dV₁ + dV₂
           = −V₁dP/K₁ − V₂dP/K₂
           = −x₁VdP/K₁ − x₂VdP/K₂
           = −VdP(x₁/K₁ + x₂/K₂),
        K = −VdP/dV = 1/(x₁/K₁ + x₂/K₂)
    */
    auto K = 1.0 / (x₁ / K₁ + x₂ / K₂);

    // https://en.wikipedia.org/wiki/Speed_of_sound#Equations
    _mach = std::sqrt(K / _density);

    // See also: http://resource.npl.co.uk/acoustics/techguides/speedair/

    const auto & h = x₁ * 100; // %

    constexpr double p₀ = 101'325, /* Pa */ T₀ = 293.15 /* K */;
    const double T = t - absoluteZero<double>; /* K */

    /* ISO 9613-1:1993(E), Acoustics — Attenuation of sound during propagation outdoors — 
       Part 1: Calculation of the absorption of sound by the atmosphere. */

    _oxygenRelaxationFrequency   = (p / p₀) * (24 + 4.04e+4 * h + (0.02 + h) / (0.391 + h));
    _nitrogenRelaxationFrequency = (p / p₀) * std::sqrt(T₀ / T) * (9 + 280 * h * exp(-4.17 * (std::cbrt(T₀ / T) - 1)));
}

void Engine::step(const double t1, const double t2) {
    using namespace std::chrono;

    const auto T1 = steady_clock::now();

    for (auto it = objects.begin(); it != objects.end(); next(t1, t2, it));

    const auto T2 = steady_clock::now();

    auto diff = duration_cast<microseconds>(T2 - T1).count();
    _lag  = (_lag + diff) / 2;
    _peak = std::max(_peak, double(diff));
}

template<typename T> inline auto traverseReciprocal(const Vector3<T> & r, const Vector3<T> & v) {
    using namespace Fundamentals;

    T x = v.x > T(0) ? std::floor(r.x) + 1 : std::ceil(r.x) - 1;
    T y = v.y > T(0) ? std::floor(r.y) + 1 : std::ceil(r.y) - 1;
    T z = v.z > T(0) ? std::floor(r.z) + 1 : std::ceil(r.z) - 1;

    T dx = x - r.x, dy = y - r.y, dz = z - r.z;

    if (std::fabs(dx) < 1e-20) dx = sign<T>(v.x);
    if (std::fabs(dy) < 1e-20) dy = sign<T>(v.y);
    if (std::fabs(dz) < 1e-20) dz = sign<T>(v.z);

    return max(
        [](auto & w1, auto & w2){ return w1.first < w2.first; },
        std::pair(v.x / dx, Vector3<T>(-sign<T>(v.x), 0, 0)),
        std::pair(v.y / dy, Vector3<T>(0, -sign<T>(v.y), 0)),
        std::pair(v.z / dz, Vector3<T>(0, 0, -sign<T>(v.z)))
    );
}

static inline auto traverse(const Vector3d & r, const Vector3d & v /* m/s */, const double rem /* s */) {
    auto [idt, n] = traverseReciprocal<double>(r, ofMeters3<double>(v));
    double dt = idt < 1e-9 ? INFINITY : 1 / idt;

    return rem < dt ? std::pair(rem, Vector3d())
                    : std::pair(dt, n);
}

inline int Engine::intersectPlayer(const Ray<double> & ray, Arc<double> & retval) {
    int target = -1;

    for (size_t i = 0; i < players.size(); i++) {
        auto & player = players[i];
        if (!player.valid()) continue;

        auto arc = player.intersect(ray);
        if (arc < retval) { retval = arc; target = i; }
    }

    return target;
}

inline bool Engine::impactPlayer(Object & o, const int target, const Vector3i & R,
                                 const Ray<double> & ray, const Arc<double> & arc) {
    auto w = arc.begin(ray);

    auto & v = o.velocity;
    bool stuck = Py_True == onPlayerHit(
        o.object(), w.x, w.y, w.z, v.x, v.y, v.z, R.x, R.y, R.z,
        o.thrower(), o.energy(), o.area, target, arc.index
    );

    if (stuck) { o.position = w; o.invalidate(); return true; }

    trace(o.index(), w, v.abs() / o.v0(), false);
    o.position = arc.end(ray);

    return false;
}

// http://panoptesv.com/RPGs/Equipment/Weapons/Projectile_physics.php
inline double maximumImpactDepth(Material * M, double m, double A, double E₀) {
    constexpr double drag = 1;

    auto xc = m / (drag * M->density * A);
    return xc * log(1 + (E₀ * drag * M->density) / (M->strength * m));
}

inline double impactRemainingEnergy(Material * M, double m, double A, double E₀, double x) {
    constexpr double drag = 1;

    auto ε = exp(-drag * A * M->density * x / m);
    return E₀ * ε - M->strength * m * (1 - ε) / (drag * M->density);
}

inline bool Engine::terminal(Object & o, const Vector3i & R, const Vector3d & dr) {
    using namespace Fundamentals;

    Voxel & voxel = vxlData.get(R);
    Material * M = voxel.material();

    auto E₀ = o.energy();

    auto depth = toMeters<double>(dr.abs());
    auto E = impactRemainingEnergy(M, o.mass, o.area, E₀, depth);

    auto ΔE = E > 0 ? E₀ - E : E₀;

    if (0 < E) {
        o.position += dr;
        o.velocity *= std::sqrt(E / E₀);
    } else
        o.velocity = {};

    if (voxel.isub(ΔE * (M->durability / M->absorption)))
        onDestroy(o.thrower(), R.x, R.y, R.z);

    return E <= 0;
}

inline void Engine::external(Object & o, const double dt, const Vector3d & dr) {
    using namespace Fundamentals;

    auto m = o.mass;
    auto u = wind - o.velocity;

    auto CD = drag(o.model(), o.ballistic, u.abs() / _mach);
    auto F  = g<double> * m + u * (0.5 * _density * u.abs() * CD * o.area);

    o.position += dr;
    o.velocity += F * (dt / m);
}

inline bool Engine::impactSurface(Object & o, const Vector3i & R, const Vector3d & n, const Vector3d & r) {
    auto & v = o.velocity;

    Voxel & voxel = vxlData.get(R);
    Material * M = voxel.material();

    constexpr double hitEffectThresholdEnergy = 5.0;
    bool stuck = hitEffectThresholdEnergy <= o.energy()
              && Py_True == onBlockHit(
                  o.object(),
                  r.x, r.y, r.z, v.x, v.y, v.z, R.x, R.y, R.z,
                  o.thrower(), o.energy(), o.area
              );

    if (stuck) { o.invalidate(); return true; }

    trace(o.index(), r, v.abs() / o.v0(), false);

    auto θ = acos(-(v, n) / v.abs());

    constexpr double ricochetSlowdown = 0.6; // TODO: something more reasonable

    if (M->deflecting <= θ && random<double>() < M->ricochet)
        v = reflect(v, n) * ricochetSlowdown;
    else
        v = cone(v, 0.05);

    return false;
}

void Engine::next(double t1, const double t2, ObjectIterator & it) {
    using namespace Fundamentals;

    Object & o = *it; Vector3d & r = o.position, & v = o.velocity;

    for (uint64_t N = 1; N < 10000 && t1 < t2; N++) {
        auto [dt, n] = traverse(r, v, t2 - t1);
        auto dr = v * (m2b<double> * dt);

        /* Midpoint of [r; r + dr] (= (r + r + dr) / 2 = r + dr / 2)
           is in the interior of the cube in which the projectile is,
           except for the rare case when r and (r + dr) are on the same face. */
        auto R = voxelOf(r + dr * 0.5);

        Ray<double> ray(r, dr); Arc<double> arc{};
        int target = intersectPlayer(ray, arc);

        if (0 <= target) {
            if (impactPlayer(o, target, R, ray, arc))
                break;
        } else if (solid(R)) {
            if (terminal(o, R, dr))
                break;
        } else {
            external(o, dt, dr);

            auto T = R - Vector3i(n);
            if (!n.isZero() && solid(T)) {
                if (impactSurface(o, T, n, r))
                    break;
            }
        }

        t1 += dt;
    }

    trace(o.index(), r, v.abs() / o.v0(), false);

    auto P = t2 - o.timestamp() <= 10;
    auto Q = 1.0 <= o.velocity.abs();
    auto R = 0 <= o.position.x && o.position.x < MAP_X
          && 0 <= o.position.y && o.position.y < MAP_Y
          && 0 <= o.position.z && o.position.z < MAP_Z;
    auto S = o.valid();

    //if (!P) printf("%ld: time out\n", o.index());
    //if (!Q) printf("%ld: speed too low (%f m/s)\n", o.index(), o.velocity.abs());
    //if (!R) printf("%ld: out of map (%f, %f, %f)\n", o.index, o.position.x, o.position.y, o.position.z);
    //if (!S) printf("%ld: invalidated\n", o.index);

    if (P && Q && R && S) ++it; else it = objects.erase(it);
}

double Engine::dragRaycast(double CD, double m, double A, double v₀, Vector3d r, const Vector3d & s) {
    double E = 0.5 * m * v₀ * v₀;
    auto [d, n] = (s - r).polar();

    while (d > 0) {
        auto [idR, _] = traverseReciprocal<double>(r, n);

        double dR = std::min(1 / idR, d);
        auto depth = toMeters<double>(dR);

        auto dr = n * dR;

        auto R = voxelOf(r + dr * 0.5);
        if (solid(R)) {
            Material * M = vxlData.get(R).material();
            E = impactRemainingEnergy(M, m, A, E, depth);

            if (E <= 0) return 0;
        } else {
            /* For a quadratic drag force F = −cv² (neglecting gravity) we have that:
                 (1) F = m dv/dt,
                 (2) dv/dt = dv/dr dr/dt = vdv/dr,
                 (3) m vdv/dr = F = −cv²,
               so dv/v = −(c/m)dr.

               Evaluating integral we get that ln(v) = −(c/m)r + C(r₀), v = v₀exp(−cr/m),
               thus E/E₀ = (mv²/2)/(mv₀²/2) = (v/v₀)² = exp(−2cr/m).

               For c = 1/2 · CD · A · ρ we finally obtain that E/E₀ = exp(−CD · A · ρ · r / m). */
            E *= exp(-CD * A * _density * depth / m);
        }

        d -= dR;
        r += dr;
    }

    return std::sqrt(2 * E / m);
}

double Engine::HopkinsonCranzCoefficient(double W /* TNT equivalent, kg */) {
    // Explosive Shocks in Air, G. F. Kinney, K. J. Graham, 2nd edition,
    // Ch. 7 “The Scaling Law”, Scaled Distance, p. 109

    using namespace Fundamentals;

    constexpr double p₀ = 101'325, /* Pa */ T₀ = 288.15 /* K */;
    const double p = pressure, /* Pa */ T = temperature - absoluteZero<double>; /* K */

    return std::cbrt(1 / W * p / p₀ * T₀ / T);
}

double Engine::attenuationCoefficient(double f /* Hz */) {
    using namespace Fundamentals;

    // ISO 9613-1:1993(E)

    constexpr double p₀ = 101'325, /* Pa */ T₀ = 293.15 /* K */;
    const double p = pressure, /* Pa */ T = temperature - absoluteZero<double>; /* K */

    const double & frO = _oxygenRelaxationFrequency;   /* Hz */
    const double & frN = _nitrogenRelaxationFrequency; /* Hz */

    const double A  = 1.84e-11 * (p₀ / p) * std::sqrt(T / T₀);
    const double B₁ = 0.01275 * exp(-2239.1 / T) / (frO + f * f / frO);
    const double B₂ = 0.10680 * exp(-3352.0 / T) / (frN + f * f / frN);

    return 8.686 * f * f * (A + powf(T / T₀, -2.5) * (B₁ + B₂));
}