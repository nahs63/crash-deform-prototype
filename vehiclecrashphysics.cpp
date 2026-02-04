// VehicleCrashPhysics.cpp
// Simplified vehicle impact & deformation simulation
// Using RAGE-style physics hooks (phInst-inspired impulse, fragment deformation, crush zones)
// Prototype for RAGE engine vehicle damage research — not actual engine code

#include <iostream>
#include <cmath>
#include <random>
#include <chrono>
#include <thread>

namespace rage {

struct Vec3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    Vec3() = default;
    Vec3(float xx, float yy, float zz) : x(xx), y(yy), z(zz) {}

    Vec3 operator+(const Vec3& rhs) const { return {x + rhs.x, y + rhs.y, z + rhs.z}; }
    Vec3 operator-(const Vec3& rhs) const { return {x - rhs.x, y - rhs.y, z - rhs.z}; }
    Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }

    float dot(const Vec3& rhs) const { return x*rhs.x + y*rhs.y + z*rhs.z; }
    Vec3 cross(const Vec3& rhs) const {
        return {
            y * rhs.z - z * rhs.y,
            z * rhs.x - x * rhs.z,
            x * rhs.y - y * rhs.x
        };
    }

    float length() const { return std::sqrt(dot(*this)); }
    Vec3 normalized() const {
        float len = length();
        return (len > 1e-6f) ? *this * (1.0f / len) : *this;
    }
};

struct phVehicleDamage {
    Vec3 position;          // fwEntity root pos (~CoM)
    Vec3 linearVel;         // m/s
    Vec3 angularVel;        // rad/s

    // Local basis (simplified from rage::Matrix34)
    Vec3 forward {0.0f, 1.0f, 0.0f};
    Vec3 right   {1.0f, 0.0f, 0.0f};
    Vec3 up      {0.0f, 0.0f, 1.0f};

    float mass = 1450.0f;   // from handling.meta approx

    // Diagonal inertia tensor (phInst style)
    Vec3 inertiaTensor {1400.0f, 2600.0f, 2200.0f};

    // Deformation scalars (like SET_VEHICLE_DAMAGE zones)
    float deformZones[6] = {0};  // 0:front, 1:rear, 2:left, 3:right, 4:roof, 5:floor

    // Fragment states (detached flags)
    bool fragmentDetached[8] = {false}; // 0–3:wheels, 4–5:doors, 6:hood, 7:trunk
};

namespace phInst {

void ApplyImpulse(phVehicleDamage& veh, const Vec3& worldImpactPos, const Vec3& impulse) {
    Vec3 r = worldImpactPos - veh.position;  // offset from CoM

    // phLinear momentum update
    veh.linearVel = veh.linearVel + impulse * (1.0f / veh.mass);

    // phAngular momentum
    Vec3 angularImpulse = r.cross(impulse);
    veh.angularVel.x += angularImpulse.x / veh.inertiaTensor.x;
    veh.angularVel.y += angularImpulse.y / veh.inertiaTensor.y;
    veh.angularVel.z += angularImpulse.z / veh.inertiaTensor.z;
}

void ProcessDeformation(phVehicleDamage& veh, const Vec3& localImpactPos, float impactEnergy) {
    // Approximate zone mapping (RAGE vehicle archetype deform)
    const float frontThresh =  0.65f;
    const float rearThresh  = -0.65f;
    const float sideThresh  =  0.85f;

    float dmg = impactEnergy * 0.00042f;  // scaled like deformationMult

    if (localImpactPos.y > frontThresh) {
        veh.deformZones[0] = std::min(veh.deformZones[0] + dmg * 1.35f, 1.0f);
    }
    if (localImpactPos.y < rearThresh) {
        veh.deformZones[1] = std::min(veh.deformZones[1] + dmg * 1.10f, 1.0f);
    }
    if (std::abs(localImpactPos.x) > sideThresh) {
        int side = (localImpactPos.x > 0.0f) ? 3 : 2;
        veh.deformZones[side] = std::min(veh.deformZones[side] + dmg * 1.45f, 1.0f);
    }

    // Fragment break chance (high energy threshold)
    if (impactEnergy > 220000.0f && (rand() % 100) < 28) {
        int fragIdx = rand() % 8;
        if (!veh.fragmentDetached[fragIdx]) {
            veh.fragmentDetached[fragIdx] = true;
            std::cout << "[phFragment] Detached index " << fragIdx << "\n";
        }
    }
}

void Integrate(phVehicleDamage& veh, float dt) {
    // Angular damping (phDampening param)
    veh.angularVel = veh.angularVel * 0.965f;

    // fwEntity position integrate
    veh.position = veh.position + veh.linearVel * dt;

    // Simple ground + drag (no full phCollider)
    if (veh.position.z < 0.45f) veh.position.z = 0.45f;
    veh.linearVel = veh.linearVel * 0.991f;
    veh.linearVel.z -= 9.81f * dt * 0.35f;  // gravity scalar
}

} // namespace phInst

} // namespace rage

int main() {
    std::srand(static_cast<unsigned>(std::time(nullptr)));

    rage::phVehicleDamage car;
    car.position    = {0.0f, 0.0f, 0.75f};
    car.linearVel   = {0.0f, 31.0f, 0.8f};  // ~112 km/h forward

    std::cout << "RAGE vehicle test — initial speed: "
              << car.linearVel.length() * 3.6f << " km/h\n\n";

    const float wallX = 38.0f;
    const float dt = 1.0f / 240.0f;
    float simTime = 0.0f;

    while (simTime < 5.0f) {
        // Simulate front collision (no real phCollider query)
        if (car.position.x > wallX - 2.4f && car.linearVel.x > 1.5f) {
            std::cout << "\n=== phImpact detected @ " << simTime << " s ===\n";

            rage::Vec3 localImpact {0.2f, 2.1f, 0.4f};  // front-right
            rage::Vec3 worldImpact = car.position + localImpact;

            rage::Vec3 wallNormal {-1.0f, 0.0f, 0.0f};

            float closingSpeed = car.linearVel.dot(wallNormal);
            if (closingSpeed < -0.4f) {
                float restitution = 0.16f;  // inelastic
                float impulseMag = -(1.0f + restitution) * closingSpeed * car.mass * 0.78f;

                rage::Vec3 impulse = wallNormal * impulseMag;

                rage::phInst::ApplyImpulse(car, worldImpact, impulse);
                float approxKE = 0.5f * car.mass * car.linearVel.length() * car.linearVel.length();
                rage::phInst::ProcessDeformation(car, localImpact, approxKE);

                std::cout << "  Front deform: " << (car.deformZones[0] * 100.0f) << "%\n";
                std::cout << "  Right deform: " << (car.deformZones[3] * 100.0f) << "%\n";
            }
        }

        rage::phInst::Integrate(car, dt);
        simTime += dt;

        // Status log
        static float lastReport = -1.0f;
        if (simTime - lastReport > 0.4f) {
            std::cout << simTime << " s | pos: " << car.position.x << ", " << car.position.y
                      << " | speed: " << (car.linearVel.length() * 3.6f) << " km/h\n";
            lastReport = simTime;
        }
    }

    std::cout << "\nRAGE sim finished.\n";
    return 0;
}