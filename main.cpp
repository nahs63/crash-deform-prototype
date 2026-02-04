
#include <iostream>
#include <cmath>
#include <random>
#include <chrono>
#include <thread>

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

struct VehicleDamageState {
    Vec3 position;          // world-space root position (~CoM)
    Vec3 linearVel;         // m/s
    Vec3 angularVel;        // rad/s

    // Local basis (simplified — real RAGE would use full matrix/quat)
    Vec3 forward {0.0f, 1.0f, 0.0f};
    Vec3 right   {1.0f, 0.0f, 0.0f};
    Vec3 up      {0.0f, 0.0f, 1.0f};

    float mass = 1450.0f;   // kg, approximate mid-size sedan

    // Diagonal inertia tensor approximation (kg·m²)
    Vec3 inertiaTensor {1400.0f, 2600.0f, 2200.0f};

    // Per-zone crush deformation (0.0 → 1.0)
    float crushZones[6] = {0};  // 0:front, 1:rear, 2:left, 3:right, 4:roof, 5:floor

    // Detached/broken components
    bool detached[8] = {false}; // 0–3:wheels, 4–5:doors, 6:hood, 7:trunk
};

namespace Physics {

void ApplyImpulse(VehicleDamageState& veh, const Vec3& worldImpactPos, const Vec3& impulse) {
    Vec3 r = worldImpactPos - veh.position;  // lever arm from CoM

    // Linear momentum change
    veh.linearVel = veh.linearVel + impulse * (1.0f / veh.mass);

    // Angular momentum change
    Vec3 angularImpulse = r.cross(impulse);
    veh.angularVel.x += angularImpulse.x / veh.inertiaTensor.x;
    veh.angularVel.y += angularImpulse.y / veh.inertiaTensor.y;
    veh.angularVel.z += angularImpulse.z / veh.inertiaTensor.z;
}

void ProcessDeformation(VehicleDamageState& veh, const Vec3& localImpactPos, float impactEnergy) {
    // Very approximate zone classification
    const float frontThreshold =  0.65f;
    const float rearThreshold  = -0.65f;
    const float sideThreshold  =  0.85f;

    float dmg = impactEnergy * 0.00042f;  // tuned empirically for ~100 km/h wall ≈ 40–60% front crush

    if (localImpactPos.y > frontThreshold) {
        veh.crushZones[0] = std::min(veh.crushZones[0] + dmg * 1.35f, 1.0f);
    }
    if (localImpactPos.y < rearThreshold) {
        veh.crushZones[1] = std::min(veh.crushZones[1] + dmg * 1.10f, 1.0f);
    }
    if (std::abs(localImpactPos.x) > sideThreshold) {
        int side = (localImpactPos.x > 0.0f) ? 3 : 2;
        veh.crushZones[side] = std::min(veh.crushZones[side] + dmg * 1.45f, 1.0f);
    }

    // Small chance of component failure on high-energy hits
    if (impactEnergy > 220000.0f && (rand() % 100) < 28) {
        int partIdx = rand() % 8;
        if (!veh.detached[partIdx]) {
            veh.detached[partIdx] = true;
            std::cout << "[Damage] Component " << partIdx << " detached\n";
        }
    }
}

void Integrate(VehicleDamageState& veh, float dt) {
    // Simple angular damping (prevents infinite spinning)
    veh.angularVel = veh.angularVel * 0.965f;

    // Position update
    veh.position = veh.position + veh.linearVel * dt;

    // Basic ground plane + air drag
    if (veh.position.z < 0.45f) veh.position.z = 0.45f;
    veh.linearVel = veh.linearVel * 0.991f;
    veh.linearVel.z -= 9.81f * dt * 0.35f;  // softened gravity
}

} // namespace Physics

int main() {
    std::srand(static_cast<unsigned>(std::time(nullptr)));

    VehicleDamageState car;
    car.position    = {0.0f, 0.0f, 0.75f};
    car.linearVel   = {0.0f, 31.0f, 0.8f};  // ~112 km/h with slight upward component

    std::cout << "Crash test started — initial speed: "
              << car.linearVel.length() * 3.6f << " km/h\n\n";

    const float wallX = 38.0f;
    const float dt = 1.0f / 240.0f;
    float simTime = 0.0f;

    while (simTime < 5.0f) {
        // Detect approximate front impact
        if (car.position.x > wallX - 2.4f && car.linearVel.x > 1.5f) {
            std::cout << "\n=== Wall impact detected @ " << simTime << " s ===\n";

            Vec3 localImpact {0.2f, 2.1f, 0.4f};           // front-right bumper area
            Vec3 worldImpact = car.position + localImpact;

            Vec3 wallNormal {-1.0f, 0.0f, 0.0f};

            float closingSpeed = car.linearVel.dot(wallNormal);
            if (closingSpeed < -0.4f) {
                float restitution = 0.16f;  // low — most energy absorbed
                float impulseMag = -(1.0f + restitution) * closingSpeed * car.mass * 0.78f;

                Vec3 impulse = wallNormal * impulseMag;

                Physics::ApplyImpulse(car, worldImpact, impulse);
                float approxKE = 0.5f * car.mass * car.linearVel.length() * car.linearVel.length();
                Physics::ProcessDeformation(car, localImpact, approxKE);

                std::cout << "  Front crush: " << (car.crushZones[0] * 100.0f) << "%\n";
                std::cout << "  Right crush: " << (car.crushZones[3] * 100.0f) << "%\n";
            }
        }

        Physics::Integrate(car, dt);
        simTime += dt;

        // Periodic status
        static float lastReport = -1.0f;
        if (simTime - lastReport > 0.4f) {
            std::cout << simTime << " s | pos: " << car.position.x << ", " << car.position.y
                      << " | speed: " << (car.linearVel.length() * 3.6f) << " km/h\n";
            lastReport = simTime;
        }
    }

    std::cout << "\nSimulation finished.\n";
    return 0;
}
