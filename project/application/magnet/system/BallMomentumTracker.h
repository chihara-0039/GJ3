#pragma once

#include "math/Struct.h"

#include <array>
#include <cstddef>

namespace magnet {

// Remembers the strongest recent swing for each ball and converts it into
// a bounded launch velocity. The tracker is independent from PhysicsWorld so
// momentum tuning and tests do not leak into the chain simulation.
class BallMomentumTracker final {
public:
	static constexpr std::size_t kBallCount = 4;

	struct Settings {
		float attackSeconds = 0.045f;
		float decaySeconds = 0.32f;
		float boostStartSpeed = 2.0f;
		float fullBoostSpeed = 14.0f;
		float maximumSpeedBoost = 0.45f;
		float maximumLaunchSpeed = 22.0f;
	};

	BallMomentumTracker() = default;
	explicit BallMomentumTracker(const Settings& settings) noexcept;

	void Reset() noexcept;
	[[nodiscard]] bool Update(
		std::size_t ballIndex,
		const Vector3& currentVelocity,
		float deltaTime) noexcept;
	[[nodiscard]] Vector3 CalculateLaunchVelocity(
		std::size_t ballIndex,
		const Vector3& currentVelocity) const noexcept;
	[[nodiscard]] const Vector3& GetStoredVelocity(std::size_t ballIndex) const noexcept;

private:
	[[nodiscard]] static bool IsFinite(const Vector3& value) noexcept;
	[[nodiscard]] static float LengthSquaredXZ(const Vector3& value) noexcept;
	[[nodiscard]] static Vector3 ClampMagnitudeXZ(
		const Vector3& value,
		float maximumMagnitude) noexcept;

	Settings settings_{};
	std::array<Vector3, kBallCount> storedVelocities_{};
};

} // namespace magnet
