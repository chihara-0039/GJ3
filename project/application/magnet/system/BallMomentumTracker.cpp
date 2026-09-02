#include "application/magnet/system/BallMomentumTracker.h"

#include <algorithm>
#include <cmath>

namespace magnet {
namespace {

constexpr float kDirectionEpsilonSquared = 1.0e-8f;
constexpr std::array<float, BallMomentumTracker::kBallCount> kMemoryBlend = {
	0.92f, 0.78f, 0.62f, 0.48f,
};
const Vector3 kZeroVelocity{};

} // namespace

BallMomentumTracker::BallMomentumTracker(const Settings& settings) noexcept
	: settings_(settings)
{
}

void BallMomentumTracker::Reset() noexcept
{
	storedVelocities_.fill(Vector3{});
}

bool BallMomentumTracker::Update(
	std::size_t ballIndex,
	const Vector3& currentVelocity,
	float deltaTime) noexcept
{
	if (ballIndex >= storedVelocities_.size() || !IsFinite(currentVelocity) ||
		!std::isfinite(deltaTime) || deltaTime <= 0.0f) {
		return false;
	}

	Vector3 planarVelocity = currentVelocity;
	planarVelocity.y = 0.0f;
	Vector3& storedVelocity = storedVelocities_[ballIndex];
	const float currentSpeedSquared = LengthSquaredXZ(planarVelocity);
	const float storedSpeedSquared = LengthSquaredXZ(storedVelocity);
	const float alignment =
		planarVelocity.x * storedVelocity.x + planarVelocity.z * storedVelocity.z;
	const bool reinforcing = currentSpeedSquared >= storedSpeedSquared && alignment >= 0.0f;
	const float timeConstant = reinforcing
		? (std::max)(settings_.attackSeconds, 1.0e-4f)
		: (std::max)(settings_.decaySeconds, 1.0e-4f);
	const float blend = 1.0f - std::exp(-deltaTime / timeConstant);
	storedVelocity += (planarVelocity - storedVelocity) * blend;
	storedVelocity.y = 0.0f;
	return IsFinite(storedVelocity);
}

Vector3 BallMomentumTracker::CalculateLaunchVelocity(
	std::size_t ballIndex,
	const Vector3& currentVelocity) const noexcept
{
	if (ballIndex >= storedVelocities_.size() || !IsFinite(currentVelocity)) {
		return {};
	}

	Vector3 launchVelocity = currentVelocity;
	launchVelocity.y = 0.0f;
	const Vector3& storedVelocity = storedVelocities_[ballIndex];
	if (LengthSquaredXZ(storedVelocity) > LengthSquaredXZ(launchVelocity)) {
		launchVelocity +=
			(storedVelocity - launchVelocity) * kMemoryBlend[ballIndex];
	}

	const float speedSquared = LengthSquaredXZ(launchVelocity);
	if (!std::isfinite(speedSquared) || speedSquared <= kDirectionEpsilonSquared) {
		return {};
	}
	const float speed = std::sqrt(speedSquared);
	const float boostRange = (std::max)(
		settings_.fullBoostSpeed - settings_.boostStartSpeed,
		1.0e-4f);
	const float normalizedMomentum = std::clamp(
		(speed - settings_.boostStartSpeed) / boostRange,
		0.0f,
		1.0f);
	const float speedBoost = 1.0f +
		settings_.maximumSpeedBoost * normalizedMomentum * normalizedMomentum;
	return ClampMagnitudeXZ(
		launchVelocity * speedBoost,
		settings_.maximumLaunchSpeed);
}

const Vector3& BallMomentumTracker::GetStoredVelocity(std::size_t ballIndex) const noexcept
{
	return ballIndex < storedVelocities_.size()
		? storedVelocities_[ballIndex]
		: kZeroVelocity;
}

bool BallMomentumTracker::IsFinite(const Vector3& value) noexcept
{
	return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

float BallMomentumTracker::LengthSquaredXZ(const Vector3& value) noexcept
{
	return value.x * value.x + value.z * value.z;
}

Vector3 BallMomentumTracker::ClampMagnitudeXZ(
	const Vector3& value,
	float maximumMagnitude) noexcept
{
	const float lengthSquared = LengthSquaredXZ(value);
	const float safeMaximum = (std::max)(maximumMagnitude, 0.0f);
	if (!std::isfinite(lengthSquared) || lengthSquared <= kDirectionEpsilonSquared) {
		return {};
	}
	if (lengthSquared <= safeMaximum * safeMaximum) {
		return value;
	}
	return value * (safeMaximum / std::sqrt(lengthSquared));
}

} // namespace magnet
