#pragma once

#include "application/magnet/system/BallMomentumTracker.h"
#include "physics/PhysicsWorld.h"

#include <array>
#include <cstddef>
#include <limits>

namespace magnet {

// Owns the prototype player command and the left/right chain topology.
// Rendering receives read-only body state from PhysicsWorld.
class MagnetChainSystem final {
public:
	static constexpr std::size_t kLinksPerSide = 4;
	static constexpr std::size_t kTestBallCapacity = 24;

	struct EmitterSettings {
		bool autoEmit = false;
		float intervalSeconds = 0.35f;
		float launchSpeed = 8.0f;
	};

	struct PlayerCommand {
		Vector3 moveDirection{};
		bool emergencyStop = false;
		bool emitOne = false;
		bool releaseChains = false;
	};

	struct ReleaseConvergenceDiagnostics {
		Vector3 focusPoint{};
		float predictedRmsSpreadBefore = 0.0f;
		float predictedRmsSpreadAfter = 0.0f;
		float maximumDirectionCorrectionRadians = 0.0f;
		bool applied = false;
		bool valid = false;
	};

	[[nodiscard]] bool Initialize();
	[[nodiscard]] bool Reset();
	void SetPlayerCommand(const PlayerCommand& command) noexcept { command_ = command; }
	void SetEmitterSettings(const EmitterSettings& settings) noexcept;
	[[nodiscard]] bool FixedUpdate(float fixedDeltaTime) noexcept;

	[[nodiscard]] const physics::PhysicsWorld& GetPhysicsWorld() const noexcept { return physicsWorld_; }
	[[nodiscard]] physics::BodyHandle GetPlayerBody() const noexcept { return playerBody_; }
	[[nodiscard]] const std::array<physics::BodyHandle, kLinksPerSide>& GetLeftChain() const noexcept { return leftChain_; }
	[[nodiscard]] const std::array<physics::BodyHandle, kLinksPerSide>& GetRightChain() const noexcept { return rightChain_; }
	[[nodiscard]] const std::array<physics::BodyHandle, kTestBallCapacity>& GetTestBalls() const noexcept { return testBalls_; }
	[[nodiscard]] std::size_t GetActiveTestBallCount() const noexcept;
	[[nodiscard]] float GetMaximumConstraintError() const noexcept;
	[[nodiscard]] float GetPlayerHeadingRadians() const noexcept { return playerHeadingRadians_; }
	[[nodiscard]] bool AreChainsAttached() const noexcept { return chainsAttached_; }
	[[nodiscard]] bool IsHealthy() const noexcept { return healthy_; }
	[[nodiscard]] const ReleaseConvergenceDiagnostics&
		GetLastReleaseConvergenceDiagnostics() const noexcept {
		return lastReleaseConvergenceDiagnostics_;
	}

private:
	static constexpr std::size_t kBendConstraintsPerSide = kLinksPerSide - 1;
	static constexpr std::size_t kInvalidConstraintIndex =
		(std::numeric_limits<std::size_t>::max)();

	[[nodiscard]] bool CreateChain(
		float sideSign,
		std::array<physics::BodyHandle, kLinksPerSide>& outputChain,
		std::array<std::size_t, kLinksPerSide>& outputConstraintIndices,
		std::array<std::size_t, kBendConstraintsPerSide>& outputBendConstraintIndices);
	[[nodiscard]] bool CreateTestBallPool();
	[[nodiscard]] bool EmitTestBall() noexcept;
	[[nodiscard]] bool ApplyMagneticRestoringForces(float fixedDeltaTime) noexcept;
	[[nodiscard]] bool UpdateMomentumTrackers(float fixedDeltaTime) noexcept;
	[[nodiscard]] bool ApplyMomentumLaunch() noexcept;
	[[nodiscard]] bool ReleaseChains() noexcept;
	void DeactivateDistantTestBalls() noexcept;

	physics::PhysicsWorld physicsWorld_;
	physics::BodyHandle playerBody_{};
	std::array<physics::BodyHandle, kLinksPerSide> leftChain_{};
	std::array<physics::BodyHandle, kLinksPerSide> rightChain_{};
	std::array<std::size_t, kLinksPerSide> leftConstraintIndices_{};
	std::array<std::size_t, kLinksPerSide> rightConstraintIndices_{};
	std::array<std::size_t, kBendConstraintsPerSide> leftBendConstraintIndices_{};
	std::array<std::size_t, kBendConstraintsPerSide> rightBendConstraintIndices_{};
	BallMomentumTracker leftMomentumTracker_{};
	BallMomentumTracker rightMomentumTracker_{};
	std::array<physics::BodyHandle, kTestBallCapacity> testBalls_{};
	PlayerCommand command_{};
	EmitterSettings emitterSettings_{};
	ReleaseConvergenceDiagnostics lastReleaseConvergenceDiagnostics_{};
	Vector3 playerVelocity_{};
	float playerHeadingRadians_ = 0.0f;
	float emitterTimer_ = 0.0f;
	std::size_t nextTestBallIndex_ = 0;
	uint32_t emittedBallSequence_ = 0;
	bool chainsAttached_ = true;
	bool healthy_ = false;
};

} // namespace magnet
