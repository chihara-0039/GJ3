#include "application/magnet/system/MagnetChainSystem.h"
#include "application/magnet/system/BallMomentumTracker.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>

namespace {

constexpr float kFixedDeltaTime = 1.0f / 60.0f;
constexpr int kDriveStepCount = 120;
constexpr int kStopObservationStepCount = 180;
constexpr int kTurnObservationStepCount = 90;
constexpr int kAlternatingTurnStepCount = 240;
constexpr int kReleaseTravelStepCount = 30;
constexpr float kMaximumAllowedConstraintError = 0.08f;
constexpr float kMaximumAllowedJointBendRadians = 1.40f;
constexpr float kMinimumVisibleJointBendRadians = 0.10f;
constexpr float kMinimumRootSideProjection = 0.35f;
constexpr float kMinimumRootReleaseTravel = 0.50f;
constexpr float kMaximumReleaseConvergenceCorrectionRadians = 0.21f;
constexpr float kMaximumReleaseConvergenceSpreadRatio = 0.95f;
constexpr float kFirstHorizontalOffset = 1.2247449f;
constexpr float kLinkLength = 1.0f;

bool IsFinite(const Vector3& value)
{
	return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

float Distance(const Vector3& a, const Vector3& b)
{
	const Vector3 delta = b - a;
	return std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
}

float DistanceXZ(const Vector3& a, const Vector3& b)
{
	const float deltaX = b.x - a.x;
	const float deltaZ = b.z - a.z;
	return std::sqrt(deltaX * deltaX + deltaZ * deltaZ);
}

Vector3 NormalizeXZ(const Vector3& value)
{
	const float length = DistanceXZ(Vector3{}, value);
	if (!std::isfinite(length) || length <= 1.0e-5f) {
		return {};
	}
	return value * (1.0f / length);
}

float DotXZ(const Vector3& a, const Vector3& b)
{
	return a.x * b.x + a.z * b.z;
}

float AngleBetweenXZ(const Vector3& a, const Vector3& b)
{
	const Vector3 normalizedA = NormalizeXZ(a);
	const Vector3 normalizedB = NormalizeXZ(b);
	if (DistanceXZ(Vector3{}, normalizedA) <= 1.0e-5f ||
		DistanceXZ(Vector3{}, normalizedB) <= 1.0e-5f) {
		return INFINITY;
	}
	return std::acos(std::clamp(DotXZ(normalizedA, normalizedB), -1.0f, 1.0f));
}

bool ValidateFiniteBodies(const magnet::MagnetChainSystem& system)
{
	const physics::PhysicsWorld& world = system.GetPhysicsWorld();
	const physics::SphereBody* player = world.GetBody(system.GetPlayerBody());
	if (!player || !IsFinite(player->position) || !IsFinite(player->linearVelocity)) {
		return false;
	}
	for (physics::BodyHandle handle : system.GetLeftChain()) {
		const physics::SphereBody* body = world.GetBody(handle);
		if (!body || !IsFinite(body->position) || !IsFinite(body->linearVelocity)) {
			return false;
		}
	}
	for (physics::BodyHandle handle : system.GetRightChain()) {
		const physics::SphereBody* body = world.GetBody(handle);
		if (!body || !IsFinite(body->position) || !IsFinite(body->linearVelocity)) {
			return false;
		}
	}
	for (physics::BodyHandle handle : system.GetTestBalls()) {
		const physics::SphereBody* body = world.GetBody(handle);
		if (!body || !IsFinite(body->position) || !IsFinite(body->linearVelocity)) {
			return false;
		}
	}
	return true;
}

float GetMaximumConstraintError(const magnet::MagnetChainSystem& system)
{
	const physics::PhysicsWorld& world = system.GetPhysicsWorld();
	float maximumError = 0.0f;
	for (const physics::DistanceConstraint& constraint : world.GetConstraints()) {
		if (!constraint.active) {
			continue;
		}
		const physics::SphereBody* bodyA = world.GetBody(constraint.bodyA);
		const physics::SphereBody* bodyB = world.GetBody(constraint.bodyB);
		if (!bodyA || !bodyB) {
			return INFINITY;
		}
		maximumError = (std::max)(
			maximumError,
			std::abs(Distance(bodyA->position, bodyB->position) - constraint.restLength));
	}
	return maximumError;
}

} // namespace

int main()
{
	magnet::BallMomentumTracker momentumTracker;
	for (int step = 0; step < 30; ++step) {
		if (!momentumTracker.Update(0, Vector3{ 0.0f, 0.0f, 4.0f }, kFixedDeltaTime)) {
			std::cerr << "Low-speed momentum tracking failed.\n";
			return 38;
		}
	}
	const Vector3 lowMomentumLaunch =
		momentumTracker.CalculateLaunchVelocity(0, Vector3{ 0.0f, 0.0f, 4.0f });
	momentumTracker.Reset();
	for (int step = 0; step < 30; ++step) {
		if (!momentumTracker.Update(0, Vector3{ 0.0f, 0.0f, 14.0f }, kFixedDeltaTime)) {
			std::cerr << "High-speed momentum tracking failed.\n";
			return 39;
		}
	}
	const Vector3 highMomentumLaunch =
		momentumTracker.CalculateLaunchVelocity(0, Vector3{ 0.0f, 0.0f, 14.0f });
	const float lowMomentumSpeed = DistanceXZ(Vector3{}, lowMomentumLaunch);
	const float highMomentumSpeed = DistanceXZ(Vector3{}, highMomentumLaunch);
	if (!std::isfinite(lowMomentumSpeed) || !std::isfinite(highMomentumSpeed) ||
		highMomentumSpeed <= lowMomentumSpeed * 3.5f || highMomentumSpeed > 22.001f) {
		std::cerr << "Momentum did not produce a bounded launch-speed increase.\n";
		return 40;
	}

	magnet::MagnetChainSystem system;
	if (!system.Initialize()) {
		std::cerr << "Initialize failed.\n";
		return 1;
	}
	if (system.GetPhysicsWorld().GetBodyCount() !=
			1 + magnet::MagnetChainSystem::kLinksPerSide * 2 +
			magnet::MagnetChainSystem::kTestBallCapacity ||
		system.GetPhysicsWorld().GetConstraints().size() != 14) {
		std::cerr << "Unexpected prototype topology.\n";
		return 2;
	}
	magnet::MagnetChainSystem::EmitterSettings emitterSettings{};
	emitterSettings.autoEmit = true;
	emitterSettings.intervalSeconds = 0.05f;
	emitterSettings.launchSpeed = 20.0f;
	system.SetEmitterSettings(emitterSettings);
	for (int step = 0; step < 120; ++step) {
		system.SetPlayerCommand({});
		if (!system.FixedUpdate(kFixedDeltaTime) || !ValidateFiniteBodies(system)) {
			std::cerr << "Emitter simulation became invalid at step " << step << ".\n";
			return 10;
		}
	}
	if (system.GetActiveTestBallCount() == 0 ||
		system.GetActiveTestBallCount() > magnet::MagnetChainSystem::kTestBallCapacity) {
		std::cerr << "Emitter Pool count is invalid.\n";
		return 11;
	}
	if (!system.Reset()) {
		std::cerr << "Reset after emitter test failed.\n";
		return 12;
	}

	magnet::MagnetChainSystem::PlayerCommand command{};
	command.moveDirection = { 0.0f, 0.0f, 1.0f };
	for (int step = 0; step < kDriveStepCount; ++step) {
		system.SetPlayerCommand(command);
		if (!system.FixedUpdate(kFixedDeltaTime) || !ValidateFiniteBodies(system)) {
			std::cerr << "Drive simulation became invalid at step " << step << ".\n";
			return 3;
		}
	}

	command = {};
	command.emergencyStop = true;
	float maximumInnerForwardOffset = 0.0f;
	float maximumOuterForwardOffset = 0.0f;
	float maximumConstraintError = 0.0f;
	for (int step = 0; step < kStopObservationStepCount; ++step) {
		system.SetPlayerCommand(command);
		if (!system.FixedUpdate(kFixedDeltaTime) || !ValidateFiniteBodies(system)) {
			std::cerr << "Stop simulation became invalid at step " << step << ".\n";
			return 4;
		}
		command.emergencyStop = false;

		const physics::PhysicsWorld& world = system.GetPhysicsWorld();
		const physics::SphereBody* player = world.GetBody(system.GetPlayerBody());
		const physics::SphereBody* inner = world.GetBody(system.GetRightChain().front());
		const physics::SphereBody* outer = world.GetBody(system.GetRightChain().back());
		if (!player || !inner || !outer) {
			std::cerr << "Body handle became stale.\n";
			return 5;
		}
		maximumInnerForwardOffset = (std::max)(
			maximumInnerForwardOffset, inner->position.z - player->position.z);
		maximumOuterForwardOffset = (std::max)(
			maximumOuterForwardOffset, outer->position.z - player->position.z);
		maximumConstraintError = (std::max)(maximumConstraintError, GetMaximumConstraintError(system));
	}

	std::cout << "inner_forward_offset=" << maximumInnerForwardOffset << '\n';
	std::cout << "outer_forward_offset=" << maximumOuterForwardOffset << '\n';
	std::cout << "maximum_constraint_error=" << maximumConstraintError << '\n';
	if (!(maximumOuterForwardOffset > maximumInnerForwardOffset) ||
		maximumOuterForwardOffset <= 0.0f) {
		std::cerr << "Outer link did not produce the expected stronger forward reaction.\n";
		return 6;
	}
	if (!std::isfinite(maximumConstraintError) ||
		maximumConstraintError > kMaximumAllowedConstraintError) {
		std::cerr << "Constraint error exceeded the prototype bound.\n";
		return 7;
	}

	if (!system.Reset()) {
		std::cerr << "Reset before turn test failed.\n";
		return 13;
	}
	command = {};
	command.moveDirection = { 0.0f, 0.0f, 1.0f };
	for (int step = 0; step < 60; ++step) {
		system.SetPlayerCommand(command);
		if (!system.FixedUpdate(kFixedDeltaTime) || !ValidateFiniteBodies(system)) {
			std::cerr << "Turn setup became invalid at step " << step << ".\n";
			return 14;
		}
	}
	command.moveDirection = { 1.0f, 0.0f, 0.0f };
	float maximumInnerPoseError = 0.0f;
	float maximumOuterPoseError = 0.0f;
	for (int step = 0; step < kTurnObservationStepCount; ++step) {
		system.SetPlayerCommand(command);
		if (!system.FixedUpdate(kFixedDeltaTime) || !ValidateFiniteBodies(system)) {
			std::cerr << "Turn simulation became invalid at step " << step << ".\n";
			return 15;
		}
		const physics::PhysicsWorld& world = system.GetPhysicsWorld();
		const physics::SphereBody* player = world.GetBody(system.GetPlayerBody());
		const physics::SphereBody* inner = world.GetBody(system.GetRightChain().front());
		const physics::SphereBody* outer = world.GetBody(system.GetRightChain().back());
		if (!player || !inner || !outer) {
			std::cerr << "Turn test body handle became stale.\n";
			return 16;
		}
		const float heading = system.GetPlayerHeadingRadians();
		const Vector3 rightAxis = { std::cos(heading), 0.0f, -std::sin(heading) };
		const Vector3 innerTarget = player->position + rightAxis * kFirstHorizontalOffset;
		const Vector3 outerTarget = player->position + rightAxis *
			(kFirstHorizontalOffset + kLinkLength *
				static_cast<float>(magnet::MagnetChainSystem::kLinksPerSide - 1));
		maximumInnerPoseError = (std::max)(
			maximumInnerPoseError, DistanceXZ(inner->position, innerTarget));
		maximumOuterPoseError = (std::max)(
			maximumOuterPoseError, DistanceXZ(outer->position, outerTarget));
	}
	std::cout << "inner_turn_pose_error=" << maximumInnerPoseError << '\n';
	std::cout << "outer_turn_pose_error=" << maximumOuterPoseError << '\n';
	if (!(maximumOuterPoseError > maximumInnerPoseError) ||
		maximumInnerPoseError <= 0.0f) {
		std::cerr << "Outer link did not show the expected larger elastic turn lag.\n";
		return 17;
	}

	float maximumObservedJointBend = 0.0f;
	float minimumObservedRootProjection = 1.0f;
	for (int step = 0; step < kAlternatingTurnStepCount; ++step) {
		command = {};
		command.moveDirection = ((step / 30) % 2 == 0)
			? Vector3{ 1.0f, 0.0f, 0.0f }
			: Vector3{ -1.0f, 0.0f, 0.0f };
		system.SetPlayerCommand(command);
		if (!system.FixedUpdate(kFixedDeltaTime) || !ValidateFiniteBodies(system)) {
			std::cerr << "Alternating turn simulation became invalid at step " << step << ".\n";
			return 27;
		}
		const physics::PhysicsWorld& world = system.GetPhysicsWorld();
		const physics::SphereBody* player = world.GetBody(system.GetPlayerBody());
		if (!player) {
			std::cerr << "Player missing during alternating turn test.\n";
			return 28;
		}
		const float heading = system.GetPlayerHeadingRadians();
		const Vector3 rightAxis = { std::cos(heading), 0.0f, -std::sin(heading) };
		const auto measureChain = [&](float sideSign, const auto& chain) {
			Vector3 parentPosition = player->position;
			Vector3 previousSegment = rightAxis * sideSign;
			for (std::size_t linkIndex = 0; linkIndex < chain.size(); ++linkIndex) {
				const physics::SphereBody* body = world.GetBody(chain[linkIndex]);
				if (!body) {
					return false;
				}
				const Vector3 segment = body->position - parentPosition;
				if (linkIndex == 0) {
					minimumObservedRootProjection = (std::min)(
						minimumObservedRootProjection,
						DotXZ(NormalizeXZ(segment), previousSegment));
				} else {
					maximumObservedJointBend = (std::max)(
						maximumObservedJointBend,
						AngleBetweenXZ(previousSegment, segment));
				}
				previousSegment = segment;
				parentPosition = body->position;
			}
			return true;
		};
		if (!measureChain(-1.0f, system.GetLeftChain()) ||
			!measureChain(1.0f, system.GetRightChain())) {
			std::cerr << "Chain geometry missing during alternating turn test.\n";
			return 29;
		}
	}
	std::cout << "maximum_joint_bend_radians=" << maximumObservedJointBend << '\n';
	std::cout << "minimum_root_side_projection=" << minimumObservedRootProjection << '\n';
	if (!std::isfinite(maximumObservedJointBend) ||
		maximumObservedJointBend > kMaximumAllowedJointBendRadians) {
		std::cerr << "Chain exceeded the bounded bend angle and may orbit the Player.\n";
		return 30;
	}
	if (maximumObservedJointBend < kMinimumVisibleJointBendRadians) {
		std::cerr << "Chain became too rigid to show visible bending.\n";
		return 31;
	}
	if (!std::isfinite(minimumObservedRootProjection) ||
		minimumObservedRootProjection < kMinimumRootSideProjection) {
		std::cerr << "A root link crossed behind the Player side axis.\n";
		return 32;
	}

	command = {};
	command.emergencyStop = true;
	for (int step = 0; step < 6; ++step) {
		system.SetPlayerCommand(command);
		if (!system.FixedUpdate(kFixedDeltaTime) || !ValidateFiniteBodies(system)) {
			std::cerr << "Release setup became invalid at step " << step << ".\n";
			return 18;
		}
		command.emergencyStop = false;
	}
	const physics::PhysicsWorld& worldBeforeRelease = system.GetPhysicsWorld();
	const physics::SphereBody* playerBeforeRelease =
		worldBeforeRelease.GetBody(system.GetPlayerBody());
	if (!playerBeforeRelease) {
		std::cerr << "Player missing before release.\n";
		return 19;
	}
	const Vector3 playerReleasePosition = playerBeforeRelease->position;
	std::array<Vector3, magnet::MagnetChainSystem::kLinksPerSide * 2> releasePositions{};
	for (std::size_t index = 0; index < magnet::MagnetChainSystem::kLinksPerSide; ++index) {
		const physics::SphereBody* left = worldBeforeRelease.GetBody(system.GetLeftChain()[index]);
		const physics::SphereBody* right = worldBeforeRelease.GetBody(system.GetRightChain()[index]);
		if (!left || !right) {
			std::cerr << "Chain body missing before release.\n";
			return 20;
		}
		releasePositions[index] = left->position;
		releasePositions[index + magnet::MagnetChainSystem::kLinksPerSide] = right->position;
	}

	command = {};
	command.emergencyStop = true;
	command.releaseChains = true;
	system.SetPlayerCommand(command);
	if (!system.FixedUpdate(kFixedDeltaTime) || !ValidateFiniteBodies(system)) {
		std::cerr << "Release step became invalid.\n";
		return 21;
	}
	if (system.AreChainsAttached() ||
		system.GetPhysicsWorld().GetActiveConstraintCount() != 0) {
		std::cerr << "MAGNET OFF did not deactivate every chain constraint.\n";
		return 22;
	}
	const magnet::MagnetChainSystem::ReleaseConvergenceDiagnostics& convergenceDiagnostics =
		system.GetLastReleaseConvergenceDiagnostics();
	std::cout << "release_predicted_rms_spread_before="
		<< convergenceDiagnostics.predictedRmsSpreadBefore << '\n';
	std::cout << "release_predicted_rms_spread_after="
		<< convergenceDiagnostics.predictedRmsSpreadAfter << '\n';
	std::cout << "release_maximum_direction_correction_radians="
		<< convergenceDiagnostics.maximumDirectionCorrectionRadians << '\n';
	std::cout << "release_focus=" << convergenceDiagnostics.focusPoint.x << ','
		<< convergenceDiagnostics.focusPoint.z << '\n';
	if (!convergenceDiagnostics.valid || !convergenceDiagnostics.applied ||
		!IsFinite(convergenceDiagnostics.focusPoint) ||
		!std::isfinite(convergenceDiagnostics.predictedRmsSpreadBefore) ||
		!std::isfinite(convergenceDiagnostics.predictedRmsSpreadAfter) ||
		convergenceDiagnostics.predictedRmsSpreadAfter >=
			convergenceDiagnostics.predictedRmsSpreadBefore) {
		std::cerr << "Release convergence assist did not reduce predicted trajectory spread.\n";
		return 34;
	}
	if (!std::isfinite(convergenceDiagnostics.maximumDirectionCorrectionRadians) ||
		convergenceDiagnostics.maximumDirectionCorrectionRadians >
			kMaximumReleaseConvergenceCorrectionRadians) {
		std::cerr << "Release convergence assist exceeded its direction-correction bound.\n";
		return 35;
	}
	if (convergenceDiagnostics.predictedRmsSpreadAfter >
		convergenceDiagnostics.predictedRmsSpreadBefore *
			kMaximumReleaseConvergenceSpreadRatio) {
		std::cerr << "Release convergence assist was too weak to be meaningful.\n";
		return 36;
	}
	command = {};
	for (int step = 0; step < kReleaseTravelStepCount; ++step) {
		system.SetPlayerCommand(command);
		if (!system.FixedUpdate(kFixedDeltaTime) || !ValidateFiniteBodies(system)) {
			std::cerr << "Released-chain simulation became invalid at step " << step << ".\n";
			return 23;
		}
	}
	const physics::PhysicsWorld& worldAfterRelease = system.GetPhysicsWorld();
	const physics::SphereBody* playerAfterRelease =
		worldAfterRelease.GetBody(system.GetPlayerBody());
	if (!playerAfterRelease ||
		DistanceXZ(playerReleasePosition, playerAfterRelease->position) > 1.0e-4f ||
		DistanceXZ(Vector3{}, playerAfterRelease->linearVelocity) > 1.0e-4f) {
		std::cerr << "Player moved because of chain release.\n";
		return 24;
	}
	float releasedChainTravel = 0.0f;
	float releasedRootTravel = 0.0f;
	float releasedOuterTravel = 0.0f;
	for (std::size_t index = 0; index < magnet::MagnetChainSystem::kLinksPerSide; ++index) {
		const physics::SphereBody* left = worldAfterRelease.GetBody(system.GetLeftChain()[index]);
		const physics::SphereBody* right = worldAfterRelease.GetBody(system.GetRightChain()[index]);
		if (!left || !right) {
			std::cerr << "Released chain body handle became stale.\n";
			return 25;
		}
		const float leftTravel = DistanceXZ(releasePositions[index], left->position);
		const float rightTravel = DistanceXZ(
			releasePositions[index + magnet::MagnetChainSystem::kLinksPerSide],
			right->position);
		releasedChainTravel += leftTravel + rightTravel;
		if (index == 0) {
			releasedRootTravel = leftTravel + rightTravel;
		} else if (index + 1 == magnet::MagnetChainSystem::kLinksPerSide) {
			releasedOuterTravel = leftTravel + rightTravel;
		}
	}
	std::cout << "released_chain_total_travel=" << releasedChainTravel << '\n';
	std::cout << "released_root_total_travel=" << releasedRootTravel << '\n';
	std::cout << "released_outer_total_travel=" << releasedOuterTravel << '\n';
	if (releasedChainTravel <= 0.25f) {
		std::cerr << "Released chains did not preserve meaningful inertia.\n";
		return 26;
	}
	if (!std::isfinite(releasedRootTravel) ||
		releasedRootTravel < kMinimumRootReleaseTravel) {
		std::cerr << "Root links still lost too much inertia before release.\n";
		return 33;
	}

	if (!system.Reset() || !ValidateFiniteBodies(system)) {
		std::cerr << "Reset failed.\n";
		return 8;
	}
	if (system.GetLastReleaseConvergenceDiagnostics().valid) {
		std::cerr << "Reset retained stale release-convergence diagnostics.\n";
		return 37;
	}
	const physics::SphereBody* resetPlayer =
		system.GetPhysicsWorld().GetBody(system.GetPlayerBody());
	if (!resetPlayer || std::abs(resetPlayer->position.x) > 1.0e-5f ||
		std::abs(resetPlayer->position.z) > 1.0e-5f) {
		std::cerr << "Reset did not restore the player origin.\n";
		return 9;
	}

	std::cout << "MAGNET_CHAIN_PHYSICS_TESTS_OK\n";
	return 0;
}
