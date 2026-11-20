#include "Go2PawnSimApi.h"

#include "AirBlueprintLib.h"
#include "Components/BoxComponent.h"
#include "Components/SceneComponent.h"
#include "Engine/OverlapResult.h"
#include "Engine/World.h"
#include "GameFramework/Pawn.h"
#include "UObject/UnrealType.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace
{
    template <size_t Count>
    void validate(const std::vector<float>& values, const char* description)
    {
        if (values.size() != Count ||
            !std::all_of(values.begin(), values.end(), [](float value) { return std::isfinite(value); }))
            throw std::invalid_argument(description);
    }
}

Go2PawnSimApi::Go2PawnSimApi(const Params& params)
    : PawnSimApi(params)
{
}

void Go2PawnSimApi::alignSpawnToGround(APawn* pawn)
{
    // The MuJoCo floor maps to the Blueprint root, which is at foot level.
    // Align it with the UE floor before recording the spawn and reset pose.
    const std::string name(TCHAR_TO_UTF8(*pawn->GetName()));
    const auto& vehicles = msr::airlib::AirSimSettings::singleton().vehicles;
    const auto setting = vehicles.find(name);
    if (setting != vehicles.end() && setting->second->vehicle_type == msr::airlib::AirSimSettings::kVehicleTypeGo2 &&
        std::isnan(setting->second->position.z())) {
        const FVector location = pawn->GetActorLocation();
        const FCollisionQueryParams query(SCENE_QUERY_STAT(Go2SpawnGround), false, pawn);
        FHitResult floor_hit;
        if (pawn->GetWorld()->LineTraceSingleByObjectType(
                floor_hit, location + FVector(0, 0, 5), location - FVector(0, 0, 5000),
                FCollisionObjectQueryParams(ECC_WorldStatic), query) &&
            floor_hit.ImpactNormal.Z > 0.5) {
            pawn->SetActorLocation(FVector(location.X, location.Y, floor_hit.ImpactPoint.Z),
                                   false, nullptr, ETeleportType::TeleportPhysics);
            UE_LOG(LogTemp, Display, TEXT("Go2 spawn aligned to floor: Z %.2f -> %.2f cm"),
                   location.Z, floor_hit.ImpactPoint.Z);
        }
        else {
            UE_LOG(LogTemp, Warning, TEXT("Go2 did not find static ground beneath its spawn point"));
        }
    }
}

void Go2PawnSimApi::initialize()
{
    APawn* pawn = getPawn();
    const TCHAR* leg_names[] = {TEXT("FL"), TEXT("FR"), TEXT("RL"), TEXT("RR")};
    const TCHAR* joint_names[] = {TEXT("HipJoint"), TEXT("ThighJoint"), TEXT("CalfJoint")};
    for (size_t leg = 0; leg < 4; ++leg) {
        for (size_t joint = 0; joint < 3; ++joint) {
            const size_t index = leg * 3 + joint;
            const FString name = FString::Printf(TEXT("%s_%s"), leg_names[leg], joint_names[joint]);
            const auto* property = FindFProperty<FObjectPropertyBase>(pawn->GetClass(), FName(*name));
            if (property != nullptr)
                joints_[index] = Cast<USceneComponent>(property->GetObjectPropertyValue_InContainer(pawn));
            if (joints_[index] == nullptr)
                throw std::invalid_argument(std::string("Go2 Blueprint is missing joint ") + TCHAR_TO_UTF8(*name));
            joint_rest_rotations_[index] = joints_[index]->GetRelativeRotation().Quaternion();
        }
    }

    spawn_transform_ = pawn->GetActorTransform();
    // The Blueprint origin is at foot level. Its four hip pivots locate the torso.
    FVector hip_center = FVector::ZeroVector;
    for (size_t leg = 0; leg < 4; ++leg)
        hip_center += spawn_transform_.InverseTransformPositionNoScale(joints_[leg * 3]->GetComponentLocation());
    body_box_center_ = hip_center / 4.0 + FVector(7.5, 0, 0);
    PawnSimApi::initialize();
    vehicle_api_ = std::make_unique<msr::airlib::Go2Api>(getGroundTruthEnvironment()->getHomeGeoPoint());
}

void Go2PawnSimApi::resetImplementation()
{
    PawnSimApi::resetImplementation();
    const std::lock_guard<std::mutex> lock(state_mutex_);
    state_ = State{};
    pose_pending_ = false;
    applyState(state_, true);
}

void Go2PawnSimApi::updateRenderedState(float dt)
{
    State state;
    bool apply_pose;
    {
        const std::lock_guard<std::mutex> lock(state_mutex_);
        state = state_;
        apply_pose = pose_pending_;
        pose_pending_ = false;
    }
    applyState(state, apply_pose);
    PawnSimApi::update(dt);
    PawnSimApi::updateRenderedState(dt);
}

void Go2PawnSimApi::applyState(const State& state, bool apply_pose)
{
    if (apply_pose) {
        const FVector position = spawn_transform_.TransformPositionNoScale(state.position);
        const FQuat rotation = spawn_transform_.GetRotation() * state.rotation;
        getPawn()->SetActorLocationAndRotation(position, rotation, false, nullptr, ETeleportType::TeleportPhysics);
    }
    for (size_t index = 0; index < joints_.size(); ++index) {
        // Reflect MuJoCo's left-positive Y into UE's right-positive Y.
        const bool is_hip = index % 3 == 0;
        const FVector axis = is_hip ? FVector::ForwardVector : FVector::RightVector;
        const float angle = is_hip ? -state.angles[index] : state.angles[index];
        joints_[index]->SetRelativeRotation(joint_rest_rotations_[index] * FQuat(axis, angle));
    }
}

msr::airlib::VehicleApiBase* Go2PawnSimApi::getVehicleApiBase() const
{
    return vehicle_api_.get();
}

void Go2PawnSimApi::setJointAngles(const std::vector<float>& angles)
{
    validate<12>(angles, "Go2 requires 12 finite joint angles in radians");
    const std::lock_guard<std::mutex> lock(state_mutex_);
    std::copy(angles.begin(), angles.end(), state_.angles.begin());
}

std::vector<float> Go2PawnSimApi::getJointAngles() const
{
    const std::lock_guard<std::mutex> lock(state_mutex_);
    return std::vector<float>(state_.angles.begin(), state_.angles.end());
}

void Go2PawnSimApi::setBaseOffset(const std::vector<float>& offset)
{
    validate<3>(offset, "Go2 requires 3 finite position offsets in meters");
    const std::lock_guard<std::mutex> lock(state_mutex_);
    state_.position = FVector(offset[0], -offset[1], offset[2]) * 100.0;
    pose_pending_ = true;
}

// Called on the game thread under state_mutex_. A queued move is newer than the rendered pose.
void Go2PawnSimApi::readCurrentPose()
{
    if (pose_pending_)
        return;
    const auto& actor = getPawn()->GetActorTransform();
    state_.position = spawn_transform_.InverseTransformPositionNoScale(actor.GetLocation());
    state_.rotation = spawn_transform_.GetRotation().Inverse() * actor.GetRotation();
}

std::vector<std::vector<float>> Go2PawnSimApi::getState()
{
    std::vector<std::vector<float>> result;
    UAirBlueprintLib::RunCommandOnGameThread([this, &result]() {
        const std::lock_guard<std::mutex> lock(state_mutex_);
        readCurrentPose();
        const FVector3f p(state_.position / 100.0);
        const FQuat4f q(state_.rotation);
        result = {{state_.angles.begin(), state_.angles.end()}, {p.X, -p.Y, p.Z}, {q.W, -q.X, q.Y, -q.Z}};
    }, true);
    return result;
}

Go2PawnSimApi::State Go2PawnSimApi::decodeState(const std::vector<float>& angles, const std::vector<float>& offset,
                                               const std::vector<float>& orientation)
{
    validate<12>(angles, "Go2 requires 12 finite joint angles in radians");
    validate<3>(offset, "Go2 requires 3 finite position offsets in meters");
    validate<4>(orientation, "Go2 requires a finite orientation quaternion in WXYZ order");
    FQuat rotation(-orientation[1], orientation[2], -orientation[3], orientation[0]);
    if (rotation.SizeSquared() < UE_SMALL_NUMBER)
        throw std::invalid_argument("Go2 orientation quaternion cannot be zero");
    rotation.Normalize();

    State target;
    std::copy(angles.begin(), angles.end(), target.angles.begin());
    target.position = FVector(offset[0], -offset[1], offset[2]) * 100.0;
    target.rotation = rotation;
    return target;
}

void Go2PawnSimApi::setState(const std::vector<float>& angles, const std::vector<float>& offset,
                            const std::vector<float>& orientation)
{
    const State target = decodeState(angles, offset, orientation);
    const std::lock_guard<std::mutex> lock(state_mutex_);
    state_ = target;
    pose_pending_ = true;
}

std::vector<std::vector<float>> Go2PawnSimApi::moveState(const std::vector<float>& angles, const std::vector<float>& offset,
                                                        const std::vector<float>& orientation)
{
    State target = decodeState(angles, offset, orientation);
    std::vector<std::vector<float>> feedback;
    UAirBlueprintLib::RunCommandOnGameThread([this, &target, &feedback]() {
        const std::lock_guard<std::mutex> lock(state_mutex_);
        readCurrentPose();
        feedback = sweepBody(target);
        state_ = target;
        pose_pending_ = true;
    }, true);
    return feedback;
}

std::vector<std::vector<float>> Go2PawnSimApi::sweepBody(State& target)
{
    const auto* world = getPawn()->GetWorld();
    const FCollisionQueryParams query(SCENE_QUERY_STAT(Go2BodySweep), false, getPawn());
    FCollisionResponseParams response(ECR_Ignore);
    response.CollisionResponse.SetResponse(ECC_WorldStatic, ECR_Block);
    const auto shape = FCollisionShape::MakeBox(FVector(26.5, 9.8, 9.3)); // Torso and muzzle, cm.
    const FQuat start_rotation = spawn_transform_.GetRotation() * state_.rotation;
    const FQuat end_rotation = spawn_transform_.GetRotation() * target.rotation;
    const FVector start_offset = start_rotation.RotateVector(body_box_center_);
    FVector root = spawn_transform_.TransformPositionNoScale(state_.position);
    FVector remaining = spawn_transform_.TransformVectorNoScale(target.position - state_.position);
    TArray<FVector, TInlineAllocator<3>> normals;

    for (int32 pass = 0; pass < 3 && !remaining.IsNearlyZero(); ++pass) {
        FHitResult hit;
        const FVector center = root + start_offset;
        if (!world->SweepSingleByChannel(hit, center, center + remaining, start_rotation,
                                         ECC_Pawn, shape, query, response)) {
            root += remaining;
            break;
        }
        const FVector normal = hit.Normal.GetSafeNormal();
        normals.Add(normal);
        // Leave a small gap so the next sweep can travel along the surface.
        root += remaining * hit.Time + normal * (hit.PenetrationDepth + 0.02);
        remaining *= 1.0 - hit.Time;
        remaining -= FMath::Min(0.0, FVector::DotProduct(remaining, normal)) * normal;
        if (normals.Num() == 2 && FVector::DotProduct(remaining, normals[0]) < 0.0) {
            const FVector edge = FVector::CrossProduct(normals[0], normal).GetSafeNormal();
            remaining = FVector::DotProduct(remaining, edge) * edge;
        }
    }

    // A box sweep translates at a fixed orientation. Check the turn separately,
    // including turns in place where there is no translational hit.
    const double angle = start_rotation.AngularDistance(end_rotation);
    const int32 steps = FMath::CeilToInt(angle / FMath::DegreesToRadians(2.0));
    bool turn_blocked = false;
    for (int32 step = 1; step <= steps; ++step) {
        const FQuat next = FQuat::Slerp(start_rotation, end_rotation, double(step) / steps);
        if (world->OverlapBlockingTestByChannel(root + next.RotateVector(body_box_center_), next,
                                                ECC_Pawn, shape, query, response)) {
            target.rotation = FQuat::Slerp(state_.rotation, target.rotation, double(step - 1) / steps);
            turn_blocked = true;
            break;
        }
    }
    if (normals.IsEmpty() && !turn_blocked)
        return {};

    target.position = spawn_transform_.InverseTransformPositionNoScale(root);
    const FVector3f offset(target.position / 100.0);
    // MuJoCo spawn axes, metres and WXYZ. An empty quaternion means the turn was accepted.
    std::vector<std::vector<float>> result{{offset.X, -offset.Y, offset.Z}, {}};
    if (turn_blocked) {
        const FQuat4f q(target.rotation);
        result[1] = {q.W, -q.X, q.Y, -q.Z};
    }
    for (const FVector& normal : normals) {
        const FVector3f local(spawn_transform_.InverseTransformVectorNoScale(normal));
        result.push_back({local.X, -local.Y, local.Z});
    }
    return result;
}

std::vector<std::vector<double>> Go2PawnSimApi::getNearbyObstacles(float radius)
{
    if (!std::isfinite(radius) || radius <= 0 || radius > 25)
        throw std::invalid_argument("Go2 obstacle radius must be in (0, 25] meters");

    std::vector<std::vector<double>> obstacles;
    UAirBlueprintLib::RunCommandOnGameThread([this, radius, &obstacles]() {
        APawn* pawn = getPawn();
        const FVector origin = pawn->GetActorLocation();
        const double radius_cm = radius * 100.0;
        static const FName obstacle_tag(TEXT("Go2Obstacle"));
        TArray<FOverlapResult> overlaps;
        FCollisionQueryParams query(SCENE_QUERY_STAT(Go2Obstacles), false, pawn);
        pawn->GetWorld()->OverlapMultiByObjectType(
            overlaps, origin, FQuat::Identity, FCollisionObjectQueryParams(ECC_WorldStatic),
            FCollisionShape::MakeSphere(radius_cm), query);

        TSet<const UPrimitiveComponent*> seen;
        for (const auto& overlap : overlaps) {
            const auto* component = overlap.GetComponent();
            if (!component || !component->GetOwner() ||
                !(component->ComponentHasTag(obstacle_tag) || component->GetOwner()->ActorHasTag(obstacle_tag)) ||
                seen.Contains(component))
                continue;

            const auto* box = Cast<UBoxComponent>(component);
            const FVector center = box ? box->GetComponentLocation() : component->Bounds.Origin;
            const FVector extent = box ? box->GetScaledBoxExtent() : component->Bounds.BoxExtent;
            if (extent.GetMin() <= UE_SMALL_NUMBER)
                continue;
            const FQuat obstacle_rotation = box ? box->GetComponentQuat() : FQuat::Identity;

            // Surface distance preserves walls whose centers lie outside the query radius.
            const FVector relative_center = obstacle_rotation.UnrotateVector(origin - center).GetAbs();
            const double distance = (relative_center - extent).ComponentMax(FVector::ZeroVector).Size();
            if (distance > radius_cm)
                continue;

            const FVector position = spawn_transform_.InverseTransformPositionNoScale(center) / 100.0;
            const FQuat rotation = spawn_transform_.GetRotation().Inverse() * obstacle_rotation;
            const FVector size = extent / 100.0;
            obstacles.push_back({static_cast<double>(component->GetUniqueID()),
                                 position.X, -position.Y, position.Z,
                                 size.X, size.Y, size.Z,
                                 rotation.W, -rotation.X, rotation.Y, -rotation.Z,
                                 distance / 100.0});
            seen.Add(component);
        }
    }, true);
    return obstacles;
}
