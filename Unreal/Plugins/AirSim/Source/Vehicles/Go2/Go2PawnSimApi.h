#pragma once

#include "Go2Api.h"
#include "PawnSimApi.h"

#include <array>
#include <mutex>

class Go2PawnSimApi : public PawnSimApi
{
public:
    explicit Go2PawnSimApi(const Params& params);

    void initialize() override;
    void resetImplementation() override;
    void updateRenderedState(float dt) override;
    Go2PawnSimApi* asGo2() override { return this; }
    static void alignSpawnToGround(APawn* pawn);
    msr::airlib::VehicleApiBase* getVehicleApiBase() const override;

    void setJointAngles(const std::vector<float>& angles);
    std::vector<float> getJointAngles() const;
    std::vector<std::vector<float>> getState();
    void setBaseOffset(const std::vector<float>& offset);
    std::vector<std::vector<double>> getNearbyObstacles(float radius);
    void setState(const std::vector<float>& angles, const std::vector<float>& offset,
                  const std::vector<float>& orientation);
    std::vector<std::vector<float>> moveState(const std::vector<float>& angles, const std::vector<float>& offset,
                                              const std::vector<float>& orientation);

private:
    struct State
    {
        std::array<float, 12> angles{0, 0.9f, -1.8f, 0, 0.9f, -1.8f,
                                     0, 0.9f, -1.8f, 0, 0.9f, -1.8f};
        FVector position = FVector::ZeroVector;
        FQuat rotation = FQuat::Identity;
    };

    void applyState(const State& state, bool apply_pose);
    void readCurrentPose();
    static State decodeState(const std::vector<float>& angles, const std::vector<float>& offset,
                             const std::vector<float>& orientation);
    std::vector<std::vector<float>> sweepBody(State& target);

    std::unique_ptr<msr::airlib::Go2Api> vehicle_api_;
    std::array<USceneComponent*, 12> joints_{};
    std::array<FQuat, 12> joint_rest_rotations_;
    FTransform spawn_transform_;
    FVector body_box_center_;
    mutable std::mutex state_mutex_;
    State state_;
    bool pose_pending_ = false;
};
