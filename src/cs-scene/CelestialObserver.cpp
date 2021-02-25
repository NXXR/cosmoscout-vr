////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
//      and may be used under the terms of the MIT license. See the LICENSE file for details.     //
//                        Copyright: (c) 2019 German Aerospace Center (DLR)                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "CelestialObserver.hpp"
#include "logger.hpp"
#include <glm/ext.hpp>

namespace cs::scene {

////////////////////////////////////////////////////////////////////////////////////////////////////

CelestialObserver::CelestialObserver(std::string const& sCenterName, std::string const& sFrameName)
    : CelestialAnchor(sCenterName, sFrameName) {

}

////////////////////////////////////////////////////////////////////////////////////////////////////

void CelestialObserver::updateMovementAnimation(double tTime) {
  if (mAnimationInProgress) {
    // mPosition = mAnimatedPosition.get(tTime);
    mPosition = mMoveSpline->getPosition(mAnimatedT.get(tTime));

    if (tTime < mAnimatedRotation.mEndTime) {
      // rotation to look-at point not done yet
      mRotation = mAnimatedRotation.get(tTime);
    }
    else if (tTime > mAnimatedRotationFinal.mStartTime) {
      // rotation to final direction started
      mRotation = mAnimatedRotationFinal.get(tTime);
    }
    else {
      // observer is moving along spline -> adjust rotation towards look-at point
      auto direction = glm::normalize(*mLookAtPoint - mPosition);
      mRotation      = glm::quatLookAt(glm::normalize(direction), glm::normalize(*mUpDirection));

    }

    if (mAnimatedRotationFinal.mEndTime < tTime) {
      mAnimationInProgress = false;
      if (!mMovementQueue.empty()){
        moveTo(mMovementQueue.front());
        mMovementQueue.pop();
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void CelestialObserver::setAnchorPosition(glm::dvec3 const& vPos) {
  if (!mAnimationInProgress) {
    CelestialAnchor::setAnchorPosition(vPos);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void CelestialObserver::setAnchorRotation(glm::dquat const& qRot) {
  if (!mAnimationInProgress) {
    CelestialAnchor::setAnchorRotation(qRot);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void CelestialObserver::changeOrigin(
    std::string const& sCenterName, std::string const& sFrameName, double dSimulationTime) {

  mAnimationInProgress = false;

  cs::scene::CelestialAnchor target(sCenterName, sFrameName);

  glm::dvec3 pos = target.getRelativePosition(dSimulationTime, *this);
  glm::dquat rot = target.getRelativeRotation(dSimulationTime, *this);

  setCenterName(sCenterName);
  setFrameName(sFrameName);

  setAnchorRotation(rot);
  setAnchorPosition(pos);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void CelestialObserver::moveTo(std::string const& sCenterName, std::string const& sFrameName,
    glm::dvec3 const& position, glm::dquat const& rotation, double dSimulationTime,
    double dRealStartTime, double dRealEndTime) {
  mAnimationInProgress = false;

  // Perform no animation at all if end time is not greater than start time.
  if (dRealStartTime >= dRealEndTime) {
    setCenterName(sCenterName);
    setFrameName(sFrameName);
    setAnchorRotation(rotation);
    setAnchorPosition(position);

  } else {
    cs::scene::CelestialAnchor target(sCenterName, sFrameName);

    try {
      glm::dvec3 startPos = target.getRelativePosition(dSimulationTime, *this);
      glm::dquat startRot = target.getRelativeRotation(dSimulationTime, *this);

      setCenterName(sCenterName);
      setFrameName(sFrameName);

      double cosTheta = glm::dot(startRot, rotation);

      // If cosTheta < 0, the interpolation will take the long way around the sphere.
      // To fix this, one quat must be negated.
      if (cosTheta < 0.0) {
        startRot = -startRot;
      }

      setAnchorRotation(startRot);
      setAnchorPosition(startPos);

      // normalize Origin-Target-Vector to calculate start control points
      glm::dvec3 normalizedOT = glm::normalize(position - startPos);

      // generate look-at point and vector for final rotation
      mLookAtPoint = std::make_shared<glm::dvec3>(position + normalizedOT);

      // calculate up direction
      glm::dvec3 upDir = glm::dvec3(0.0, 1.0, 0.0);
      upDir = glm::rotate(rotation, upDir);
      mUpDirection = std::make_shared<glm::dvec3>(upDir);

      // create quat for rotation onto OT direction
      glm::dquat rotationOT = glm::quatLookAt(normalizedOT, upDir);

      // set spline Control Points
      std::vector<glm::dvec3> splinePoints = {startPos - normalizedOT, startPos,
          startPos + normalizedOT, position - normalizedOT, position, position + normalizedOT};
      mMoveSpline = std::make_shared<UniformCubicBSpline<glm::dvec3, double>>(splinePoints);

      // calculate differences between direction vectors to find weights for durations
      glm::dvec3 startDirection = glm::rotate(startRot, glm::dvec3(0.0, 0.0, -1.0));
      glm::dvec3 transitionDirection = normalizedOT;
      glm::dvec3 finalDirection = glm::rotate(rotation, glm::dvec3(0.0, 0.0, -1.0));

      // function to calculate the angle between 2 vectors and return a value (0..1) according to their alignment (0°..180°)
      auto getWeight = [](glm::dvec3 a, glm::dvec3 b){
        // calculate cos(Angle) from dot product of normalized vectors
        double cosAngle = glm::dot(glm::normalize(a),glm::normalize(b));
        // transform from cosAngle (1..-1) to weight (0..1) & clamp to cut off rounding errors
        return std::clamp(-(cosAngle-1)/2, 0.0, 1.0);
      };

      // add weight to default durations
      double weightStart = getWeight(startDirection,transitionDirection);
      double weightFinal = getWeight(transitionDirection,finalDirection);

      logger().info("weight for rotations: start rotation = {} ; end rotation = {}", weightStart, weightFinal);

      // calculate duration from start & end time to divide into rotations and translation
      double duration = dRealEndTime - dRealStartTime;
      // start & end rotation maximum duration = 1/3, if duration = 0, set it to 0.1s to avoid errors with animatedValue
      double durationStartRot = weightStart > 0.0 ? (1.0/3.0) * weightStart * duration : 0.1;
      double durationFinalRot = weightFinal > 0.0 ? (1.0/3.0) * weightFinal * duration : 0.1;

      logger().info("overall duration: {}", duration);
      logger().info("duration for rotations: start rotation = {} ; end rotation = {}", durationStartRot, durationFinalRot);

      mAnimatedT =
          utils::AnimatedValue<double>(0, mMoveSpline->getMaxT(), dRealStartTime + durationStartRot,
              dRealEndTime - durationFinalRot, utils::AnimationDirection::eInOut);

      // start rotation onto OT vector
      mAnimatedRotation = utils::AnimatedValue<glm::dquat>(startRot, rotationOT, dRealStartTime,
          dRealStartTime + durationStartRot, utils::AnimationDirection::eInOut);

      // final rotation onto look-at vector
      mAnimatedRotationFinal = utils::AnimatedValue<glm::dquat>(rotationOT, rotation,
          dRealEndTime - durationFinalRot, dRealEndTime, utils::AnimationDirection::eInOut);

      mAnimationInProgress = true;
    } catch (std::exception const& e) {
      // Getting the relative transformation may fail due to insufficient SPICE data.
      logger().warn("CelestialObserver::moveTo failed: {}", e.what());
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void CelestialObserver::moveTo(const CelestialObserver::defaultPoint2Point& moveDescriptionP2P) {
  moveTo(
      moveDescriptionP2P.sTargetCenterName,
      moveDescriptionP2P.sTargetFrameName,
      moveDescriptionP2P.finalPosition,
      moveDescriptionP2P.finalRotation,
      moveDescriptionP2P.dSimulationTime,
      moveDescriptionP2P.dRealStartTime,
      moveDescriptionP2P.dRealEndTime
      );
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void CelestialObserver::moveTo(const CelestialObserver::movementDescription_t& moveDescription) {
  switch (moveDescription.index()) {
  case 0: moveTo(std::get<0>(moveDescription));
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void CelestialObserver::moveTo(std::queue<movementDescription_t>&& moveDescriptionsQueue) {
  // make sure queue contains at least one element
  if (!moveDescriptionsQueue.empty()) {
    // swap new movements into queue
    mMovementQueue = std::move(moveDescriptionsQueue);
    // execute first movement instruction
    moveTo(mMovementQueue.front());
    mMovementQueue.pop();
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool CelestialObserver::isAnimationInProgress() const {
  return mAnimationInProgress;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

} // namespace cs::scene
