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

    if (tTime < mAnimatedRotationStart.mEndTime) {
      // rotation to look-at point not done yet
      mRotation = mAnimatedRotationStart.get(tTime);
    } else if (tTime > mAnimatedRotationFinal.mStartTime) {
      // rotation to final direction started
      mRotation = mAnimatedRotationFinal.get(tTime);
    } else {
      // observer is moving along spline -> adjust rotation towards look-at point
      auto direction = glm::normalize(*mLookAtPoint - mPosition);
      mRotation      = glm::quatLookAt(glm::normalize(direction), glm::normalize(*mUpDirection));
    }

    if (mAnimatedRotationFinal.mEndTime < tTime) {
      mAnimationInProgress = false;
      if (!mMovementQueue.empty()) {
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
      upDir            = glm::rotate(rotation, upDir);
      mUpDirection     = std::make_shared<glm::dvec3>(upDir);

      // create quat for rotation onto OT direction
      glm::dquat rotationOT = glm::quatLookAt(normalizedOT, upDir);

      // set spline Control Points
      std::vector<glm::dvec3> splinePoints = {startPos - normalizedOT, startPos,
          startPos + normalizedOT, position - normalizedOT, position, position + normalizedOT};
      mMoveSpline = std::make_shared<UniformCubicBSpline<glm::dvec3, double>>(splinePoints);

      // calculate differences between direction vectors to find weights for durations
      glm::dvec3 startDirection      = glm::rotate(startRot, glm::dvec3(0.0, 0.0, -1.0));
      glm::dvec3 transitionDirection = normalizedOT;
      glm::dvec3 finalDirection      = glm::rotate(rotation, glm::dvec3(0.0, 0.0, -1.0));

      // function to calculate the angle between 2 vectors and return a value (0..1) according to
      // their alignment (0°..180°)
      auto getWeight = [](glm::dvec3 a, glm::dvec3 b) {
        // calculate cos(Angle) from dot product of normalized vectors
        double cosAngle = glm::dot(glm::normalize(a), glm::normalize(b));
        // transform from cosAngle (1..-1) to weight (0..1) & clamp to cut off rounding errors
        return std::clamp(-(cosAngle - 1) / 2, 0.0, 1.0);
      };

      // add weight to default durations
      double weightStart = getWeight(startDirection, transitionDirection);
      double weightFinal = getWeight(transitionDirection, finalDirection);

      // calculate duration from start & end time to divide into rotations and translation
      double duration = dRealEndTime - dRealStartTime;
      // start & end rotation maximum duration = 1/3, if duration = 0, set it to 0.1s to avoid
      // errors with animatedValue
      double durationStartRot = (1.0 / 3.0) * weightStart * duration;
      double durationFinalRot = (1.0 / 3.0) * weightFinal * duration;

      // set AnimatedValue for moveSpline
      mAnimatedT =
          utils::AnimatedValue<double>(0, mMoveSpline->getMaxT(), dRealStartTime + durationStartRot,
              dRealEndTime - durationFinalRot, utils::AnimationDirection::eInOut);

      // start rotation onto OT vector
      mAnimatedRotationStart = utils::AnimatedValue<glm::dquat>(startRot, rotationOT,
          dRealStartTime, dRealStartTime + durationStartRot, utils::AnimationDirection::eInOut);

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
  moveTo(moveDescriptionP2P.sTargetCenterName, moveDescriptionP2P.sTargetFrameName,
      moveDescriptionP2P.finalPosition, moveDescriptionP2P.finalRotation,
      moveDescriptionP2P.dSimulationTime, moveDescriptionP2P.dRealStartTime,
      moveDescriptionP2P.dRealEndTime);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void CelestialObserver::moveTo(const CelestialObserver::defaultOrbit& moveDescriptionOrbit) {
  mAnimationInProgress = false;

  // Perform no animation at all if end time is not greater than start time.
  if (moveDescriptionOrbit.dRealStartTime >= moveDescriptionOrbit.dRealEndTime) {
    setAnchorRotation(moveDescriptionOrbit.finalRotation);
    setAnchorPosition(moveDescriptionOrbit.finalPosition);

  } else {
    try {
      // check current and target frame against center
      bool isTargetFrameDifferent = true;
      bool isCurrentIAU           = false;
      if (moveDescriptionOrbit.sTargetFrameName == "IAU_" + getCenterName()) {
        isTargetFrameDifferent = false;
      }
      if (getFrameName() == "IAU_" + getCenterName()) {
        isCurrentIAU = true;
      }

      glm::dvec3      startPos = mPosition;
      glm::dquat      startRot = mRotation;
      CelestialAnchor iauFrame(getCenterName(), "IAU_" + getCenterName());
      if (!isCurrentIAU) {
        // shift into IAU frame "IAU_<center>"
        startPos = iauFrame.getRelativePosition(moveDescriptionOrbit.dSimulationTime, *this);
        startRot = iauFrame.getRelativeRotation(moveDescriptionOrbit.dSimulationTime, *this);
        setFrameName("IAU_" + getCenterName());
        // adjust position
        setAnchorRotation(startRot);
        setAnchorPosition(startPos);
      }

      glm::dvec3 finalPos = moveDescriptionOrbit.finalPosition;
      glm::dquat finalRot = moveDescriptionOrbit.finalRotation;
      if (isTargetFrameDifferent) {
        CelestialAnchor targetFrameAnchor(getCenterName(), moveDescriptionOrbit.sTargetFrameName);
        targetFrameAnchor.setAnchorPosition(finalPos);
        targetFrameAnchor.setAnchorRotation(finalRot);
        finalPos =
            iauFrame.getRelativePosition(moveDescriptionOrbit.dSimulationTime, targetFrameAnchor);
        finalRot =
            iauFrame.getRelativeRotation(moveDescriptionOrbit.dSimulationTime, targetFrameAnchor);
      }

      // calculate normal on origin-target-plane
      glm::dvec3 normalOT =
          glm::normalize(glm::triangleNormal(glm::dvec3(0.0, 0.0, 0.0), startPos, finalPos));

      // calculate angle between origin & target
      double angleOT =
          glm::orientedAngle(glm::normalize(startPos), glm::normalize(finalPos), normalOT);

      // calculate control points in 2D (on origin-target-plane) and with unit circle
      // according to https://pomax.github.io/bezierinfo/#circles_cubic
      glm::dvec2 s_pos(1.0, 0.0);
      glm::dvec2 e_pos(glm::cos(angleOT), glm::sin(angleOT));
      double     f = (4.0 / 3.0) * glm::tan(angleOT / 4.0);
      glm::dvec2 c_1(1.0, f);
      glm::dvec2 c_2(
          glm::cos(angleOT) + f * glm::sin(angleOT), glm::sin(angleOT) - f * glm::cos(angleOT));

      // calculate angle between start/end point and control points
      double angleS_C1 = glm::orientedAngle(glm::normalize(s_pos), glm::normalize(c_1));
      double angleE_C2 = glm::orientedAngle(glm::normalize(e_pos), glm::normalize(c_2));

      // rotate start pos by angle and scale by 2D length
      glm::dvec3 c1Pos  = glm::rotate(startPos, angleS_C1, normalOT) * glm::length(c_1);
      glm::dvec3 c1rPos = glm::rotate(startPos, -angleS_C1, normalOT) * glm::length(c_1);
      glm::dvec3 c2Pos  = glm::rotate(finalPos, angleE_C2, normalOT) * glm::length(c_2);
      glm::dvec3 c2rPos = glm::rotate(finalPos, -angleE_C2, normalOT) * glm::length(c_2);

      // if target frame is different, convert all point into target frame
      if (isTargetFrameDifferent) {
        // set up IAU to target frame conversion
        CelestialAnchor targetFrame(getCenterName(), moveDescriptionOrbit.sTargetFrameName);
        CelestialAnchor iauObject(getCenterName(), "IAU_" + getCenterName());
        // move anchor and convert start pos
        iauObject.setAnchorPosition(startPos);
        iauObject.setAnchorRotation(startRot);
        startPos = targetFrame.getRelativePosition(moveDescriptionOrbit.dSimulationTime, iauObject);
        startRot = targetFrame.getRelativeRotation(moveDescriptionOrbit.dSimulationTime, iauObject);
        // reset final pos and rot from earlier conversion
        finalPos = moveDescriptionOrbit.finalPosition;
        finalRot = moveDescriptionOrbit.finalRotation;
        // move anchor and convert control point positions
        iauObject.setAnchorPosition(c1Pos);
        c1Pos = targetFrame.getRelativePosition(moveDescriptionOrbit.dSimulationTime, iauObject);
        iauObject.setAnchorPosition(c1rPos);
        c1rPos = targetFrame.getRelativePosition(moveDescriptionOrbit.dSimulationTime, iauObject);
        iauObject.setAnchorPosition(c2Pos);
        c2Pos = targetFrame.getRelativePosition(moveDescriptionOrbit.dSimulationTime, iauObject);
        iauObject.setAnchorPosition(c2rPos);
        c2rPos = targetFrame.getRelativePosition(moveDescriptionOrbit.dSimulationTime, iauObject);
      }

      // set look-at point depending on target frame
      if (isTargetFrameDifferent) {
        CelestialAnchor targetFrame(getCenterName(), moveDescriptionOrbit.sTargetFrameName);
        CelestialAnchor iauCenter(getCenterName(), "IAU_" + getCenterName());
        mLookAtPoint = std::make_shared<glm::dvec3>(
            targetFrame.getRelativePosition(moveDescriptionOrbit.dSimulationTime, iauCenter));
      } else {
        mLookAtPoint = std::make_shared<glm::dvec3>(0.0, 0.0, 0.0);
      }

      // create spline
      std::vector<glm::dvec3> splinePoints = {c1rPos, startPos, c1Pos, c2Pos, finalPos, c2rPos};
      mMoveSpline = std::make_shared<UniformCubicBSpline<glm::dvec3, double>>(splinePoints);

      // calculate up direction
      mUpDirection = std::make_shared<glm::dvec3>(glm::rotate(startRot, glm::dvec3(0.0, 1.0, 0.0)));

      // calculate rotation durations
      glm::dvec3 startDirection      = glm::rotate(startRot, glm::dvec3(0.0, 0.0, -1.0));
      glm::dvec3 transitionDirection = -startPos;
      glm::dvec3 finalDirection      = glm::rotate(finalRot, glm::dvec3(0.0, 0.0, -1.0));

      // function to calculate the angle between 2 vectors and return a value (0..1) according to
      // their alignment (0°..180°)
      auto getWeight = [](glm::dvec3 a, glm::dvec3 b) {
        // calculate cos(Angle) from dot product of normalized vectors
        double cosAngle = glm::dot(glm::normalize(a), glm::normalize(b));
        // transform from cosAngle (1..-1) to weight (0..1) & clamp to cut off rounding errors
        return std::clamp(-(cosAngle - 1) / 2, 0.0, 1.0);
      };

      // set timings
      double startTime = moveDescriptionOrbit.dRealStartTime;
      double endTime   = moveDescriptionOrbit.dRealEndTime;
      double duration  = endTime - startTime;
      double durationStartRot =
          (1.0 / 3.0) * duration * getWeight(startDirection, transitionDirection);
      double durationFinalRot =
          (1.0 / 3.0) * duration * getWeight(transitionDirection, finalDirection);

      // set animation for move spline
      mAnimatedT =
          utils::AnimatedValue<double>(0, mMoveSpline->getMaxT(), startTime + durationStartRot,
              endTime - durationFinalRot, utils::AnimationDirection::eInOut);

      // set animation for start rotation
      glm::dquat lookAtCenter = glm::quatLookAt(*mLookAtPoint - startPos, *mUpDirection);
      mAnimatedRotationStart  = utils::AnimatedValue<glm::dquat>(startRot, lookAtCenter, startTime,
          startTime + durationStartRot, utils::AnimationDirection::eInOut);

      // set animation for final rotation
      lookAtCenter           = glm::quatLookAt(*mLookAtPoint - finalPos, *mUpDirection);
      mAnimatedRotationFinal = utils::AnimatedValue<glm::dquat>(lookAtCenter, finalRot,
          endTime - durationFinalRot, endTime, utils::AnimationDirection::eInOut);

      mAnimationInProgress = true;
    } catch (std::exception const& e) {
      // Getting the relative transformation may fail due to insufficient SPICE data.
      logger().warn("CelestialObserver::moveTo failed: {}", e.what());
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void CelestialObserver::moveTo(const CelestialObserver::MovementDescription& moveDescription) {
  switch (moveDescription.index()) {
  case 0:
    moveTo(std::get<0>(moveDescription));
    break;
  case 1:
    moveTo(std::get<1>(moveDescription));
    break;
  default:
    logger().warn("CelestialObserver::moveTo failed: movementDescription type index out of bounds "
                  "(index: '{}')",
        moveDescription.index());
    break;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void CelestialObserver::moveTo(std::queue<MovementDescription> const& moveDescriptionsQueue) {
  // make sure queue contains at least one element
  if (!moveDescriptionsQueue.empty()) {
    // swap new movements into queue
    mMovementQueue = moveDescriptionsQueue;
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
