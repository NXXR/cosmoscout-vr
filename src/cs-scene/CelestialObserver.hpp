////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
//      and may be used under the terms of the MIT license. See the LICENSE file for details.     //
//                        Copyright: (c) 2019 German Aerospace Center (DLR)                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef CS_SCENE_CELESTIAL_OBSERVER_HPP
#define CS_SCENE_CELESTIAL_OBSERVER_HPP

#include "../cs-core/Settings.hpp"
#include "../cs-utils/AnimatedValue.hpp"
#include "CelestialAnchor.hpp"
#include <queue>
#include <spline_library/splines/uniform_cr_spline.h>
#include <spline_library/splines/uniform_cubic_bspline.h>
#include <variant>

namespace cs::scene {

/// The CelestialObserver represents the camera in the scene. It provides methods for moving the
/// camera around, where the camera position and rotation is being interpolated from the start to
/// the end location.
class CS_SCENE_EXPORT CelestialObserver : public CelestialAnchor {
 public:
  explicit CelestialObserver(std::string const& sCenterName = "Solar System Barycenter",
      std::string const&                        FrameName   = "J2000");

  // Movement Description Structs:
  struct defaultPoint2Point {
    std::string sTargetCenterName;
    std::string sTargetFrameName;
    glm::dvec3 finalPosition;
    glm::dquat finalRotation;
    double dSimulationTime;
    double dRealStartTime;
    double dRealEndTime;
  };

  struct defaultOrbit {
    std::string sTargetFrameName;
    glm::dvec3 finalPosition;
    glm::dquat finalRotation;
    double dSimulationTime;
    double dRealStartTime;
    double dRealEndTime;
  };

  // Variant to bundle movement descriptions
  using MovementDescription = std::variant<defaultPoint2Point,defaultOrbit>;

  /// Updates position and rotation according to the last moveTo call.
  virtual void updateMovementAnimation(double tTime);

  /// These are overidden here because they are ignored if any animation done by MoveTo is in
  /// progress.
  void setAnchorPosition(glm::dvec3 const& vPos) override;
  void setAnchorRotation(glm::dquat const& qRot) override;

  /// Calls setCenterName() and setFrameName() but updates position and rotation in such a way that
  /// the universal position and orientation does not change. This may throw a std::runtime_error if
  /// no sufficient SPICE data is available.
  void changeOrigin(
      std::string const& sCenterName, std::string const& sFrameName, double dSimulationTime);

  /// Gradually moves the observer's position and rotation from their current values to the given
  /// values.
  ///
  /// @param sCenterName      The SPICE name of the targets center.
  /// @param sFrameName       The SPICE reference frame of the targets location.
  /// @param position         The target position in the targets coordinate system.
  /// @param rotation         The target rotation in the targets coordinate system.
  /// @param dSimulationTime  The current time of the simulation in Barycentric Dynamical Time.
  /// @param dRealStartTime   The time in the real world, when the animation should start, in TDB.
  /// @param dRealEndTime     The time in the real world, when the animation should finish, in TDB.
  void moveTo(std::string const& sCenterName, std::string const& sFrameName,
      glm::dvec3 const& position, glm::dquat const& rotation, double dSimulationTime,
      double dRealStartTime, double dRealEndTime);

  /// Gradually moves the observer's position and rotation from their current values to the given
  /// values.
  ///
  /// @param moveDescriptionP2P   The defaultPoint2Point movement description struct.
  void moveTo(defaultPoint2Point const& moveDescriptionP2P);

  /// Gradually moves the observer's position and rotation from their current values to the given
  /// values.
  ///
  /// @param moveDescriptionOrbit   The defaultOrbit movement description struct.
  void moveTo(defaultOrbit const& moveDescriptionOrbit);

  /// Gradually moves the observer's position and rotation from their current values to the given
  /// values.
  ///
  /// @param moveDescriptionP2P   Any movement description struct.
  void moveTo(MovementDescription const& moveDescription);

  /// Gradually moves the observer's position and rotation from their current values to the given
  /// values.
  ///
  /// @param moveDescription   The queue of movement description structs.
  void moveTo(std::queue<MovementDescription>const & moveDescriptionsQueue);

  /// @return true, if the observer is currently being moved.
  bool isAnimationInProgress() const;

 protected:
  utils::AnimatedValue<double> mAnimatedT;
  std::shared_ptr<UniformCubicBSpline<glm::dvec3, double>> mMoveSpline;
  glm::dvec3 mUpDirection{};
  glm::dvec3 mLookAtPoint{};
  utils::AnimatedValue<glm::dvec3> mAnimatedPosition;
  utils::AnimatedValue<glm::dquat> mAnimatedRotationStart;
  utils::AnimatedValue<glm::dquat> mAnimatedRotationFinal;

  std::queue<MovementDescription> mMovementQueue{};

  bool mAnimationInProgress = false;
};
} // namespace cs::scene

#endif // CS_SCENE_CELESTIAL_OBSERVER_HPP
