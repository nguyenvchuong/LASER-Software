/*! @file MiniCheetah.h
 *  @brief Utility function to build a AlienGo Quadruped object
 *
 * This file is based on MiniCheetahFullRotorModel_mex.m and builds a my_model
 * of the AlienGo robot.  The inertia parameters of all bodies are
 * determined from CAD.
 *
 */

#ifndef PROJECT_ALIENGO_H
#define PROJECT_ALIENGO_H

#include "FloatingBaseModel.h"
#include "Quadruped.h"
#include "../../src/Quadruped.cpp"

/*!
 * Generate a Quadruped my_model of Aliengo
 */
template <typename T>
Quadruped<T> buildAlienGo() {
  Quadruped<T> aliengo;

  aliengo._bodyMass = 9.041;
  aliengo._bodyLength = 0.647;
  aliengo._bodyWidth = 0.15;
  aliengo._bodyHeight = 0.112;
  aliengo._abadGearRatio = 6;
  aliengo._hipGearRatio = 6;
  aliengo._kneeGearRatio = 9.33;
  aliengo._abadLinkLength = 0.0418;
  aliengo._hipLinkLength = 0.25;
  aliengo._kneeLinkLength = 0.25;
  aliengo._maxLegLength = 0.5;

  aliengo._motorTauMax = 44.4;
  aliengo._batteryV = 24;
  aliengo._motorKT = .05;  // this is flux linkage * pole pairs
  aliengo._motorR = 0.173;  
  aliengo._jointDamping = .01;
  aliengo._jointDryFriction = .2;   // still need these values for aliengo robot
  //cheetah._jointDamping = .0;
  //cheetah._jointDryFriction = .0;


  // rotor inertia if the rotor is oriented so it spins around the z-axis (need value for aliengo robot)
  Mat3<T> rotorRotationalInertiaZ;
  rotorRotationalInertiaZ << 33, 0, 0, 0, 33, 0, 0, 0, 63;
  rotorRotationalInertiaZ = 1e-6 * rotorRotationalInertiaZ;

  Mat3<T> RY = coordinateRotation<T>(CoordinateAxis::Y, M_PI / 2);
  Mat3<T> RX = coordinateRotation<T>(CoordinateAxis::X, M_PI / 2);
  Mat3<T> rotorRotationalInertiaX =
      RY * rotorRotationalInertiaZ * RY.transpose();
  Mat3<T> rotorRotationalInertiaY =
      RX * rotorRotationalInertiaZ * RX.transpose();

  // spatial inertias
  Mat3<T> abadRotationalInertia;
  abadRotationalInertia << 2093, -71,-1, -71, 4907.5, -1.75, -1, -1.75, 5586.9;
  abadRotationalInertia = abadRotationalInertia * 1e-6;
  Vec3<T> abadCOM(-0.022, 0.015, 0);  // LEFT
  SpatialInertia<T> abadInertia(1.993, abadCOM, abadRotationalInertia);

  Mat3<T> hipRotationalInertia;
  hipRotationalInertia << 5666.8, 3.59, 491.4, 3.59, 5847.2, 10, 491.4, 10, 369.8;
  hipRotationalInertia = hipRotationalInertia * 1e-6;
  Vec3<T> hipCOM(-0.005, -0.0038, -0.048);
  SpatialInertia<T> hipInertia(0.639, hipCOM, hipRotationalInertia);

  Mat3<T> kneeRotationalInertia, kneeRotationalInertiaRotated;
  kneeRotationalInertiaRotated << 6341.3, 0, -87.9, 0, 6355.1, 1.3, -87.9, 1.3, 39.1;
  kneeRotationalInertiaRotated = kneeRotationalInertiaRotated * 1e-6; 
  kneeRotationalInertia = RY * kneeRotationalInertiaRotated * RY.transpose();
  Vec3<T> kneeCOM(0.0027, 0, -0.1425);
  SpatialInertia<T> kneeInertia(0.207, kneeCOM, kneeRotationalInertia);

  Vec3<T> rotorCOM(0, 0, 0);
  SpatialInertia<T> rotorInertiaX(0.055, rotorCOM, rotorRotationalInertiaX);
  SpatialInertia<T> rotorInertiaY(0.055, rotorCOM, rotorRotationalInertiaY);

  Mat3<T> bodyRotationalInertia;
  bodyRotationalInertia << 33260.2, 0 , 0, 0, 36203, 0, 0, 0, 42673;
  bodyRotationalInertia = bodyRotationalInertia * 1e-6;
  Vec3<T> bodyCOM(0.008465, 0.004045, -0.000763);
  SpatialInertia<T> bodyInertia(aliengo._bodyMass, bodyCOM,
                                bodyRotationalInertia);

  aliengo._abadInertia = abadInertia;
  aliengo._hipInertia = hipInertia;
  aliengo._kneeInertia = kneeInertia;
  aliengo._abadRotorInertia = rotorInertiaX;
  aliengo._hipRotorInertia = rotorInertiaY;
  aliengo._kneeRotorInertia = rotorInertiaY;
  aliengo._bodyInertia = bodyInertia;

  // locations
  aliengo._abadRotorLocation = Vec3<T>(0.2399, 0.051, 0.083);
  aliengo._abadLocation =
      Vec3<T>(aliengo._bodyLength, aliengo._bodyWidth, 0) * 0.5;
  aliengo._hipLocation = Vec3<T>(0, aliengo._abadLinkLength, 0);
  aliengo._hipRotorLocation = Vec3<T>(0, 0.04, 0);
  aliengo._kneeLocation = Vec3<T>(0, 0, -aliengo._hipLinkLength);
  aliengo._kneeRotorLocation = Vec3<T>(0, 0, 0);

  return aliengo;
}

#endif  