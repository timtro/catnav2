#pragma once
/*
 * This file is modified with permission from CrossWing's
 *    telep-base/control/nav2remote.h
 * Copyright (C) 2012 CrossWing Inc. www.crosswing.com
 * All Rights Reserved.
 */

#include <chrono>
#include <optional>
#include <variant>

#include "../include/XY.hpp"
#include "../include/nav2_Pose.hpp"

/**
 * @brief
 * Class for controlling Nav2 via turtle interface.
 *
 * This class is a C++ front-end for Crosswing's serial turtle interface.
 * This interface is <b>not</b> thread-safe.  Most of these functions
 * block on I/O.
 *
 * Note that the turtle commands run asynchronously.  Each
 * command is added to the robot's path plan, and the function returns
 * immediately.  Call getQueueSize() to determine whether
 * the robot has completed the requested path.  Call wait() to wait until
 * the queue is empty.
 *
 * When this object goes out of scope (eg, when the program finishes),
 * the robot will stop immediately, even if there are commands in the queue.
 * Use wait() if you want to ensure that the path completes.
 */

namespace nav2 {

  template <typename Clock = std::chrono::steady_clock>
  struct XYState {
    std::chrono::time_point<Clock> localTimestamp;
    double remoteTimestamp;
    XY position;
    XY velocity;
  };

  namespace actions {
    struct Null {};
    struct Stop {};
    struct SetRelativeVelocity {
      double dir = 0;
      double speed = 0;
      double turnRate = 0;
    };
  }  // namespace actions

  using Action =
      std::variant<actions::Null, actions::Stop, actions::SetRelativeVelocity>;

  class Remote {
    // No copying allowed
    Remote(const Remote&);
    Remote& operator=(const Remote&);

    mutable char* line;
    mutable int lineLen;
    int fd;

    int readLine() const;

   public:
    /**
     * @brief Create the Remote object.
     *
     * Constructs the object and connects to a remote turtle interface.
     * If there is a connection problem, this function throws
     * std::runtime_error.
     *
     * @param host The remote hostname or IP address.
     * @param port The TCP port.
     */
    Remote(const char* host, int port = 5010);

    ~Remote();

    /**
     * @brief Set the target orientation in world coordinates.
     *
     * At an orientation of zero, the robot is facing in the direction of
     * the positive x axis.  If the robot needs to rotate, its rotational
     * speed will gradually increase then gradually decrease to try to
     * achieve the given orientation.
     *
     * @param orientation the target orientation in radians.
     * @return 0 on success, non-zero on IO error.
     */
    int setTargetOrientation(double orientation);

    /**
     * @brief Set the velocity in absolute coordinates
     *
     * Set the robot's velocity in absolute coordinates.
     * <b>This function overrides and clears the turtle command queue.</b>
     *
     * @param vx The x-component of the velocity in meters per second.
     * @param vy The y-component of the velocity in meters per second.
     * @return 0 on success, non-zero on IO error.
     * @see setRelativeVelocity, estimatePosition
     */
    int setAbsoluteVelocity(double vx, double vy);

    /**
     * @brief Set the relative velocity and turn rate.
     *
     * Set the robot's velocity relative to its own orientation, and
     * the rate of rotation.  If the speed and turn rate are both
     * non-zero, the robot will follow a circular path.  A speed of
     * zero with a non-zero turn rate will cause the robot to rotate
     * in place.
     * <b>This function overrides and clears the turtle command queue.</b>
     *
     * <b>Examples:</b><br>
     * setRelativeVelocity(0.0,0.3,0.0)  - Move straight forward.<br>
     * setRelativeVelocity(1.5708,0.3,0.0)  - Move leftward.<br>
     * setRelativeVelocity(-1.5708,0.3,-1.5)  - Move rightward, while rotating
     * clockwise.<br>
     *
     * @param dir The direction of travel relative to forward, in radians.
     * @param speed The speed in meters per second.
     * @param turnRate The turn rate in radians per second.  Use positive
     * values for counterclockwise rotation, or negative values for
     * clockwise rotation.
     * @return 0 on success, non-zero on IO error.
     * @see setAbsoluteVelocity
     */
    int setRelativeVelocity(double dir, double speed, double turnRate = 0.0);

    /**
     * @brief Estimate the position and orientation.
     *
     * @return returns Pose on success, nullopt on IO error.
     */
    std::optional<Pose<>> estimatePosition() const;

    /**
     * @brief Estimate the position and orientation.
     *
     * @return returns Pose on success, nullopt on IO error.
     */
    std::optional<XYState<>> estimateXYState() const;

    /**
     * @brief Set the current actual position and orientation.
     *
     * The provided values
     * will be used as ground truth for future estimates.
     *
     * @param x The absolute x coordinate in meters.
     * @param y The absolute y coordinate in meters.
     * @param orientation The absolute orientation in radians.
     * @return 0 on success, non-zero on IO error.
     */
    int setPosition(double x, double y, double orientation);

    /**
     * @brief Stop the robot immediately.
     *
     * @return 0 on success, non-zero on IO error.
     */
    int stop();

    /**
     * @brief Turn left.
     *
     * Queue a turtle command to turn left by the specified angle.
     *
     * @param angle The angle in radians.
     * @return 0 on success, non-zero on IO error.
     */
    int turnLeft(double angle);

    /**
     * @brief Turn right.
     *
     * Queue a turtle command to turn right by the specified angle.
     *
     * @param angle The angle in radians.
     * @return 0 on success, non-zero on IO error.
     */
    int turnRight(double angle) { return turnLeft(-angle); }

    /**
     * @brief Move in a specified direction.
     *
     * Queue a turtle command to move a specified distance
     * in the specified direction (relative to the robot's heading).
     *
     * @param dist The distance in meters.
     * @param direction The relative direction of travel in radians.
     * Positive values are to the robot's left;
     * Negative values are to the robot's right.
     * @return 0 on success, non-zero on IO error.
     */
    int move(double dist, double direction);

    /**
     * @brief Move forward.
     *
     * Queue a turtle command to move the robot forward by the specified
     * distance.  This is equivalent to move(dist,0.0).
     *
     * @param dist The distance in meters.
     * @return 0 on success, non-zero on IO error.
     */
    int forward(double dist) { return move(dist, 0.0); }

    /**
     * @brief Move backward.
     *
     * Queue a turtle command to move the robot backward by the specified
     * distance.  This is equivalent to move(-dist,0.0).
     *
     * @param dist The distance in meters.
     * @return 0 on success, non-zero on IO error.
     */
    int back(double dist) { return forward(-dist); }

    /**
     * @brief Set the maximum speed for turtle commands.
     *
     * This does not affect the behaviour of setTargetVelocity().
     *
     * @param maxSpeed The maximum speed in meters per second.
     * @return 0 on success, non-zero on IO error.
     * @see getMaxSpeed
     */
    int setMaxSpeed(double maxSpeed);

    /**
     * @brief Set the maximum acceleration for turtle commands.
     *
     * This does not affect the behaviour of setTargetVelocity().
     *
     * @param maxAccel The maximum acceleration in meters per second squared.
     * @return 0 on success, non-zero on IO error.
     * @see getMaxAccel
     */
    int setMaxAccel(double maxAccel);

    /**
     * @brief Set the maximum cornering error for turtle commands.
     *
     * When following turtle commands, the robot will plan to
     * come at least that near to each corner in the resulting path.
     * If this value is set to 0, the robot will come to a complete stop
     * at each corner.  Setting this to larger values allows the robot to
     * follow the path at a higher speed.
     *
     * @param error The maximum cornering error in meters.
     * @return 0 on success, non-zero on IO error.
     * @see getMaxCorneringError
     */
    int setMaxCorneringError(double error);

    /**
     * @brief Get the maximum speed for turtle commands.
     *
     * @return The maximum speed in meters per second, or -1 on IO error.
     * @see setMaxSpeed
     */
    double getMaxSpeed() const;

    /**
     * @brief Get the maximum acceleration for turtle commands.
     *
     * @return The maximum acceleration in meters per second squared,
     * or -1 on IO error.
     * @see setMaxAccel
     */
    double getMaxAccel() const;

    /**
     * @brief Get the maximum cornering error for turtle commands.
     *
     * @return The maximum cornering error in meters, or -1 on IO error.
     * @see setMaxCorneringError
     */
    double getMaxCorneringError() const;

    /**
     * @brief Get the turtle command queue size.
     *
     * Get the number of turtle line segments remaining in the queue.
     * @return The number of segments, or -1 on IO error.
     * @see wait
     */
    int getQueueSize() const;

    /**
     * @brief Tilt the head
     *
     * Tilt the head to the given angle.  Angle 0 indicates that the
     * robot is looking straight horizontally.  Positive angles indicate
     * that the head is angled upwards.
     * @param angle The tilt angle in radians.
     * @return 0 on success, or -1 on IO error.
     * @see getHeadTilt
     */
    int setHeadTilt(double angle);

    /**
     * @brief Get the head tilt angle
     *
     * Get the angle of the head tilt.  Angle 0 indicates that the
     * robot is looking straight horizontally.  Positive angles indicate
     * that the head is angled upwards.
     * @param angle Output parameter for the tilt angle in radians.
     * @return 0 on success, or -1 on IO error.
     * @see setHeadTilt
     */
    int getHeadTilt(double& angle) const;

    /**
     * @brief Wait until the turtle command queue is empty.
     * @return 0 on success, or -1 on IO error.
     * @see getQueueSize
     */
    int wait() const;

    /**
     * @brief Execute a GRV expression on the robot.
     *
     * @param expr The expression to evaluate
     * @param variance Output variable for fetching the variance of the result,
     * if needed.
     */
    double eval(const char* expr, double* variance = 0);

    /**
     * @brief Execute a nav2::Action.
     *
     * @param expr The expression to evaluate
     * @return Integer returned by corresponding method call: 0 on success, or
     *   -1 on IO error.
     */
    int execute(Action);
  };

}  // namespace nav2
