/*
 * This file is modified with permission from CrossWing's
 *    telep-base/control/nav2remote.cpp
 * Copyright (C) 2012 CrossWing Inc. www.crosswing.com
 * All Rights Reserved.
 */

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <string>

#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include "Nav2remote.hpp"

nav2::Remote::Remote(const char* host, int port)
    : line(nullptr), lineLen(0), fd(-1) {
  if (port < 1 || port > 65535) throw std::invalid_argument("Invalid port");

  char service[6];
  sprintf(service, "%d", port);

  struct addrinfo* ai;
  struct addrinfo hints;
  memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  int s = getaddrinfo(host, service, &hints, &ai);
  if (s != 0) throw std::runtime_error("Can't get address info");

  for (struct addrinfo* rp = ai;; rp = rp->ai_next) {
    if (rp == NULL) {
      freeaddrinfo(ai);
      throw std::runtime_error("Can't connect to robot");
    }

    fd = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
    if (fd == -1) continue;
    if (connect(fd, rp->ai_addr, rp->ai_addrlen) == 0) break;
    close(fd);
    fd = -1;
  }

  freeaddrinfo(ai);
}

nav2::Remote::~Remote() {
  if (line) free(line);
  close(fd);
}

int nav2::Remote::readLine() const {
  for (int pos = 0;; ++pos) {
    if (lineLen <= pos + 1) {
      lineLen += 32;
      line = (char*) realloc(line, lineLen);
      if (!line) throw std::bad_alloc();
    }
    if (read(fd, &line[pos], 1) <= 0) return -1;

    if (line[pos] == '\r') {
      --pos;  // Ignore carriage returns, just in case!
      continue;
    }

    if (line[pos] == '\n') {
      line[pos] = 0;

      // Ignore lines that begin with | or +.
      if (line[0] == '|' || line[0] == '+') {
        pos = -1;
        continue;
      }

      return pos;
    }
  }
}

int nav2::Remote::setTargetOrientation(double orientation) {
  char msg[128];
  int p = sprintf(msg, "o %lf\n", orientation * (180.0 / M_PI));
  return write(fd, msg, p) == p ? 0 : -1;
}

int nav2::Remote::setAbsoluteVelocity(double vx, double vy) {
  char msg[128];
  int p = sprintf(msg, "av %lf %lf\n", vx, vy);
  return write(fd, msg, p) == p ? 0 : -1;
}

int nav2::Remote::setRelativeVelocity(double dir, double speed,
                                      double turnRate) {
  char msg[128];
  int p = sprintf(msg, "v %lf %lf %lf\n", dir * (180.0 / M_PI), speed,
                  turnRate * (180.0 / M_PI));
  return write(fd, msg, p) == p ? 0 : -1;
}

std::optional<nav2::Pose<std::chrono::steady_clock>>
    nav2::Remote::estimatePosition() const {
  if (write(fd, "q\n", 2) != 2) return std::nullopt;

  // Read the result
  if (readLine() < 0) return std::nullopt;
  const auto now = std::chrono::steady_clock::now();

  double x, y, orientation;
  int qlen;
  sscanf(line, "%lf %lf %lf %d", &x, &y, &orientation, &qlen);
  // Nav2 reports in degrees, but we keep rads for std::-trig functions:
  orientation *= (M_PI / 180.0);

  return {{now, {x, y}, orientation}};
}

std::optional<nav2::XYState<std::chrono::steady_clock>>
    nav2::Remote::estimateXYState() const {
  if (write(fd, "w\n", 2) != 2) return std::nullopt;

  // Read the result
  if (readLine() < 0) return std::nullopt;
  const auto now = std::chrono::steady_clock::now();

  double remoteTime, x, y, vx, vy;
  sscanf(line, "%lf %lf %lf %lf %lf", &remoteTime, &x, &y, &vx, &vy);

  return {{now, remoteTime, {x, y}, {vx, vy}}};
}

int nav2::Remote::setPosition(double x, double y, double orientation) {
  char msg[128];
  int p = sprintf(msg, "p %lf %lf %lf\n", x, y, orientation * (180.0 / M_PI));
  return write(fd, msg, p) == p ? 0 : -1;
}

int nav2::Remote::stop() { return write(fd, "s\n", 2) == 2 ? 0 : -1; }

int nav2::Remote::turnLeft(double angle) {
  char msg[128];
  int p = sprintf(msg, "lt %lf\n", angle * (180.0 / M_PI));
  return write(fd, msg, p) == p ? 0 : -1;
}

int nav2::Remote::move(double dist, double direction) {
  char msg[128];
  int p = sprintf(msg, "mv %lf %lf\n", dist, direction * (180.0 / M_PI));
  return write(fd, msg, p) == p ? 0 : -1;
}

int nav2::Remote::setMaxSpeed(double maxSpeed) {
  char msg[128];
  int p = sprintf(msg, "sms %lf\n", maxSpeed);
  return write(fd, msg, p) == p ? 0 : -1;
}

int nav2::Remote::setMaxAccel(double maxAccel) {
  char msg[128];
  int p = sprintf(msg, "sma %lf\n", maxAccel);
  return write(fd, msg, p) == p ? 0 : -1;
}

int nav2::Remote::setMaxCorneringError(double maxCorneringError) {
  char msg[128];
  int p = sprintf(msg, "smce %lf\n", maxCorneringError);
  return write(fd, msg, p) == p ? 0 : -1;
}

double nav2::Remote::getMaxSpeed() const {
  if (write(fd, "qms\n", 4) != 4) return -1;

  // Read the result
  if (readLine() < 0) return -1;

  double result;
  sscanf(line, "%lf", &result);

  return result;
}

double nav2::Remote::getMaxAccel() const {
  if (write(fd, "qma\n", 4) != 4) return -1;

  // Read the result
  if (readLine() < 0) return -1;

  double result;
  sscanf(line, "%lf", &result);

  return result;
}

double nav2::Remote::getMaxCorneringError() const {
  if (write(fd, "qmce\n", 5) != 5) return -1;

  // Read the result
  if (readLine() < 0) return -1;

  double result;
  sscanf(line, "%lf", &result);

  return result;
}

int nav2::Remote::getQueueSize() const {
  if (write(fd, "q\n", 2) != 2) return -1;

  // Read the result
  if (readLine() < 0) return -1;

  double x;
  int qlen;
  sscanf(line, "%lf %lf %lf %d", &x, &x, &x, &qlen);

  return qlen;
}

int nav2::Remote::setHeadTilt(double angle) {
  char msg[128];
  int p = sprintf(msg, "tilt %lf\n", angle * (180.0 / M_PI));
  return write(fd, msg, p) == p ? 0 : -1;
}

int nav2::Remote::getHeadTilt(double& angle) const {
  if (write(fd, "qtilt\n", 6) != 6) return -1;

  // Read the result
  if (readLine() < 0) return -1;

  sscanf(line, "%lf", &angle);
  angle *= M_PI / 180.0;

  return 0;
}

int nav2::Remote::wait() const {
  while (1) {
    int rc = getQueueSize();
    if (rc < 1) return rc;
    usleep(100000);
  }
}

double nav2::Remote::eval(const char* expr, double* variance) {
  std::string s = ":";
  s += expr;
  s += '\n';
  if (write(fd, s.c_str(), s.size()) != (int) s.size())
    throw std::runtime_error("Problem sending expression");

  // Read the result
  if (readLine() < 0) throw std::runtime_error("Problem reading result");

  if (line[0] != 'N' || line[1] != '(') throw std::runtime_error(line);

  double u, v;
  sscanf(line + 2, "%lf,%lf", &u, &v);
  if (variance) *variance = v;
  return u;
}
