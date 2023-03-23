/* Copyright (C) Imperial College, Smart Robotics Lab - All Rights Reserved
 * Unauthorised copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Dimos Tzoumanikas <dt214@ic.ac.uk>
 * 2015-2019
 */

#pragma once

#include <Eigen/Core>

namespace controller {
static const unsigned int kHorizonLength = 10; ///< Discrete Horizon length
static const unsigned int kStateDim = 8;       ///< State dimension.
static const unsigned int kInputDim = 3;       ///< Input dimension
} // namespace controller