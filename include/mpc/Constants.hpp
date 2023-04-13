#pragma once

#include <Eigen/Core>

namespace controller {
static const unsigned int kHorizonLength = 10; ///< Discrete Horizon length
static const unsigned int kStateDim = 8;       ///< State dimension.
static const unsigned int kInputDim = 3;       ///< Input dimension
} // namespace controller
