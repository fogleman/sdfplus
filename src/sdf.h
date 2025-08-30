#pragma once

#include <chrono>
#include <fstream>
#include <functional>
#include <string>
#include <thread>
#include <vector>

#include <boost/asio/thread_pool.hpp>
#include <boost/asio/post.hpp>
#include <boost/functional/hash.hpp>
#include <boost/interprocess/file_mapping.hpp>
#include <boost/interprocess/mapped_region.hpp>

#define GLM_FORCE_CTOR_INIT
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
// #include <glm/gtc/constants.hpp>
// #include <glm/gtc/matrix_transform.hpp>
// #include <glm/gtx/component_wise.hpp>
#include <glm/gtx/hash.hpp>
// #include <glm/gtx/norm.hpp>
#include <glm/gtx/normal.hpp>
// #include <glm/gtx/string_cast.hpp>

#include <embree4/rtcore.h>
#include <pmmintrin.h>
#include <xmmintrin.h>

#define DOUBLE_PRECISION

#ifdef DOUBLE_PRECISION
    using real = double;
    using mat2 = glm::dmat2;
    using mat3 = glm::dmat3;
    using mat4 = glm::dmat4;
    using vec2 = glm::dvec2;
    using vec3 = glm::dvec3;
    using vec4 = glm::dvec4;
#else
    using real = float;
    using mat2 = glm::mat2;
    using mat3 = glm::mat3;
    using mat4 = glm::mat4;
    using vec2 = glm::vec2;
    using vec3 = glm::vec3;
    using vec4 = glm::vec4;
#endif

using ivec2 = glm::ivec2;
using ivec3 = glm::ivec3;
using ivec4 = glm::ivec4;

const vec3 X(1, 0, 0);
const vec3 Y(0, 1, 0);
const vec3 Z(0, 0, 1);

#include "util.h"
#include "stl.h"
#include "marching.h"
#include "sdf3.h"
#include "embree.h"
