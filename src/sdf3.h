#pragma once

class SDF3 {
public:
    template <typename F>
    SDF3(const F &f) : m_Func(f) {}

    real operator()(const vec3 &p) const {
        return m_Func(p);
    }

private:
    std::function<real(const vec3 &)> m_Func;
};

// primitives
SDF3 Sphere(const real radius = 1, const vec3 &center = vec3{}) {
    return [=](const vec3 &p) -> real {
        return glm::distance(center, p) - radius;
    };
}

SDF3 Cylinder(const real radius = 1) {
    return [=](const vec3 &p) -> real {
        return glm::length(vec2(p)) - radius;
    };
}

SDF3 Plane(const vec3 &normal = Z, const vec3 &point = vec3{}) {
    return [=](const vec3 &p) -> real {
        return glm::dot(point - p, normal);
    };
}

SDF3 Box(const vec3 &size = vec3{1}) {
    return [=](const vec3 &p) -> real {
        const vec3 q = glm::abs(p) - size;
        return glm::length(glm::max(q, real(0))) + glm::min(glm::max(q.x, glm::max(q.y, q.z)), real(0));
    };
}

// CSG operations
SDF3 Union(const SDF3 &a, const SDF3 &b) {
    return [=](const vec3 &p) -> real {
        return std::min(a(p), b(p));
    };
}

SDF3 Difference(const SDF3 &a, const SDF3 &b) {
    return [=](const vec3 &p) -> real {
        return std::max(a(p), -b(p));
    };
}

SDF3 Intersection(const SDF3 &a, const SDF3 &b) {
    return [=](const vec3 &p) -> real {
        return std::max(a(p), b(p));
    };
}

// transforms

SDF3 Translate(const SDF3 &other, const vec3 &offset) {
    return [=](const vec3 &p) -> real {
        return other(p - offset);
    };
}

SDF3 Scale(const SDF3 &other, const real factor) {
    return [=](const vec3 &p) -> real {
        return other(p / factor) * factor;
    };
}

SDF3 Rotate(const SDF3 &other, const real angle, const vec3 vector = Z) {
    const vec3 v = glm::normalize(vector);
    const real x = v.x;
    const real y = v.y;
    const real z = v.z;
    const real s = std::sin(angle);
    const real c = std::cos(angle);
    const real m = 1 - c;
    const mat3 matrix{
        m*x*x + c, m*x*y + z*s, m*z*x - y*s,
        m*x*y - z*s, m*y*y + c, m*y*z + x*s,
        m*z*x + y*s, m*y*z - x*s, m*z*z + c,
    };
    return [=](const vec3 &p) -> real {
        return other(matrix * p);
    };
}

// operators
SDF3 operator|(const SDF3 &lhs, const SDF3& rhs) {
    return Union(lhs, rhs);
}

SDF3 operator-(const SDF3 &lhs, const SDF3& rhs) {
    return Difference(lhs, rhs);
}

SDF3 operator&(const SDF3 &lhs, const SDF3& rhs) {
    return Intersection(lhs, rhs);
}

SDF3 &operator|=(SDF3 &lhs, const SDF3& rhs) {
    lhs = Union(lhs, rhs);
    return lhs;
}

SDF3 &operator-=(SDF3 &lhs, const SDF3& rhs) {
    lhs = Difference(lhs, rhs);
    return lhs;
}

SDF3 &operator&=(SDF3 &lhs, const SDF3& rhs) {
    lhs = Intersection(lhs, rhs);
    return lhs;
}

SDF3 operator+(const SDF3 &lhs, const vec3& rhs) {
    return Translate(lhs, rhs);
}

SDF3 operator-(const SDF3 &lhs, const vec3& rhs) {
    return Translate(lhs, -rhs);
}

SDF3 operator*(const SDF3 &lhs, const real rhs) {
    return Scale(lhs, rhs);
}
