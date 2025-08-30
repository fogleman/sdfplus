#pragma once

using DistFunc = std::function<real(const vec3 &)>;
using ColorFunc = std::function<vec3(const vec3 &)>;

const ColorFunc DefaultColorFunc = [](const vec3 &) {
    return vec3{0};
};

class SDF3 {
public:
    template <typename F>
    SDF3(const F &f) : m_DistFunc(f), m_ColorFunc(DefaultColorFunc) {}

    SDF3(const DistFunc &distFunc, const ColorFunc &colorFunc) :
        m_DistFunc(distFunc),
        m_ColorFunc(colorFunc) {}

    real operator()(const vec3 &p) const {
        return m_DistFunc(p);
    }

    vec3 GetColor(const vec3 &p) const {
        return m_ColorFunc(p);
    }

    ColorFunc GetColorFunc() const {
        return m_ColorFunc;
    }

    SDF3 &Color(const vec3 &color) {
        m_ColorFunc = [=](const vec3 &p) -> vec3 {
            return color;
        };
        return *this;
    }

    SDF3 &Color(const int color) {
        const real r = real((color >> 16) & 255) / 255;
        const real g = real((color >> 8) & 255) / 255;
        const real b = real((color >> 0) & 255) / 255;
        return Color({r, g, b});
    }

private:
    DistFunc m_DistFunc;
    ColorFunc m_ColorFunc;
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
    const auto d = [=](const vec3 &p) -> real {
        return std::min(a(p), b(p));
    };
    const auto c = [=](const vec3 &p) -> vec3 {
        if (a(p) < b(p)) {
            return a.GetColor(p);
        } else {
            return b.GetColor(p);
        }
    };
    return SDF3(d, c);
}

SDF3 Difference(const SDF3 &a, const SDF3 &b) {
    const auto d = [=](const vec3 &p) -> real {
        return std::max(a(p), -b(p));
    };
    const auto c = [=](const vec3 &p) -> vec3 {
        if (a(p) > -b(p)) {
            return a.GetColor(p);
        } else {
            return b.GetColor(p);
        }
    };
    return SDF3(d, c);
}

SDF3 Intersection(const SDF3 &a, const SDF3 &b) {
    const auto d = [=](const vec3 &p) -> real {
        return std::max(a(p), b(p));
    };
    const auto c = [=](const vec3 &p) -> vec3 {
        if (a(p) > b(p)) {
            return a.GetColor(p);
        } else {
            return b.GetColor(p);
        }
    };
    return SDF3(d, c);
}

// transforms

SDF3 Translate(const SDF3 &other, const vec3 &offset) {
    return SDF3([=](const vec3 &p) -> real {
        return other(p - offset);
    }, other.GetColorFunc());
}

SDF3 Scale(const SDF3 &other, const real factor) {
    return SDF3([=](const vec3 &p) -> real {
        return other(p / factor) * factor;
    }, other.GetColorFunc());
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
    return SDF3([=](const vec3 &p) -> real {
        return other(matrix * p);
    }, other.GetColorFunc());
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
