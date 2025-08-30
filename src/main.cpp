#include "sdf.h"

int main(int argc, char **argv) {
    RTCDevice device = rtcNewDevice(NULL);

    const real r = 320;

    SDF3 f = Sphere(r);
    f &= Box(vec3(r * 0.75));
    f -= Rotate(Cylinder(r / 2), M_PI / 2, X);
    f -= Rotate(Cylinder(r / 2), M_PI / 2, Y);
    f -= Rotate(Cylinder(r / 2), M_PI / 2, Z);
    f &= Plane(Z);

    f = Mesh(device, argv[1]);
    f &= Plane(Y);

    std::vector<vec3> points;
    std::mutex mutex;

    const auto worker = [&](const int wi, const int wn) {
        _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
        _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);

        std::vector<vec3> workerPoints;
        const int h = r + 1;
        const real kHalfDiag = 0.8660254037844386;
        for (int x0 = -h + wi; x0 < h; x0 += wn) {
            const int x1 = x0 + 1;
            for (int y0 = -h; y0 < h; y0++) {
                const int y1 = y0 + 1;
                for (int z0 = -h; z0 < h; z0++) {
                    const int z1 = z0 + 1;

                    const real d = std::abs(f(vec3(x0 + 0.5, y0 + 0.5, z0 + 0.5)));
                    if (d > kHalfDiag) {
                        z0 += std::floor(d - kHalfDiag);
                        continue;
                    }

                    const std::array<vec3, 8> p = {{
                        {x0, y0, z0},
                        {x1, y0, z0},
                        {x1, y1, z0},
                        {x0, y1, z0},
                        {x0, y0, z1},
                        {x1, y0, z1},
                        {x1, y1, z1},
                        {x0, y1, z1},
                    }};

                    const std::array<real, 8> v = {{
                        f(p[0]),
                        f(p[1]),
                        f(p[2]),
                        f(p[3]),
                        f(p[4]),
                        f(p[5]),
                        f(p[6]),
                        f(p[7]),
                    }};

                    MarchingCubes(p, v, 0, workerPoints);
                }
            }
        }
        std::lock_guard<std::mutex> guard(mutex);
        points.insert(points.end(), workerPoints.begin(), workerPoints.end());
    };

    auto done = timed("running workers");
    RunWorkers(worker);
    done();

    done = timed("writing output");
    SaveBinarySTL("out.stl", points);
    done();

    return 0;
}
