#include "sdf.h"

int main(int argc, char **argv) {
    RTCDevice device = rtcNewDevice(NULL);

    auto done = timed("initializing");

    // const real r = 500;
    // const int hx = r + 1, hy = r + 1, hz = r + 1;

    // SDF3 f = Sphere(r).Color({0, 0, 0});
    // f &= Box(vec3(r * 0.75)).Color({1, 1, 1});
    // f -= Rotate(Cylinder(r / 2).Color({1, 0, 0}), M_PI / 2, X);
    // f -= Rotate(Cylinder(r / 2).Color({0, 1, 0}), M_PI / 2, Y);
    // f -= Rotate(Cylinder(r / 2).Color({0, 0, 1}), M_PI / 2, Z);
    // f &= Plane(Z).Color({1, 0, 1});

    const int hx = 16 * 20;
    const int hy = 16 * 20;
    const int hz = 26 * 20;

    SDF3 f = Mesh(device, argv[1]).Color(0x3498DB);
    f &= Rotate(Plane(Y).Color(0xE74C3C), M_PI / 8, X);

    done();

    std::vector<vec3> points;
    std::vector<vec3> colors;
    std::mutex mutex;

    const auto worker = [&](const int wi, const int wn) {
        _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
        _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);

        std::vector<vec3> workerPoints;
        std::vector<vec3> workerColors;
        const real kHalfDiag = 0.8660254037844386;
        for (int x0 = -hx + wi; x0 < hx; x0 += wn) {
            const int x1 = x0 + 1;
            for (int y0 = -hy; y0 < hy; y0++) {
                const int y1 = y0 + 1;
                real best = 1e9;
                for (int z0 = -hz; z0 < hz; z0++) {
                    const int z1 = z0 + 1;

                    const vec3 mid(x0 + 0.5, y0 + 0.5, z0 + 0.5);
                    const real d = std::abs(f(mid));
                    best = std::min(best, d);
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

                    const int numTriangles = MarchingCubes(p, v, 0, workerPoints);

                    if (numTriangles > 0) {
                        const vec3 color = f.GetColor(mid);
                        for (int i = 0; i < numTriangles; i++) {
                            workerColors.push_back(color);
                        }
                    }
                }

                // if (best > kHalfDiag) {
                //     y0 += std::floor(best - kHalfDiag);
                // }
            }
        }
        std::lock_guard<std::mutex> guard(mutex);
        points.insert(points.end(), workerPoints.begin(), workerPoints.end());
        colors.insert(colors.end(), workerColors.begin(), workerColors.end());
    };

    done = timed("running workers");
    RunWorkers(worker);
    done();

    done = timed("writing output");
    SaveBinarySTL("out.stl", points, colors);
    done();

    return 0;
}
