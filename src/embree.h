#pragma once

typedef struct {
    float x, y, z;
} EmbreeVertex;

typedef struct {
    int v0, v1, v2;
} EmbreeTriangle;

std::pair<vec3, vec3> closestPointTriangle(
    const vec3 &p,
    const vec3 &a, const vec3 &b, const vec3 &c,
    const vec3 &na, const vec3 &nb, const vec3 &nc,
    const vec3 &nab, const vec3 &nac, const vec3 &nbc)
{
    const vec3 ab = b - a;
    const vec3 ac = c - a;
    const vec3 ap = p - a;

    const real d1 = glm::dot(ab, ap);
    const real d2 = glm::dot(ac, ap);
    if (d1 <= 0 && d2 <= 0) {
        return std::make_pair(a, na);
    }

    const vec3 bp = p - b;
    const real d3 = glm::dot(ab, bp);
    const real d4 = glm::dot(ac, bp);
    if (d3 >= 0 && d4 <= d3) {
        return std::make_pair(b, nb);
    }

    const vec3 cp = p - c;
    const real d5 = glm::dot(ab, cp);
    const real d6 = glm::dot(ac, cp);
    if (d6 >= 0 && d5 <= d6) {
        return std::make_pair(c, nc);
    }

    const real vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) {
        const real v = d1 / (d1 - d3);
        return std::make_pair(a + v * ab, nab);
    }

    const real vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) {
        const real v = d2 / (d2 - d6);
        return std::make_pair(a + v * ac, nac);
    }

    const real va = d3 * d6 - d5 * d4;
    if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
        const real v = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return std::make_pair(b + v * (c - b), nbc);
    }

    const real denom = 1 / (va + vb + vc);
    const real v = vb * denom;
    const real w = vc * denom;
    return std::make_pair(a + v * ab + w * ac, glm::triangleNormal(a, b, c));
}

struct ClosestPointResult {
    ClosestPointResult() :
        primID(RTC_INVALID_GEOMETRY_ID),
        geomID(RTC_INVALID_GEOMETRY_ID) {}

    vec3 p;
    real d;
    unsigned int primID;
    unsigned int geomID;

    EmbreeVertex *vertexBuf;
    EmbreeTriangle *triangleBuf;
    EmbreeVertex *normalBuf;
    EmbreeVertex *edgeNormalBuf;
};

SDF3 Mesh(const RTCDevice device, const std::string &path) {
    // load the stl
    const std::vector<vec3> data = LoadBinarySTL(path);

    // deduplicate vertices
    std::vector<vec3> positions;
    std::vector<ivec3> triangles;
    std::unordered_map<vec3, int> lookup;
    for (const vec3 &v : data) {
        if (lookup.find(v) == lookup.end()) {
            lookup[v] = positions.size();
            positions.push_back(v);
        }
    }

    // create triangles
    for (int i = 0; i < data.size(); i += 3) {
        const int i0 = lookup[data[i + 0]];
        const int i1 = lookup[data[i + 1]];
        const int i2 = lookup[data[i + 2]];
        triangles.emplace_back(i0, i1, i2);
    }

    // compute angle-weighted pseudonormals
    std::vector<vec3> normals(positions.size(), vec3{0});
    std::unordered_map<std::pair<int, int>, vec3, boost::hash<std::pair<int, int>>> edgeNormalsMap;
    for (const ivec3 &t : triangles) {
        const vec3 &a = positions[t.x];
        const vec3 &b = positions[t.y];
        const vec3 &c = positions[t.z];
        const vec3 n = glm::triangleNormal(a, b, c);
        const vec3 ab = glm::normalize(b - a);
        const vec3 ac = glm::normalize(c - a);
        const vec3 bc = glm::normalize(c - b);
        const real thetaA = std::acos(std::clamp(glm::dot(ab, ac), real(-1), real(1)));
        const real thetaB = std::acos(std::clamp(glm::dot(-ab, bc), real(-1), real(1)));
        const real thetaC = std::acos(std::clamp(glm::dot(-ac, -bc), real(-1), real(1)));
        normals[t.x] += n * thetaA;
        normals[t.y] += n * thetaB;
        normals[t.z] += n * thetaC;
        edgeNormalsMap[std::make_pair(std::min(t.x, t.y), std::max(t.x, t.y))] += n;
        edgeNormalsMap[std::make_pair(std::min(t.x, t.z), std::max(t.x, t.z))] += n;
        edgeNormalsMap[std::make_pair(std::min(t.y, t.z), std::max(t.y, t.z))] += n;
    }
    for (int i = 0; i < normals.size(); i++) {
        normals[i] = glm::normalize(normals[i]);
    }
    std::vector<vec3> edgeNormals(triangles.size() * 3, vec3{0});
    for (int i = 0; i < triangles.size(); i++) {
        const ivec3 &t = triangles[i];
        edgeNormals[i*3+0] = glm::normalize(edgeNormalsMap[std::make_pair(std::min(t.x, t.y), std::max(t.x, t.y))]);
        edgeNormals[i*3+1] = glm::normalize(edgeNormalsMap[std::make_pair(std::min(t.x, t.z), std::max(t.x, t.z))]);
        edgeNormals[i*3+2] = glm::normalize(edgeNormalsMap[std::make_pair(std::min(t.y, t.z), std::max(t.y, t.z))]);
    }

    RTCScene scene = rtcNewScene(device);
    RTCGeometry geom = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);

    // TODO: memory leaks

    EmbreeVertex *vertexBuf = (EmbreeVertex *)rtcSetNewGeometryBuffer(
        geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3,
        sizeof(EmbreeVertex), positions.size());
    for (int i = 0; i < positions.size(); i++) {
        const auto &p = positions[i];
        vertexBuf[i].x = (p.x - 0) * 10;
        vertexBuf[i].y = (p.y - 0) * 10;
        vertexBuf[i].z = (p.z - 24) * 10;
    }

    EmbreeTriangle *triangleBuf = (EmbreeTriangle *)rtcSetNewGeometryBuffer(
        geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3,
        sizeof(EmbreeTriangle), triangles.size());
    for (int i = 0; i < triangles.size(); i++) {
        const auto &t = triangles[i];
        triangleBuf[i].v0 = t.x;
        triangleBuf[i].v1 = t.y;
        triangleBuf[i].v2 = t.z;
    }

    EmbreeVertex *normalBuf = (EmbreeVertex *)malloc(sizeof(EmbreeVertex) * normals.size());
    for (int i = 0; i < normals.size(); i++) {
        const auto &n = normals[i];
        normalBuf[i].x = n.x;
        normalBuf[i].y = n.y;
        normalBuf[i].z = n.z;
    }

    EmbreeVertex *edgeNormalBuf = (EmbreeVertex *)malloc(sizeof(EmbreeVertex) * edgeNormals.size());
    for (int i = 0; i < edgeNormals.size(); i++) {
        const auto &n = edgeNormals[i];
        edgeNormalBuf[i].x = n.x;
        edgeNormalBuf[i].y = n.y;
        edgeNormalBuf[i].z = n.z;
    }

    const RTCPointQueryFunction closestPointFunc = [](RTCPointQueryFunctionArguments *args) -> bool {
        ClosestPointResult *result = (ClosestPointResult *)args->userPtr;
        const EmbreeTriangle &triangle = result->triangleBuf[args->primID];
        const EmbreeVertex &v0 = result->vertexBuf[triangle.v0];
        const EmbreeVertex &v1 = result->vertexBuf[triangle.v1];
        const EmbreeVertex &v2 = result->vertexBuf[triangle.v2];
        const EmbreeVertex &n0 = result->normalBuf[triangle.v0];
        const EmbreeVertex &n1 = result->normalBuf[triangle.v1];
        const EmbreeVertex &n2 = result->normalBuf[triangle.v2];
        const EmbreeVertex &n01 = result->edgeNormalBuf[args->primID * 3 + 0];
        const EmbreeVertex &n02 = result->edgeNormalBuf[args->primID * 3 + 1];
        const EmbreeVertex &n12 = result->edgeNormalBuf[args->primID * 3 + 2];
        // TODO: silly copies
        const vec3 a(v0.x, v0.y, v0.z);
        const vec3 b(v1.x, v1.y, v1.z);
        const vec3 c(v2.x, v2.y, v2.z);
        const vec3 na(n0.x, n0.y, n0.z);
        const vec3 nb(n1.x, n1.y, n1.z);
        const vec3 nc(n2.x, n2.y, n2.z);
        const vec3 nab(n01.x, n01.y, n01.z);
        const vec3 nac(n02.x, n02.y, n02.z);
        const vec3 nbc(n12.x, n12.y, n12.z);
        const vec3 q(args->query->x, args->query->y, args->query->z);
        const auto closest = closestPointTriangle(q, a, b, c, na, nb, nc, nab, nac, nbc);
        const vec3 p = closest.first;
        const vec3 n = closest.second;
        const real d = glm::distance(p, q);
        if (d < args->query->radius) {
            args->query->radius = d;
            result->p = p;
            result->d = d;
            if (glm::dot(q - p, n) < 0) {
                result->d = -d;
            }
            result->primID = args->primID;
            result->geomID = args->geomID;
            return true;
        }
        return false;
    };

    rtcSetGeometryPointQueryFunction(geom, closestPointFunc);
    rtcCommitGeometry(geom);
    rtcAttachGeometry(scene, geom);
    rtcReleaseGeometry(geom);
    rtcCommitScene(scene);

    return [=](const vec3 &p) -> real {

        RTCPointQuery query;
        query.x = p.x; 
        query.y = p.y;
        query.z = p.z;
        query.radius = std::numeric_limits<float>::infinity();
        query.time = 0.f;

        ClosestPointResult result;
        result.vertexBuf = vertexBuf;
        result.triangleBuf = triangleBuf;
        result.normalBuf = normalBuf;
        result.edgeNormalBuf = edgeNormalBuf;
        RTCPointQueryContext context;
        rtcInitPointQueryContext(&context);
        rtcPointQuery(scene, &query, &context, nullptr, (void *)&result);
        assert(result.primID != RTC_INVALID_GEOMETRY_ID || result.geomID != RTC_INVALID_GEOMETRY_ID);

        return result.d;
    };
}
