#pragma once

std::vector<vec3> LoadBinarySTL(std::string path) {
    using namespace boost::interprocess;
    file_mapping fm(path.c_str(), read_only);
    mapped_region mr(fm, read_only);
    uint8_t *src = (uint8_t *)mr.get_address();
    const int numBytes = mr.get_size();
    const int numTriangles = std::max(0, (numBytes - 84) / 50);
    std::vector<vec3> points;
    points.reserve(numTriangles * 3);
    src += 96;
    for (int i = 0; i < numTriangles; i++) {
        const float *p = (float *)src;
        points.emplace_back(p[0], p[1], p[2]);
        points.emplace_back(p[3], p[4], p[5]);
        points.emplace_back(p[6], p[7], p[8]);
        src += 50;
    }
    return points;
}

void SaveBinarySTL(
    std::string path,
    const std::vector<vec3> &points,
    const std::vector<vec3> &colors = std::vector<vec3>{})
{
    using namespace boost::interprocess;
    const uint32_t numTriangles = points.size() / 3;
    const uint64_t numBytes = uint64_t(numTriangles) * 50 + 84;

    {
        file_mapping::remove(path.c_str());
        std::filebuf fbuf;
        fbuf.open(path.c_str(),
            std::ios_base::in | std::ios_base::out | std::ios_base::trunc |
            std::ios_base::binary);
        fbuf.pubseekoff(numBytes - 1, std::ios_base::beg);
        fbuf.sputc(0);
    }

    file_mapping fm(path.c_str(), read_write);
    mapped_region mr(fm, read_write);
    uint8_t *dst = (uint8_t *)mr.get_address();

    memcpy(dst + 80, &numTriangles, 4);

    const auto encodeColor = [](const vec3 &c) {
        const int r = std::round(glm::clamp(c.r, real(0), real(1)) * 31);
        const int g = std::round(glm::clamp(c.g, real(0), real(1)) * 31);
        const int b = std::round(glm::clamp(c.b, real(0), real(1)) * 31);
        uint16_t result = 1 << 15;
        result |= r << 10;
        result |= g << 5;
        result |= b << 0;
        return result;
    };

    for (uint32_t i = 0; i < numTriangles; i++) {
        const uint64_t idx = 84 + i * 50;
        const glm::vec3 p0 = points[i*3+0];
        const glm::vec3 p1 = points[i*3+1];
        const glm::vec3 p2 = points[i*3+2];
        const glm::vec3 normal = glm::triangleNormal(p0, p1, p2);
        memcpy(dst + idx + 0, &normal, 12);
        memcpy(dst + idx + 12, &p0, 12);
        memcpy(dst + idx + 24, &p1, 12);
        memcpy(dst + idx + 36, &p2, 12);
        if (i < colors.size()) {
            const uint16_t color = encodeColor(colors[i]);
            memcpy(dst + idx + 48, &color, 2);
        }
    }
}
