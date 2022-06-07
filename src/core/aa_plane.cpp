#include "aa_plane.h"

namespace pbrt {

Point3f AAPlane::V0() const {
    return lo;
}

Point3f AAPlane::V1() const {
    Point3f v1(0, 0, 0);
    v1[ax] = lo[ax];
    v1[ax0] = lo[ax0];
    v1[ax1] = hi[ax1];
    return v1;
}

Point3f AAPlane::V2() const {
    return hi;
}

Point3f AAPlane::V3() const {
    Point3f v2(0, 0, 0);
    v2[ax] = lo[ax];
    v2[ax0] = hi[ax0];
    v2[ax1] = lo[ax1];
    return v2;
}

Point3f AAPlane::Sample_wrt_Area(const Point2f &u, Float *pdf) const {
    Point3f p;
    p[ax] = lo[ax];
    p[ax0] = lo[ax0] + (hi[ax0] - lo[ax0]) * u.x;
    p[ax1] = lo[ax1] + (hi[ax1] - lo[ax1]) * u.y;
    *pdf = 1 / area;
    return p;
}

Point3f AAPlane::Sample_wrt_SolidAngle(const Point3f& ref,
                                       const Point2f &u,
                                       Vector3f* wi, Float *pdf) const {
    Point3f sampled;
    sampled[ax] = lo[ax];
    sampled[ax0] = lo[ax0] + (hi[ax0] - lo[ax0]) * u.x;
    sampled[ax1] = lo[ax1] + (hi[ax1] - lo[ax1]) * u.y;

    *wi = Normalize(sampled - ref);
    *pdf = DistanceSquared(ref, sampled) / (AbsDot(Normal(), -*wi) * Area());

    return sampled;
}
Float AAPlane::Area() const {
    return area;
}

Normal3f AAPlane::Normal() const {
    Normal3f res;
    res[ax] = 1;

    if (!facingFw) {
        res *= -1;
    }

    return res;
}

bool AAPlane::Intersect(const Ray &ray, Float *tHit) const {

    // convert ray to object space
    Vector3f oErr, dErr;
    Ray rayT = (*w2o)(ray, &oErr, &dErr);

    Float t = (lo[ax] - rayT.o[ax]) / ray.d[ax];
    Point3f pHit = rayT.o + t * rayT.d;

    // check intersection in object space
    if (pHit[ax0] > (*w2o)(lo)[ax0] &&
        pHit[ax0] < (*w2o)(hi)[ax0] &&
        pHit[ax1] > (*w2o)(lo)[ax1] &&
        pHit[ax1] < (*w2o)(hi)[ax1]) {

        if (tHit) *tHit = t;
        return true;
    }

    return false;
}

Point2f AAPlane::Uv(Point3f p) const {
    Float u = (p[ax0] - lo[ax0]) / (hi[ax0] - lo[ax0]);
    Float v = (p[ax1] - lo[ax1]) / (hi[ax1] - lo[ax1]);
    return {u, v};
}

bool AAPlane::InFront(Point3f p) const {

    if (facingFw) {
        return p[ax] > lo[ax];
    } else {
        return p[ax] < lo[ax];
    }
}

}