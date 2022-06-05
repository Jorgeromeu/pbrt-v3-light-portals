#include "shapes/plane.h"
#include "paramset.h"

namespace pbrt {

Bounds3f AAPlane::ObjectBound() const {
    return { (*WorldToObject)(lo), (*WorldToObject)(hi) };
}


Float AAPlane::Area() const {
    Float area =  (hi[ax0] - lo[ax0]) * (hi[ax1] - lo[ax1]);
    return area;
}

bool AAPlane::Intersect(const Ray &ray,
                        Float *tHit, SurfaceInteraction* isect,
                        bool testAlphaTexture) const {

    // ray-aa-plane intersection
    float t = (lo[ax] - ray.o[ax]) / ray.d[ax];
    Point3f pHit = ray.o + t * ray.d;

    if (pHit[ax0] > lo[ax0] &&
        pHit[ax0] < hi[ax0] &&
        pHit[ax1] > lo[ax1] &&
        pHit[ax1] < hi[ax1] &&
        t < ray.tMax &&
        t > 0) {

        Vector3f error = Vector3f(0.01, 0.01, 0.01);

        Float u = (pHit[ax0] - lo[ax0]) / (hi[ax0] - lo[ax0]);
        Float v = (pHit[ax1] - lo[ax1]) / (hi[ax1] - lo[ax1]);
        Point2f uv = Point2f(u, v);

        Vector3f dpdu = Vector3f(-1, 0, 0);
        Vector3f dpdv = Vector3f(0, 1, 0);

        Normal3f dndu = Normal3f(0, 0, 0);
        Normal3f dndv = Normal3f(0, 0, 0);

        if (tHit) *tHit = t;
        if (isect) *isect = SurfaceInteraction(pHit, error, uv, -ray.d,
                                    dpdu, dpdv, dndu, dndv,
                                    ray.wvls, ray.time, this);

        return true;
    }

    return false;
}

Interaction AAPlane::Sample(const Point2f &u, Float *pdf) const {

    Interaction it;
    it.p = Point3f(0, 0, 0);
    it.p[ax] = lo[ax];
    it.p[ax0] = lo[ax0] + (hi[ax0] - lo[ax0]) * u.x;
    it.p[ax1] = lo[ax1] + (hi[ax1] - lo[ax1]) * u.y;

    it.n = n;
    it.pError = Vector3f(0.1, 0.1, 0.1);

    *pdf = 1 / Area();
    return it;
}

std::shared_ptr<Shape> CreateAAPlaneShape(
        const Transform *o2w, const Transform *w2o, bool reverseOrientation,
        const ParamSet &params) {

    // find geometry of portal
    Point3f lo = params.FindOnePoint3f("lo", Point3f(0, 0, 0));
    Point3f hi = params.FindOnePoint3f("hi", Point3f(0, 0, 0));
    int ax = params.FindOneInt("axis", 2);
    Normal3f n = params.FindOneNormal3f("n", Normal3f(1, 0, 0));

    return std::make_shared<AAPlane>(o2w, w2o, reverseOrientation, lo, hi, ax, n);
}

}
