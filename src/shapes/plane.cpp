#include "shapes/plane.h"
#include "paramset.h"

namespace pbrt {

Bounds3f AAPlaneShape::ObjectBound() const {
    return { lo, hi };
}


Float AAPlaneShape::Area() const {
    return area;
}

bool AAPlaneShape::Intersect(const Ray &ray,
                             Float *tHit, SurfaceInteraction* isect,
                             bool testAlphaTexture) const {

    // bring ray to object space
    Vector3f oErr, dErr;
    Ray rayT = (*WorldToObject)(ray, &oErr, &dErr);

    Float t = (lo[ax] - rayT.o[ax]) / rayT.d[ax];
    Point3f pHit = rayT.o + t * rayT.d;

    // check intersection in object space
    if (pHit[ax0] > lo[ax0] &&
        pHit[ax0] < hi[ax0] &&
        pHit[ax1] > lo[ax1] &&
        pHit[ax1] < hi[ax1] &&
        t < rayT.tMax) {

        Vector3f error = Vector3f(0.01, 0.01, 0.01);

        Float u = (pHit[ax0] - lo[ax0]) / (hi[ax0] - lo[ax0]);
        Float v = (pHit[ax1] - lo[ax1]) / (hi[ax1] - lo[ax1]);
        Point2f uv = Point2f(u, v);

        // TODO dont cheese
        Vector3f dpdu = Vector3f(-1, 0, 0);
        Vector3f dpdv = Vector3f(0, 1, 0);
        Normal3f dndu = Normal3f(0, 0, 0);
        Normal3f dndv = Normal3f(0, 0, 0);

        // bring isect data back to world space
        if (tHit) *tHit = t;
        if (isect) *isect = (*ObjectToWorld)(SurfaceInteraction(pHit, error, uv, -ray.d,
                                                                dpdu, dpdv, dndu, dndv,
                                                                ray.wvls, ray.time, this));
        return true;

    }

    return false;
}

Interaction AAPlaneShape::Sample(const Point2f &u, Float *pdf) const {

    Point3f loWorld = (*ObjectToWorld)(lo);
    Point3f hiWorld = (*ObjectToWorld)(hi);

    Interaction it;
    it.p[ax] = loWorld[ax];
    it.p[ax0] = loWorld[ax0] + (hiWorld[ax0] - loWorld[ax0]) * u.x;
    it.p[ax1] = loWorld[ax1] + (hiWorld[ax1] - loWorld[ax1]) * u.y;
    *pdf = 1 / Area();

    it.n = Normal();
    it.pError = Vector3f(0.1, 0.1, 0.1);
    *pdf = 1 / Area();
    return it;
}

Normal3f AAPlaneShape::Normal() const {
    Normal3f res;
    res[ax] = 1;

    if (!facingFw) {
        res *= -1;
    }

    return res;
}

Point3f AAPlaneShape::V0() const {
    return lo;
}

Point3f AAPlaneShape::V1() const {
    Point3f v1(0, 0, 0);
    v1[ax] = lo[ax];
    v1[ax0] = lo[ax0];
    v1[ax1] = hi[ax1];
    return (v1);
}

Point3f AAPlaneShape::V2() const {
    return (hi);
}

Point3f AAPlaneShape::V3() const {
    Point3f v2(0, 0, 0);
    v2[ax] = lo[ax];
    v2[ax0] = hi[ax0];
    v2[ax1] = lo[ax1];
    return (v2);
}

bool AAPlaneShape::InFront(Point3f p) const {
    if (facingFw) {
        return p[ax] > (lo)[ax];
    } else {
        return p[ax] < lo[ax];
    }
}

std::shared_ptr<Shape> CreateAAPlaneShape(
        const Transform *o2w, const Transform *w2o, bool reverseOrientation,
        const ParamSet &params) {

    // find geometry of portal
    Point3f lo = params.FindOnePoint3f("lo", Point3f(0, 0, 0));
    Point3f hi = params.FindOnePoint3f("hi", Point3f(0, 0, 0));
    int ax = params.FindOneInt("axis", 2);
    bool facingFw = params.FindOneBool("facingFw", true);

    return std::make_shared<AAPlaneShape>(o2w, w2o, reverseOrientation, lo, hi, ax, facingFw);
}

}
