#include "shapes/plane.h"
#include "paramset.h"
#include "aa_plane.h"

namespace pbrt {

Bounds3f AAPlaneShape::ObjectBound() const {
    return { (*WorldToObject)(geometry.lo), (*WorldToObject)(geometry.hi) };
}


Float AAPlaneShape::Area() const {
    return geometry.Area();
}

bool AAPlaneShape::Intersect(const Ray &ray,
                             Float *tHit, SurfaceInteraction* isect,
                             bool testAlphaTexture) const {


    Float t;
    if (!geometry.Intersect(ray, &t)) return false;

    if (t < ray.tMax) {
        Point3f pHit = ray.o + ray.d * t;
        Vector3f error = Vector3f(0.01, 0.01, 0.01);

        Point2f uv = geometry.Uv(pHit);

        // TODO dont cheese
        Vector3f dpdu = Vector3f(-1, 0, 0);
        Vector3f dpdv = Vector3f(0, 1, 0);
        Normal3f dndu = Normal3f(0, 0, 0);
        Normal3f dndv = Normal3f(0, 0, 0);

        if (tHit) *tHit = t;
        if (isect) *isect = (*ObjectToWorld)(SurfaceInteraction(pHit, error, uv, -ray.d,
                                               dpdu, dpdv, dndu, dndv,
                                               ray.wvls, ray.time, this));
        return true;
    }

    return false;
}

Interaction AAPlaneShape::Sample(const Point2f &u, Float *pdf) const {

    Interaction it;
    it.p = geometry.Sample_wrt_Area(u, pdf);
    it.n = geometry.Normal();
    it.pError = Vector3f(0.1, 0.1, 0.1);
    *pdf = 1 / geometry.Area();
    return it;
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
