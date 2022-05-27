#include "shapes/plane.h"
#include "paramset.h"

namespace pbrt {

// bounds
Bounds3f AAPlane::ObjectBound() const {
    const Point3f &pLo = Point3f(loX, loY, z-0.01);
    const Point3f &pHi = Point3f(hiX, hiY, z+0.01);
    return {(*WorldToObject)(pLo), (*WorldToObject)(pHi) };
}


Float AAPlane::Area() const {
    return (hiY - loY) * (hiX - loX);
}

bool AAPlane::Intersect(const Ray &ray,
                        Float *tHit, SurfaceInteraction* isect,
                        bool testAlphaTexture) const {

    // ray-aa-plane intersection
    float t = (z - ray.o.z) / ray.d.z;
    Point3f pHit = ray.o + t * ray.d;

    if (pHit.x > loX &&
        pHit.x < hiX &&
        pHit.y > loY &&
        pHit.y < hiY &&
        t < ray.tMax &&
        t > 0) {

        Vector3f error = Vector3f(0.01, 0.01, 0.01);

        Float u = (pHit.x - loX) / (hiX - loX);
        Float v = (pHit.y - loY) / (hiY - loY);
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

    Float randX = loX + (hiX - loX) * u.x;
    Float randY = loY + (hiY - loY) * u.y;

    Interaction it;
    it.p = Point3f(randX, randY, z);
    it.n = n;
    it.pError = Vector3f(0.1, 0.1, 0.1);

    *pdf = 1 / Area();
    return it;
}


std::shared_ptr<Shape> CreateAAPlaneShape(
        const Transform *o2w, const Transform *w2o, bool reverseOrientation,
        const ParamSet &params) {

    // find geometry of portal
    Float loY = params.FindOneFloat("loY", 0);
    Float hiY = params.FindOneFloat("hiY", 0);
    Float loX = params.FindOneFloat("loX", 0);
    Float hiX = params.FindOneFloat("hiX", 0);
    Float z = params.FindOneFloat("z", 0);
    Normal3f n = params.FindOneNormal3f("n", Normal3f(1, 0, 0));

    return std::make_shared<AAPlane>(o2w, w2o, reverseOrientation, loY, hiY, loX, hiX, z, n);
}

}
