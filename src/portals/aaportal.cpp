#include "aaportal.h"
#include "shapes/triangle.h"
#include "shapes/plane.h"
#include "geometry.h"
#include "interaction.h"
#include "ext/sexpresso.hpp"

AAPortal::AAPortal(const Float loY, const Float hiY,
                   const Float loX, const Float hiX, const Float z,
                   bool greater,
                   AAPlane &light) :
        loY(loY),
        hiY(hiY),
        hiX(hiX),
        loX(loX),
        z(z),
        greater(greater),
        light(light),
        // compute portal center area and normal here so its cached for future lookups
        n(greater ? Normal3f(0, 0, 1) : Normal3f(0, 0, -1)),
        area(((hiX - loX) * (hiY - loY))) {

    // Calculate viewing frustum
    // TODO MAKE AGNOSTIC!!
    Point3f l0 = light.lo;
    Point3f l1 = Point3f(light.lo.x, light.hi.y, light.lo.z);
    Point3f l2 = Point3f(light.hi.x, light.lo.x, light.lo.z);
    Point3f l3 = light.hi;


    Point3f p0 = Point3f(loX, loY, z);
    Point3f p1 = Point3f(loX, hiY, z);
    Point3f p2 = Point3f(hiX, loY, z);
    Point3f p3 = Point3f(hiX, hiY, z);

    // directions
    auto fd3 = Normalize(p3 - l0);
    auto fd2 = Normalize(p2 - l1);
    auto fd1 = Normalize(p1 - l2);
    auto fd0 = Normalize(p0 - l3);

    // frustum directions
//    LOG(INFO) << "DBG DIR-WHITE:" << p0 << ";" << fd0;
//    LOG(INFO) << "DBG DIR-BLUE:" << p1 << ";" << fd1;
//    LOG(INFO) << "DBG DIR-GREEN:" << p2 << ";" << fd2;
//    LOG(INFO) << "DBG DIR-RED:" << p3 << ";" << fd3;

    // frustum plane normals
    if (!greater) {
        fn0 = Normal3f(Cross(fd1, fd0));
        fn1 = Normal3f(Cross(fd3, fd1));
        fn2 = Normal3f(Cross(fd2, fd3));
        fn3 = Normal3f(Cross(fd0, fd2));
    } else {
        fn0 = Normal3f(Cross(fd0, fd1));
        fn1 = Normal3f(Cross(fd1, fd3));
        fn2 = Normal3f(Cross(fd3, fd2));
        fn3 = Normal3f(Cross(fd2, fd0));
    }

    // frustum plane points
    fp0 = (p0 + p1) / 2;
    fp1 = (p1 + p3) / 2;
    fp2 = (p2 + p3) / 2;
    fp3 = (p0 + p2) / 2;
//    LOG(INFO) << "DBG DIR-CYAN:" << fp0 << ";" << fn0;
//    LOG(INFO) << "DBG DIR-CYAN:" << fp1 << ";" << fn1;
//    LOG(INFO) << "DBG DIR-CYAN:" << fp2 << ";" << fn2;
//    LOG(INFO) << "DBG DIR-CYAN:" << fp3 << ";" << fn3;
}

// AAPortal area sapling
void AAPortal::SamplePortal(const Interaction &ref,
                            const Point2f &u,
                            Vector3f *wi, Float *pdf) const {

    // sample portal uniformly
    float randY = loY + u.y * (hiY - loY);
    float randX = loX + u.x * (hiX - loX);
    Point3f sampledPoint = Point3f(randX, randY, this->z);

    *wi = Normalize(sampledPoint - ref.p);
    *pdf = DistanceSquared(ref.p, sampledPoint) / (AbsDot(this->n, -*wi) * area);
}

Float AAPortal::Pdf_Portal(const Interaction &ref, const Vector3f &wi) const {

    // ax aligned across z ax, plane intersection
    Ray r = ref.SpawnRay(wi);

    if (r.d.z == 0) return 0;

    float t = (z - r.o.z) / r.d.z;
    Point3f isect = r.o + r.d * t;

    // if hit plane
    if (isect.y >= loY && isect.y <= hiY && isect.x >= loX && isect.x <= hiX) {
        return DistanceSquared(ref.p, isect) / (AbsDot(n, -Normalize(wi)) * area);
    }

    return 0;
}

bool AAPortal::InFront(const Point3f& p) const {
    if (greater) {
        return p.z > z;
    } else {
        return p.z < z;
    }
}

bool AAPortal::InFrustum(const Point3f &p) const {
    bool res = true;
    res &= Dot(fp0 - p, fn0) >= 0;
    res &= Dot(fp1 - p, fn1) >= 0;
    res &= Dot(fp2 - p, fn2) >= 0;
    res &= Dot(fp3 - p, fn3) >= 0;

    return res;
}

void AAPortal::SampleProj(const Interaction &ref, const Point2f &u,
                          Vector3f *wi, Float *pdf) const {

    // TODO make efficient :)
    // can make it faster by only doing math w x and y, not Z

    auto dLo = Normalize(ref.p - light.lo);
    auto dHi = Normalize(ref.p - light.hi);

    if (dLo.z == 0 || dHi.z == 0) {
        *pdf = 0;
        return;
    }

    Float tLo = (z - light.lo.z) / dLo.z;
    Float tHi = (z - light.hi.z) / dHi.z;

    Point3f projLo = light.lo + dLo * tLo;
    Point3f projHi = light.hi + dHi * tHi;

    // compute intersection
    Point3f isectHi = Max(Point3f(loX, loY, z), projLo);
    Point3f isectLo = Min(Point3f(hiX, hiY, z), projHi);

    // TODO: possibly check for bad bounds (or maybe this is slower?)

    Float isectLenX = isectHi.x - isectLo.x;
    Float isectLenY = isectHi.y - isectLo.y;

    Float randX = isectLo.x + u.x * isectLenX;
    Float randY = isectLo.y + u.y * isectLenY;
    Point3f sampled = Point3f(randX, randY, z);

    *wi = Normalize(sampled - ref.p);
    *pdf = DistanceSquared(ref.p, sampled) / (AbsDot(n, -*wi) * isectLenX * isectLenY);
}
Float AAPortal::Pdf_Proj(const Interaction &ref, const Vector3f &wi) const {
    // TODO implement for MIS
    return 0;
}
