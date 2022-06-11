#include "aaportal.h"
#include "shapes/triangle.h"
#include "shapes/plane.h"
#include "geometry.h"
#include "interaction.h"
#include "ext/sexpresso.hpp"

AAPortal::AAPortal(const Point3f &lo, const Point3f &hi,
                   int axis, bool facingFw,
                   AAPlaneShape &light) :
                   // TODO fix
        portal(AAPlaneShape(light.ObjectToWorld, light.WorldToObject, !facingFw, lo, hi, axis, facingFw)),
        light(light) {

    // Calculate viewing frustum
    Point3f p0 = portal.V0();
    Point3f p1 = portal.V1();
    Point3f p2 = portal.V2();
    Point3f p3 = portal.V3();

//    LOG(INFO) << "DBG POINT:" << light.V0() << "; li0";
//    LOG(INFO) << "DBG POINT:" << light.V1() << "; li1";
//    LOG(INFO) << "DBG POINT:" << light.V2() << "; li2";
//    LOG(INFO) << "DBG POINT:" << light.V3() << "; li3";
//
//    LOG(INFO) << "DBG POINT:" << p0 << "; p0";
//    LOG(INFO) << "DBG POINT:" << p1 << "; p1";
//    LOG(INFO) << "DBG POINT:" << p2 << "; p2";
//    LOG(INFO) << "DBG POINT:" << p3 << "; p3";

    // Frustum diagonal directions
    auto fd0 = Normalize(p0 - light.V2());
    auto fd1 = Normalize(p1 - light.V3());
    auto fd2 = Normalize(p2 - light.V0());
    auto fd3 = Normalize(p3 - light.V1());

    // frustum plane normals (z axis)
    if (!facingFw) {
        fn0 = Normal3f(Cross(fd1, fd0));
        fn1 = Normal3f(Cross(fd2, fd1));
        fn2 = Normal3f(Cross(fd3, fd2));
        fn3 = Normal3f(Cross(fd0, fd3));
    } else {
        fn0 = Normal3f(Cross(fd0, fd1));
        fn1 = Normal3f(Cross(fd1, fd2));
        fn2 = Normal3f(Cross(fd2, fd3));
        fn3 = Normal3f(Cross(fd3, fd0));
    }

    // frustum plane points
    fp0 = (p0 + p1) / 2;
    fp1 = (p1 + p2) / 2;
    fp2 = (p2 + p3) / 2;
    fp3 = (p3 + p0) / 2;

//    LOG(INFO) << "DBG POINT:" << fp0;
//    LOG(INFO) << "DBG POINT:" << fp1;
//    LOG(INFO) << "DBG POINT:" << fp2;
//    LOG(INFO) << "DBG POINT:" << fp3;
//
//    LOG(INFO) << "DBG DIR-WHITE:" << p0 << ";" << fd0 << ";p0";
//    LOG(INFO) << "DBG DIR-BLUE:" << p1 << ";" << fd1 << ";p1";
//    LOG(INFO) << "DBG DIR-GREEN:" << p2 << ";" << fd2 << ";p2";
//    LOG(INFO) << "DBG DIR-RED:" << p3 << ";" << fd3 << ";p3";
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
    Float areaPdf;
    Point3f sampledPoint = portal.Sample(u, &areaPdf).p;

    *wi = Normalize(sampledPoint - ref.p);
    *pdf = DistanceSquared(ref.p, sampledPoint) / (AbsDot(portal.Normal(), -*wi) * portal.Area());
}

Float AAPortal::Pdf_Portal(const Interaction &ref, const Vector3f &wi) const {

    // ax aligned across z ax, plane intersection
    Ray r = ref.SpawnRay(wi);
    Float tHit;
    if (portal.IntersectP(r)) {
        return DistanceSquared(ref.p, (r.o + r.d * tHit)) / (AbsDot(portal.Normal(), -Normalize(wi)) * portal.Area());
    }

    return 0;
}

bool AAPortal::InFront(const Point3f &p) const {
    return portal.InFront((*light.WorldToObject)(p));
}

bool AAPortal::InFrustum(const Point3f &p) const {

    return true;

    bool res = true;
    res &= Dot(fp0 - p, fn0) >= 0;
    res &= Dot(fp1 - p, fn1) >= 0;
    res &= Dot(fp2 - p, fn2) >= 0;
    res &= Dot(fp3 - p, fn3) >= 0;

    return res;
}

void AAPortal::SampleProj(const Interaction &ref, const Point2f &u,
                          Vector3f *wi, Float *pdf) const {

    // bring reference point to world space
    Point3f pRef = (*portal.WorldToObject)(ref.p);

    // TODO make efficient :))
    // can make it faster by only doing math w x and y, not Z

    auto dLo = Normalize(ref.p - light.lo);
    auto dHi = Normalize(ref.p - light.hi);

    if (dLo.z == 0 || dHi.z == 0) {
        *pdf = 0;
        return;
    }

    Float tLo = (portal.lo[portal.ax] - light.lo[light.ax]) / dLo[light.ax];
    Float tHi = (portal.lo[portal.ax] - light.hi[light.ax]) / dHi[light.ax];

    Point3f projLo = light.lo + dLo * tLo;
    Point3f projHi = light.hi + dHi * tHi;

    // compute intersection
    Point3f isectHi = Max(portal.lo, projLo);
    Point3f isectLo = Min(portal.hi, projHi);


    // TODO: possibly check for bad bounds (or maybe this is slower?)

    // sample the projection intersection
    Float isectLen0 = isectHi[portal.ax0] - isectLo[portal.ax0];
    Float isectLen1 = isectHi[portal.ax1] - isectLo[portal.ax1];

    Point3f sampled;
    sampled[portal.ax] = portal.lo[portal.ax];
    sampled[portal.ax0] = isectLo[portal.ax0] + u.x * isectLen0;
    sampled[portal.ax1] = isectLo[portal.ax1] + u.x * isectLen1;

    // bring back to world space
    Point3f sampledWorld = (*portal.WorldToObject)(sampled);
    LOG(INFO) << "DBG POINT:" << sampledWorld;

    *wi = sampledWorld - ref.p;
    *pdf = DistanceSquared(ref.p, sampled) / (AbsDot(portal.Normal(), -*wi) * (isectLen0 * isectLen1));
}

Float AAPortal::Pdf_Proj(const Interaction &ref, const Vector3f &wi) const {
    // TODO implement for MIS
    return 0;
}
