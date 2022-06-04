#ifndef PBRT_V3_PORTAL_H
#define PBRT_V3_PORTAL_H

#include "pbrt.h"
#include "geometry.h"
#include "shapes/plane.h"

using namespace pbrt;

class Portal {
public:

    Portal(const Float loY, const Float hiY,
           const Float loX, const Float hiX, const Float z,
           bool greater,
           AAPlane &light);

    Portal(const std::string &data, const AAPlane &light);

    void SamplePortal(const Interaction &ref,
                      const Point2f &u,
                      Vector3f *wi,
                      Float *pdf) const;

    Float Pdf_Portal(const Interaction &ref,
                     const Vector3f &wi) const;

    bool InFrustrum(const Point3f &p) const;

    bool InFront(const Point3f &p) const;

    void SampleProj(const Interaction &ref,
                    const Point2f &u,
                    Vector3f *wi, Float *pdf) const;

    // light geometry
    const AAPlane& light;

    // portal geometry
    Float loY;
    Float hiY;
    Float loX;
    Float hiX;
    Float z;
    bool greater;

    Normal3f n;
    Point3f center;
    Float area;

    // frustum data
    // point and normal for each frustum plane
    Normal3f fn0;
    Normal3f fn1;
    Normal3f fn2;
    Normal3f fn3;
    Point3f fp0;
    Point3f fp1;
    Point3f fp2;
    Point3f fp3;
};

#endif //PBRT_V3_PORTAL_H
