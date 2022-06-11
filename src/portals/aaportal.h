#ifndef PBRT_V3_AAPORTAL_H
#define PBRT_V3_AAPORTAL_H

#include "pbrt.h"
#include "geometry.h"
#include "shapes/plane.h"
#include "portal.h"

using namespace pbrt;

class AAPortal : public Portal {
public:

    AAPortal(const Point3f& lo, const Point3f& hi, int axis, bool facingFw,
             AAPlaneShape &lightShape);

    bool InFrustum(const Point3f &p) const override;

    bool InFront(const Point3f &p) const override;

    void SamplePortal(const Interaction &ref,
                      const Point2f &u,
                      Vector3f *wi,
                      Float *pdf) const override;

    Float Pdf_Portal(const Interaction &ref,
                     const Vector3f &wi) const override;

    void SampleProj(const Interaction &ref,
                    const Point2f &u,
                    Vector3f *wi, Float *pdf) const override;

    Float Pdf_Proj(const Interaction &ref,
                   const Vector3f &wi) const override;

    // light geometry
    const AAPlane& light;

    // portal geometry
    const AAPlane portal;

    Normal3f n;
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

#endif //PBRT_V3_AAPORTAL_H
