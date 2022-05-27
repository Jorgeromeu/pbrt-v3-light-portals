#ifndef PBRT_V3_PORTAL_H
#define PBRT_V3_PORTAL_H

#include "pbrt.h"
#include "geometry.h"

using namespace pbrt;

class Portal {
public:

    Portal(const Float loY, const Float hiY,
           const Float loX, const Float hiX, const Float z,
           const Normal3f &n);

    void SamplePortal(const Interaction &ref,
                      const Point2f &u,
                      Vector3f *wi,
                      Float *pdf) const;

    Float Pdf_Portal(const Interaction &ref,
                     const Vector3f &wi) const;

    const Float loY;
    const Float hiY;
    const Float loX;
    const Float hiX;
    const Float z;
    const Normal3f n;
    const Point3f center;
    const Float area;
};


#endif //PBRT_V3_PORTAL_H
