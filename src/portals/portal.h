#include "interaction.h"

#ifndef PBRT_V3_PORTAL_H
#define PBRT_V3_PORTAL_H

namespace pbrt {

class Portal {

public:

    // for determining if we should sample the portal or not
    virtual bool InFrustum(const Point3f &p) const = 0;

    virtual bool InFront(const Point3f &p) const = 0;

    // Uniform portal sampling
    virtual void SamplePortal(const Interaction &ref,
                      const Point2f &u,
                      Vector3f *wi,
                      Float *pdf) const = 0;

    virtual Float Pdf_Portal(const Interaction &ref,
                     const Vector3f &wi) const = 0;

    // Projection Sampling
    virtual void SampleProj(const Interaction &ref,
                    const Point2f &u,
                    Vector3f *wi, Float *pdf) const = 0;

    virtual Float Pdf_Proj(const Interaction &ref,
                   const Vector3f &wi) const = 0;

};

}



#endif //PBRT_V3_PORTAL_H
