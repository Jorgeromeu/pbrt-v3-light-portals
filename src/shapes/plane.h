//
// Created by jorge on 5/26/22.
//

#ifndef PBRT_V3_PLANE_H
#define PBRT_V3_PLANE_H

#include "shape.h"

namespace pbrt {

class AAPlane : public Shape {

public:
    AAPlane(const Transform *ObjectToWorld, const Transform *WorldToObject,
            bool reverseOrientation,
            const Float &loY,
            const Float &hiY,
            const Float &loX,
            const Float &hiX,
            const Float &z,
            const Normal3f &normal)
            : Shape(ObjectToWorld, WorldToObject, reverseOrientation),
              loX(loX),
              hiX(hiX),
              loY(loY),
              hiY(hiY),
              n(normal),
              z(z) {}

    // bounds
    Bounds3f ObjectBound() const;

    // intersection
    bool Intersect(const Ray &ray, Float *tHit,
                           SurfaceInteraction *isect,
                           bool testAlphaTexture = true) const;

    Float Area() const;

    // Sample a point on the surface of the shape and return the PDF with
    // respect to area on the surface.
    Interaction Sample(const Point2f &u, Float *pdf) const;

    Float Pdf(const Interaction &) const { return 1 / Area(); }

    // portal data
    const Float loX;
    const Float hiX;
    const Float loY;
    const Float hiY;
    const Float z;
    const Normal3f n;

};

std::shared_ptr<Shape> CreateAAPlaneShape(const Transform *o2w,
                                          const Transform *w2o,
                                          bool reverseOrientation,
                                          const ParamSet &params);

}


#endif //PBRT_V3_PLANE_H
