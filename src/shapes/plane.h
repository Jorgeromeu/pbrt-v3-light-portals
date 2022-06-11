//
// Created by jorge on 5/26/22.
//

#ifndef PBRT_V3_PLANE_H
#define PBRT_V3_PLANE_H

#include "shape.h"
#include "core/aa_plane.h"

namespace pbrt {

class AAPlaneShape : public Shape {

public:
    AAPlaneShape(const Transform *ObjectToWorld, const Transform *WorldToObject,
                 bool reverseOrientation,
                 Point3f &lo,
                 Point3f &hi,
                 int axis,
                 bool facingFw)
            : Shape(ObjectToWorld, WorldToObject, reverseOrientation),
              geometry(AAPlane(lo, hi, axis, facingFw)) {
    }

    // bounds
    Bounds3f ObjectBound() const override;

    // intersection
    bool Intersect(const Ray &ray, Float *tHit,
                   SurfaceInteraction *isect,
                   bool testAlphaTexture = true) const override;

    Float Area() const override;

    // Sample a point on the surface of the shape and return the PDF with
    // respect to area on the surface.
    Interaction Sample(const Point2f &u, Float *pdf) const override;

    Float Pdf(const Interaction &) const override { return 1 / Area(); }

    const AAPlane geometry;


};

std::shared_ptr<Shape> CreateAAPlaneShape(const Transform *o2w,
                                          const Transform *w2o,
                                          bool reverseOrientation,
                                          const ParamSet &params);

}


#endif //PBRT_V3_PLANE_H
