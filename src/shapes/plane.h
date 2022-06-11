//
// Created by jorge on 5/26/22.
//

#ifndef PBRT_V3_PLANE_H
#define PBRT_V3_PLANE_H

#include "shape.h"

namespace pbrt {

class AAPlaneShape : public Shape {

public:
    AAPlaneShape(const Transform *ObjectToWorld, const Transform *WorldToObject,
                 bool reverseOrientation,
                 const Point3f &lo,
                 const Point3f &hi,
                 int axis,
                 bool facingFw)
            : Shape(ObjectToWorld, WorldToObject, reverseOrientation),
              lo(lo),
              hi(hi),
              facingFw(!reverseOrientation),
              ax(axis),
              ax0(axis == 2 ? 0 : (axis == 0 ? 1 : 2)),
              ax1(axis == 2 ? 1 : (axis == 0 ? 2 : 0)) {

        Point3f loWorld = (*ObjectToWorld)(lo);
        Point3f hiWorld = (*ObjectToWorld)(hi);
        area = (hiWorld[ax0] - loWorld[ax0]) * (hiWorld[ax1] - loWorld[ax1]);
    }

    bool facingFw;
    Point3f lo;
    Point3f hi;
    const int ax;
    const int ax0;
    const int ax1;
    Float area;

    // bounds
    Bounds3f ObjectBound() const override;

    // intersection
    bool Intersect(const Ray &ray, Float *tHit,
                   SurfaceInteraction *isect,
                   bool testAlphaTexture = true) const override;

    Float Area() const override;

    Normal3f Normal() const;

    // Sample a point on the surface of the shape and return the PDF with
    // respect to area on the surface.
    Interaction Sample(const Point2f &u, Float *pdf) const override;

    Float Pdf(const Interaction &) const override { return 1 / Area(); }

    Point3f V0() const;

    Point3f V1() const;

    Point3f V2() const;

    Point3f V3() const;

    bool InFront(Point3f p) const;

};

std::shared_ptr<Shape> CreateAAPlaneShape(const Transform *o2w,
                                          const Transform *w2o,
                                          bool reverseOrientation,
                                          const ParamSet &params);

}


#endif //PBRT_V3_PLANE_H
