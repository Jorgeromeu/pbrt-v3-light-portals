#include "portal.h"
#include "shapes/triangle.h"
#include "geometry.h"
#include "interaction.h"

Portal::Portal(const Float loY, const Float hiY,
               const Float loX, const Float hiX, const Float z,
               const Normal3f &n) :
        loY(loY),
        hiY(hiY),
        hiX(hiX),
        loX(loX),
        z(z),
        n(n),
        // compute center and area here so its cached for future lookups
        center(Point3f(loX + (hiX - loX) / 2, loY + (hiY - loY) / 2, z)),
        area(((hiX - loX) * (hiY - loY))) {

}


// Portal area sapling
void Portal::SamplePortal(const Interaction &ref,
                          const Point2f &u,
                          Vector3f *wi, Float *pdf) const {

    // sample portal uniformly
    float randY = loY + u.y * (hiY - loY);
    float randX = loX + u.x * (hiX - loX);
    Point3f sampledPoint = Point3f(randX, randY, this->z);

    *wi = Normalize(sampledPoint - ref.p);
    *pdf = DistanceSquared(ref.p, sampledPoint) / (AbsDot(this->n, -*wi) * area);
}

Float Portal::Pdf_Portal(const Interaction &ref, const Vector3f &wi) const {

    // axis aligned across z axis, plane intersection
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



