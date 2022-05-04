#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_LIGHTS_PORTAL
#define PBRT_LIGHTS_PORTAL

// lights/portal.h*
#include "random"
#include "pbrt.h"
#include "light.h"
#include "primitive.h"

namespace pbrt {

// DiffuseAreaLight Declarations
class PortalLight : public AreaLight {
  public:

    // DiffuseAreaLight Public Methods
    PortalLight(const Transform &LightToWorld,
                const MediumInterface &mediumInterface, const Spectrum &Le,
                int nSamples,
                const std::shared_ptr<Shape> &shape,
                const Point3f &x0,
                const Point3f &x1,
                const Point3f &x2,
                const Point3f &x3,
                bool twoSided = false);

    Spectrum L(const Interaction &intr, const Vector3f &w) const {
        return (twoSided || Dot(intr.n, w) > 0) ? Lemit : Spectrum(0.f);
    }
    Spectrum Power() const;
    Spectrum Sample_Li(const Interaction &ref, const Point2f &u, Vector3f *wo,
                       Float *pdf, VisibilityTester *vis) const;
    Float Pdf_Li(const Interaction &, const Vector3f &) const;
    Spectrum Sample_Le(const Point2f &u1, const Point2f &u2, Float time,
                       Ray *ray, Normal3f *nLight, Float *pdfPos,
                       Float *pdfDir) const;
    void Pdf_Le(const Ray &, const Normal3f &, Float *pdfPos,
                Float *pdfDir) const;

  protected:
    // DiffuseAreaLight Protected Data
    const Spectrum Lemit;
    std::shared_ptr<Shape> shape;
    const Point3f x0;
    const Point3f x1;
    const Point3f x2;
    const Point3f x3;
    // Added after book publication: by default, DiffuseAreaLights still
    // only emit in the hemimsphere around the surface normal.  However,
    // this behavior can now be overridden to give emission on both sides.
    const bool twoSided;
    const Float area;

    std::mt19937 gen;

    std::uniform_real_distribution<float> distr_y;
    std::uniform_real_distribution<float> distr_z;
};

std::shared_ptr<AreaLight> CreatePortalLight(
    const Transform &light2world, const Medium *medium,
    const ParamSet &paramSet, const std::shared_ptr<Shape> &shape);

}  // namespace pbrt

#endif  // PBRT_LIGHTS_PORTAL
