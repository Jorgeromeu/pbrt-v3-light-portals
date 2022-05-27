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
#include "diffuse.h"
#include "primitive.h"
#include "portal.h"
#include "shapes/triangle.h"

namespace pbrt {

// DiffuseAreaLight Declarations
class PortalLight : public DiffuseAreaLight {
  public:

    // reference to portal
    const Portal* portal;
    Float minCos;

    // DiffuseAreaLight Public Methods
    PortalLight(const Transform &LightToWorld,
                const MediumInterface &mediumInterface, const Spectrum &Le,
                int nSamples,
                const std::shared_ptr<Triangle> &shape,
                const Float &loY,
                const Float &hiY,
                const Float &loX,
                const Float &hiX,
                const Float &portalZ,
                const Normal3f &portalNormal,
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


    void SampleProj(const Point3f &ref,
                    const Point2f& u,
                    Point3f *sampled,
                    Float *pdf, Vector3f *wi) const;

  protected:


    // DiffuseAreaLight Protected Data
    const Spectrum Lemit;
    std::shared_ptr<Triangle> shape;
    // Added after book publication: by default, DiffuseAreaLights still
    // only emit in the hemimsphere around the surface normal.  However,
    // this behavior can now be overridden to give emission on both sides.
    const bool twoSided;
    const Float area;

    const Scene* scene;

    void Preprocess(const Scene &scene) override;

};

std::shared_ptr<PortalLight> CreatePortalLight(
    const Transform &light2world, const Medium *medium,
    const ParamSet &paramSet, const std::shared_ptr<Triangle> &shape);

}  // namespace pbrt

#endif  // PBRT_LIGHTS_PORTAL
