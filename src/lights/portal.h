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

namespace pbrt {

// DiffuseAreaLight Declarations
class PortalLight : public DiffuseAreaLight {
  public:

    // DiffuseAreaLight Public Methods
    PortalLight(const Transform &LightToWorld,
                const MediumInterface &mediumInterface, const Spectrum &Le,
                int nSamples,
                const std::shared_ptr<Shape> &shape,
                const Float &loY,
                const Float &hiY,
                const Float &loX,
                const Float &hiX,
                const Float &portalZ,
                const Vector3f &portalNormal,
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

    Point3f SamplePortal(const Interaction &it,
                         const Point2f &uPortal,
                         Vector3f *wi,
                         Float *pdf) const;

    Float Pdf_Portal(const Interaction &it,
                       const Vector3f &wi) const;

    Point3f SampleIntersection(const Interaction &ref, const Point2f &uPortal, Vector3f *wi, Float *pdf) const;

    // portal data
    const Float loX;
    const Float hiX;
    const Float loY;
    const Float hiY;
    const Float portalZ;
    const Vector3f portalNormal;
    const Point3f portalCenter;

  protected:
    // DiffuseAreaLight Protected Data
    const Spectrum Lemit;
    std::shared_ptr<Shape> shape;
    // Added after book publication: by default, DiffuseAreaLights still
    // only emit in the hemimsphere around the surface normal.  However,
    // this behavior can now be overridden to give emission on both sides.
    const bool twoSided;
    const Float area;

    // required data
    const Float portalArea;
    const Scene* scene;

    void Preprocess(const Scene &scene) override;

};

std::shared_ptr<PortalLight> CreatePortalLight(
    const Transform &light2world, const Medium *medium,
    const ParamSet &paramSet, const std::shared_ptr<Shape> &shape);

}  // namespace pbrt

#endif  // PBRT_LIGHTS_PORTAL
