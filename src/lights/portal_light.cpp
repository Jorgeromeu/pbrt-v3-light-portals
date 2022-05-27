#include "lights/portal_light.h"
#include "paramset.h"
#include "sampling.h"
#include "shapes/triangle.h"
#include "stats.h"
#include "diffuse.h"
#include "scene.h"

namespace pbrt {

// PortalLight Method Definitions
PortalLight::PortalLight(const Transform &LightToWorld,
                         const MediumInterface &mediumInterface,
                         const Spectrum &Lemit, int nSamples,
                         const std::shared_ptr<Triangle> &shape,
                         const Float &loY,
                         const Float &hiY,
                         const Float &loX,
                         const Float &hiX,
                         const Float &portalZ,
                         const Normal3f &portalNormal,
                         bool twoSided)
: DiffuseAreaLight(LightToWorld, mediumInterface, Lemit, nSamples, shape, twoSided),
      Lemit(Lemit),
      shape(shape),
      twoSided(twoSided),
      minCos(0),
      area(shape->Area()) {

    // initialize portal
    portal = new Portal(loY, hiY, loX, hiX, portalZ, portalNormal);
    scene = nullptr;
}

Spectrum PortalLight::Power() const {
    return (twoSided ? 2 : 1) * Lemit * area * Pi;
}

void PortalLight::Preprocess(const Scene &scene) {
    minCos = shape->MinSampleCosine(this->portal);
    this->scene = &scene;
}

// Li Sampling
Spectrum PortalLight::Sample_Li(const Interaction &ref, const Point2f &u,
                                Vector3f *wi, Float *pdf,
                                VisibilityTester *vis) const {

    ProfilePhase _(Prof::LightSample);
    Float lightPdf;

    // sample point on light
    Interaction pLight = shape->Sample(ref, u, &lightPdf);
    if (lightPdf == 0 || (pLight.p - ref.p).LengthSquared() == 0) {
        lightPdf = 0;
        return 0.f;
    }
    *wi = Normalize(pLight.p - ref.p);
    *vis = VisibilityTester(ref, pLight);
    *pdf = lightPdf;

    return L(pLight, -*wi);
}

Float PortalLight::Pdf_Li(const Interaction &ref,
                          const Vector3f &wi) const {
    ProfilePhase _(Prof::LightPdf);
    return shape->Pdf(ref, wi);
}

// FOR BIDIRECTIONAL METHODS
Spectrum PortalLight::Sample_Le(const Point2f &u1, const Point2f &u2,
                                Float time, Ray *ray, Normal3f *nLight,
                                Float *pdfPos, Float *pdfDir) const {
    ProfilePhase _(Prof::LightSample);
    // Sample a point on the area light's _Shape_, _pShape_
    Interaction pShape = shape->Sample(u1, pdfPos);
    pShape.mediumInterface = mediumInterface;
    *nLight = pShape.n;

    // Sample a cosine-weighted outgoing direction _w_ for area light
    Vector3f w;
    if (twoSided) {
        Point2f u = u2;
        // Choose a side to sample and then remap u[0] to [0,1] before
        // applying cosine-weighted hemisphere sampling for the chosen side.
        if (u[0] < .5) {
            u[0] = std::min(u[0] * 2, OneMinusEpsilon);
            w = CosineSampleHemisphere(u);
        } else {
            u[0] = std::min((u[0] - .5f) * 2, OneMinusEpsilon);
            w = CosineSampleHemisphere(u);
            w.z *= -1;
        }
        *pdfDir = 0.5f * CosineHemispherePdf(std::abs(w.z));
    } else {
        w = CosineSampleHemisphere(u2);
        *pdfDir = CosineHemispherePdf(w.z);
    }

    Vector3f v1, v2, n(pShape.n);
    CoordinateSystem(n, &v1, &v2);
    w = w.x * v1 + w.y * v2 + w.z * n;
    *ray = pShape.SpawnRay(w);
    return L(pShape, w);
}

void PortalLight::Pdf_Le(const Ray &ray, const Normal3f &n, Float *pdfPos,
                         Float *pdfDir) const {
    ProfilePhase _(Prof::LightPdf);
    Interaction it(ray.o, n, Vector3f(), Vector3f(n), ray.wvls, ray.time,
                   mediumInterface);
    *pdfPos = shape->Pdf(it);
    *pdfDir = twoSided ? (.5 * CosineHemispherePdf(AbsDot(n, ray.d)))
                       : CosineHemispherePdf(Dot(n, ray.d));
}

void PortalLight::SampleProj(const Point3f &ref,
                             const Point2f& u,
                             Point3f *sampled,
                             Float *pdf,
                             Vector3f *wi) const {
    shape->SampleProjectionFastClip(ref, *portal, u, sampled, pdf, wi);
}


std::shared_ptr<PortalLight> CreatePortalLight(
        const Transform &light2world,
        const Medium *medium,
        const ParamSet &paramSet, const std::shared_ptr<Triangle> &shape) {

    Spectrum L = paramSet.FindOneSpectrum("L", Spectrum(1.0));
    Spectrum sc = paramSet.FindOneSpectrum("scale", Spectrum(1.0));
    int nSamples = paramSet.FindOneInt("samples",paramSet.FindOneInt("nsamples", 1));
    bool twoSided = paramSet.FindOneBool("twosided", false);

    // find geometry of portal
    Float loY = paramSet.FindOneFloat("loY", 0);
    Float hiY = paramSet.FindOneFloat("hiY", 0);
    Float loX = paramSet.FindOneFloat("loZ", 0);
    Float hiX = paramSet.FindOneFloat("hiZ", 0);
    Float portalZ = paramSet.FindOneFloat("portalX", 0);
    Normal3f n = paramSet.FindOneNormal3f("portalNormal", Normal3f (0, 0, -1));

    if (PbrtOptions.quickRender) nSamples = std::max(1, nSamples / 4);

    return std::make_shared<PortalLight>(light2world, medium, L * sc,
                                         nSamples, shape, loY, hiY, loX, hiX, portalZ, n, twoSided);
}

}  // namespace pbrt
