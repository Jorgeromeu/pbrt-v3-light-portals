#include "lights/portal.h"
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
                         const std::shared_ptr<Shape> &shape,
                         const Float &loY,
                         const Float &hiY,
                         const Float &loX,
                         const Float &hiX,
                         const Float &portalZ,
                         const Vector3f &portalNormal,
                         bool twoSided)
: DiffuseAreaLight(LightToWorld, mediumInterface, Lemit, nSamples, shape, twoSided),
      Lemit(Lemit),
      shape(shape),
      twoSided(twoSided),
      area(shape->Area()),
      loX(loX),
      hiX(hiX),
      loY(loY),
      hiY(hiY),
      portalZ(portalZ),
      portalNormal(portalNormal),
      // midpoint of portal
      portalCenter(Point3f(loX + (hiX - loX) / 2, loY + (hiY - loY) / 2, portalZ)),
      portalArea(((hiX - loX) * (hiY - loY)) / 2) {
    scene = nullptr;
}



Spectrum PortalLight::Power() const {
    return (twoSided ? 2 : 1) * Lemit * area * Pi;
}

void PortalLight::Preprocess(const Scene &scene) {
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

// Intersection Sampling
Point3f PortalLight::SampleIntersection(const Interaction &ref,
                                        const Point2f &uPortal,
                                        Vector3f *wi,
                                        Float *pdf) const {

    // HARDCODED PRODUCT
    float loXProd = -2.26887;
    float hiXProd = -1.73113;

    float prodArea = ((hiXProd - loXProd) * (hiY - loY)) / 2;

    // sample intersection uniformly
    float randY = loY + uPortal.x * (hiY - loY);
    float randX = loXProd + uPortal.y * (hiXProd - loXProd);
    Point3f sampledPoint = Point3f(randX, randY, portalZ);

    *wi = Normalize(sampledPoint - ref.p);
    *pdf = DistanceSquared(ref.p, sampledPoint) / (AbsDot(portalNormal, -*wi) * prodArea);

    return sampledPoint;
}


// Visibility Sampling
Point3f PortalLight::SamplePortal(const Interaction &ref,
                                  const Point2f &uPortal,
                                  Vector3f *wi,
                                  Float *pdf) const {

    // sample portal uniformly
    float randY = loY + uPortal.y * (hiY - loY);
    float randX = loX + uPortal.x * (hiX - loX);
    Point3f sampledPoint = Point3f(randX, randY, portalZ);

    *wi = Normalize(sampledPoint - ref.p);
    *pdf = DistanceSquared(ref.p, sampledPoint) / (AbsDot(portalNormal, -*wi) * portalArea);

    return sampledPoint;
}

Float PortalLight::Pdf_Portal(const Interaction &ref,
                              const Vector3f &wi) const {

    // axis aligned across z axis, plane intersection
    Ray r = ref.SpawnRay(wi);

    if (r.d.z == 0) return 0;

    float t = (portalZ - r.o.z) / r.d.z;
    Point3f isect = r.o + r.d * t;

    // if hit plane
    if (isect.y >= loY && isect.y <= hiY && isect.x >= loX && isect.x <= hiX) {
        return DistanceSquared(ref.p, isect) / (AbsDot(portalNormal, -Normalize(wi)) * portalArea);
    }

    return 0;
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



std::shared_ptr<PortalLight> CreatePortalLight(
        const Transform &light2world,
        const Medium *medium,
        const ParamSet &paramSet, const std::shared_ptr<Shape> &shape) {

    Spectrum L = paramSet.FindOneSpectrum("L", Spectrum(1.0));
    Spectrum sc = paramSet.FindOneSpectrum("scale", Spectrum(1.0));
    int nSamples = paramSet.FindOneInt("samples",paramSet.FindOneInt("nsamples", 1));
    bool twoSided = paramSet.FindOneBool("twosided", false);

    // find geometry of portal
    Float loY = paramSet.FindOneFloat("loY", 0);
    Float hiY = paramSet.FindOneFloat("hiY", 0);
    Float loZ = paramSet.FindOneFloat("loZ", 0);
    Float hiZ = paramSet.FindOneFloat("hiZ", 0);
    Float portalX = paramSet.FindOneFloat("portalX", 0);
    Vector3f portalNormal = paramSet.FindOneVector3f("portalNormal", Vector3f(1, 0, 0));

    if (PbrtOptions.quickRender) nSamples = std::max(1, nSamples / 4);

    return std::make_shared<PortalLight>(light2world, medium, L * sc,
                                         nSamples, shape, loY, hiY, loZ, hiZ, portalX, portalNormal, twoSided);
}

}  // namespace pbrt
