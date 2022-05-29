#include "aaportal_light.h"
#include "stats.h"
#include "paramset.h"
#include "portal.h"
#include "reflection.h"
#include "diffuse.h"
#include "scene.h"

namespace pbrt {

AAPortalLight::AAPortalLight(const Transform &LightToWorld,
                             const MediumInterface &mediumInterface, const Spectrum &Le,
                             int nSamples,
                             const std::shared_ptr<AAPlane> &light,
                             const Portal &portal,
                             const PortalStrategy strategy,
                             bool useFrustum,
                             bool twoSided)
        : DiffuseAreaLight(LightToWorld, mediumInterface, Le, nSamples, light, twoSided),
          portal(portal),
          shape(light),
          strat(strategy),

          // frustum normals and points
          useFrustum(useFrustum),
          fn0(Normal3f(0, 0, 0)),
          fn1(Normal3f(0, 0, 0)),
          fn2(Normal3f(0, 0, 0)),
          fn3(Normal3f(0, 0, 0)),
          fp0(Point3f(0, 0, 0)),
          fp1(Point3f(0, 0, 0)),
          fp2(Point3f(0, 0, 0)),
          fp3(Point3f(0, 0, 0))
{
}

bool AAPortalLight::InFrustrum(const Point3f p) const {
    bool res = true;
    res &= Dot(fp0 - p, fn0) >= 0;
    res &= Dot(fp1 - p, fn1) >= 0;
    res &= Dot(fp2 - p, fn2) >= 0;
    res &= Dot(fp3 - p, fn3) >= 0;

    return res;
}

Spectrum AAPortalLight::EstimateDirect(const Interaction &it,
                        const Point2f &u1, const Point2f &u2,
                        const Scene &scene, bool specular) const {

    // if above portal, do standard strategy
    if (it.p.z > portal.z) {
        return EstimateDirectLight(it, u1, u2, scene, specular);
    }

    // if not in frustum, return black
    if (useFrustum && !InFrustrum(it.p)) {
        return 0;
    }

    if (strat == PortalStrategy::SampleUniformPortal) {
        return EstimateDirectPortal(it, u1, u2, scene, specular);
    }

    else if (strat == PortalStrategy::SampleUniformLight) {
        return EstimateDirectLight(it, u1, u2, scene, specular);
    }

    else if (strat == PortalStrategy::SampleProjection) {
        return EstimateDirectProj(it, u1, u2, scene, specular);
    }

}

Spectrum AAPortalLight::EstimateDirectLight(const Interaction &it,
                                            const Point2f &u1, const Point2f &u2,
                                            const Scene &scene, bool specular) const {

    // cast reference point to surface interaction
    const auto &ref = (const SurfaceInteraction &)it;

    // reused variables
    BxDFType bsdfFlags = specular ? BSDF_ALL : BxDFType(BSDF_ALL & ~BSDF_SPECULAR);
    Vector3f wi;
    Spectrum Li;
    Spectrum f;
    Float pdf = 0;
    VisibilityTester vis;

    Spectrum Ld(0.f);

    // SAMPLE LIGHT
    Li = Sample_Li(it, u1, &wi, &pdf, &vis);

    if (!Li.IsBlack() && pdf > 0) {

        SurfaceInteraction lightIsect;
        Ray ray = it.SpawnRay(wi);
        if (scene.Intersect(ray, &lightIsect)) {
            Li = lightIsect.Le(-wi);
        }

        // compute BSDF for sampled direction
        f = ref.bsdf->f(ref.wo, wi, bsdfFlags) * AbsDot(wi, ref.shading.n);

        if (!f.IsBlack() && !Li.IsBlack()) {
            // weight = PowerHeuristic3(1, pdf, 1, scatteringPdf, 1, lightPdf);
            Ld += f * Li / pdf;
        }
    }

    return Ld;
}


Spectrum AAPortalLight::EstimateDirectPortal(const Interaction &it,
                                             const Point2f &u1, const Point2f &u2,
                                             const Scene &scene, bool specular) const {


    // cast reference point to surface interaction
    const auto &ref = (const SurfaceInteraction &)it;

    // reused variables
    BxDFType bsdfFlags = specular ? BSDF_ALL : BxDFType(BSDF_ALL & ~BSDF_SPECULAR);
    Vector3f wi;
    Spectrum Li;
    Spectrum f;
    Float portalPdf = 0;

    Spectrum Ld(0.f);

    if (ref.p.z > portal.z) {
        return EstimateDirectLight(it, u1, u2, scene, specular);
    }

    // SAMPLE PORTAL
    portal.SamplePortal(it, u1, &wi, &portalPdf);

    if (portalPdf > 0) {

        // get direct illumination from sampled direction
        Li = 0;
        SurfaceInteraction lightIsect;
        Ray ray = it.SpawnRay(wi);
        if (scene.Intersect(ray, &lightIsect)) {
            Li = lightIsect.Le(-wi);
        }

        // compute BSDF for sampled direction
        f = ref.bsdf->f(ref.wo, wi, bsdfFlags) * AbsDot(wi, ref.shading.n);

        if (!f.IsBlack() && !Li.IsBlack()) {
            // weight = PowerHeuristic3(1, portalPdf, 1, scatteringPdf, 1, lightPdf);
            Ld += f * Li / portalPdf;
        }
    }

    return Ld;
}

Spectrum AAPortalLight::EstimateDirectProj(const Interaction &it,
                                           const Point2f &u1, const Point2f &u2,
                                           const Scene &scene, bool specular) const {

    // cast reference point to surface interaction
    const auto &ref = (const SurfaceInteraction &)it;

    // reused variables
    BxDFType bsdfFlags = specular ? BSDF_ALL : BxDFType(BSDF_ALL & ~BSDF_SPECULAR);
    Vector3f wi;
    Spectrum Li;
    Spectrum f;
    Float projPdf = 0;
    Spectrum Ld(0.f);

    // SAMPLE PORTAL
    shape->SampleProjection(ref.p, portal, u1, &wi, &projPdf);

    if (projPdf > 0) {

        // get direct illumination from sampled direction
        Li = 0;
        SurfaceInteraction lightIsect;
        Ray ray = it.SpawnRay(wi);
        if (scene.Intersect(ray, &lightIsect)) {
            Li = lightIsect.Le(-wi);
        }

        // compute BSDF for sampled direction
        f = ref.bsdf->f(ref.wo, wi, bsdfFlags) * AbsDot(wi, ref.shading.n);

        if (!f.IsBlack() && !Li.IsBlack()) {
            // weight = PowerHeuristic3(1, projPdf, 1, scatteringPdf, 1, lightPdf);
            Ld += f * Li / projPdf;
        }
    }

    return Ld;
}

void AAPortalLight::Preprocess(const Scene &scene) {

    // light and portal points
    Point3f l0 = Point3f(shape->loX, shape->loY, shape->z);
    Point3f l1 = Point3f(shape->loX, shape->hiY, shape->z);
    Point3f l2 = Point3f(shape->hiX, shape->loY, shape->z);
    Point3f l3 = Point3f(shape->hiX, shape->hiY, shape->z);

    Point3f p0 = Point3f(portal.loX, portal.loY, portal.z);
    Point3f p1 = Point3f(portal.loX, portal.hiY, portal.z);
    Point3f p2 = Point3f(portal.hiX, portal.loY, portal.z);
    Point3f p3 = Point3f(portal.hiX, portal.hiY, portal.z);

    // directions
    auto fd3 = Normalize(p3 - l0);
    auto fd2 = Normalize(p2 - l1);
    auto fd1 = Normalize(p1 - l2);
    auto fd0 = Normalize(p0 - l3);

    // frustum directions
    LOG(INFO) << "DBG DIR-WHITE:" << p0 << ";" << fd0;
    LOG(INFO) << "DBG DIR-BLUE:" << p1 << ";" << fd1;
    LOG(INFO) << "DBG DIR-GREEN:" << p2 << ";" << fd2;
    LOG(INFO) << "DBG DIR-RED:" << p3 << ";" << fd3;

    // frustum plane normals
    fn0 = Normal3f(Cross(fd1, fd0));
    fn1 = Normal3f(Cross(fd3, fd1));
    fn2 = Normal3f(Cross(fd2, fd3));
    fn3 = Normal3f(Cross(fd0, fd2));

    // frustum plane points
    fp0 = (p0 + p1) / 2;
    fp1 = (p1 + p3) / 2;
    fp2 = (p2 + p3) / 2;
    fp3 = (p0 + p2) / 2;
    LOG(INFO) << "DBG DIR-CYAN:" << fp0 << ";" << fn0;
    LOG(INFO) << "DBG DIR-CYAN:" << fp1 << ";" << fn1;
    LOG(INFO) << "DBG DIR-CYAN:" << fp2 << ";" << fn2;
    LOG(INFO) << "DBG DIR-CYAN:" << fp3 << ";" << fn3;

    Light::Preprocess(scene);
}


std::shared_ptr<AAPortalLight> CreateAAPortal(
        const Transform &light2world,
        const Medium *medium,
        const ParamSet &paramSet, const std::shared_ptr<AAPlane> &shape) {

    Spectrum L = paramSet.FindOneSpectrum("L", Spectrum(1.0));
    Spectrum sc = paramSet.FindOneSpectrum("scale", Spectrum(1.0));
    int nSamples = paramSet.FindOneInt("samples", paramSet.FindOneInt("nsamples", 1));
    bool twoSided = paramSet.FindOneBool("twosided", false);

    // find geometry of portal
    Float loX = paramSet.FindOneFloat("loX", 0);
    Float loY = paramSet.FindOneFloat("loY", 0);
    Float hiX = paramSet.FindOneFloat("hiX", 0);
    Float hiY = paramSet.FindOneFloat("hiY", 0);
    Float portalZ = paramSet.FindOneFloat("portalZ", 0);
    Normal3f n = paramSet.FindOneNormal3f("portalNormal", Normal3f(0, 0, -1));
    auto portal = new Portal(loY, hiY, loX, hiX, portalZ, n);

    PortalStrategy strategy;
    auto st = paramSet.FindOneString("strategy", "light");
    if (st == "light") {
        strategy = PortalStrategy::SampleUniformLight;
    } else if (st == "portal") {
        strategy = PortalStrategy::SampleUniformPortal;
    } else if (st == "projection") {
        strategy = PortalStrategy::SampleProjection;
    } else {
        Warning("Portal strategy unknown");
    }

    bool useFrustum = paramSet.FindOneBool("useFrustum", true);


    if (PbrtOptions.quickRender) nSamples = std::max(1, nSamples / 4);


    return std::make_shared<AAPortalLight>(light2world, medium, L * sc,
                                         nSamples, shape, *portal, strategy, useFrustum, twoSided);
}


}