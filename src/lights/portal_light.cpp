#include "portal_light.h"
#include "stats.h"
#include "paramset.h"
#include "portal.h"
#include "reflection.h"
#include "diffuse.h"
#include "scene.h"
#include "ext/sexpresso.hpp"

namespace pbrt {

PortalLight::PortalLight(const Transform &LightToWorld,
                         const MediumInterface &mediumInterface, const Spectrum &Le,
                         int nSamples,
                         const std::shared_ptr<AAPlane> &light,
                         const Portal &portal,
                         const PortalStrategy strategy,
                         bool twoSided)
        : DiffuseAreaLight(LightToWorld, mediumInterface, Le, nSamples, light, twoSided),
          portal(portal),
          shape(light),
          strat(strategy) {
}

Spectrum PortalLight::EstimateDirect(const Interaction &it,
                                     const Point2f &u1, const Point2f &u2,
                                     const Scene &scene, bool specular) const {

    if (!portal.InFront(it.p)) {
        return EstimateDirectLight(it, u1, u2, scene, specular);
    }

    // if not in frustum, return black
    if (!portal.InFrustrum(it.p)) {
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

    return 0;

}

Spectrum PortalLight::EstimateDirectLight(const Interaction &it,
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


Spectrum PortalLight::EstimateDirectPortal(const Interaction &it,
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

Spectrum PortalLight::EstimateDirectProj(const Interaction &it,
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
    portal.SampleProj(ref, u1, &wi, &projPdf);

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

void PortalLight::Preprocess(const Scene &scene) {
    Light::Preprocess(scene);
}


std::shared_ptr<PortalLight> CreateAAPortal(
        const Transform &light2world,
        const Medium *medium,
        const ParamSet &paramSet, const std::shared_ptr<AAPlane> &shape) {

    Spectrum L = paramSet.FindOneSpectrum("L", Spectrum(1.0));
    Spectrum sc = paramSet.FindOneSpectrum("scale", Spectrum(1.0));
    int nSamples = paramSet.FindOneInt("samples", paramSet.FindOneInt("nsamples", 1));
    bool twoSided = paramSet.FindOneBool("twosided", false);


    // replace with parsing the data

    std::string portalData = paramSet.FindOneString("portalData", "");
    auto parseTree = sexpresso::parse(portalData);

    Portal* portal = nullptr;
    auto type = parseTree.getChild(0).getChild(0).toString();
    if (type == "AA") {
        Float loX = std::stof(parseTree.getChild(0).getChild(1).toString());
        Float loY = std::stof(parseTree.getChild(0).getChild(2).toString());
        Float hiX = std::stof(parseTree.getChild(0).getChild(3).toString());
        Float hiY = std::stof(parseTree.getChild(0).getChild(4).toString());
        Float z = std::stof(parseTree.getChild(0).getChild(5).toString());
        std::string orientation = parseTree.getChild(0).getChild(6).toString();
        bool greater = orientation == "+";
        portal = new Portal(loY, hiY, loX, hiX, z, greater, *shape);
    }

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



    if (PbrtOptions.quickRender) nSamples = std::max(1, nSamples / 4);


    return std::make_shared<PortalLight>(light2world, medium, L * sc,
                                         nSamples, shape, *portal, strategy, twoSided);
}


}