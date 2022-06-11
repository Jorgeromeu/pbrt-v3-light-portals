#include "portal_arealight.h"

#include <utility>
#include "stats.h"
#include "paramset.h"
#include "portals/aaportal.h"
#include "reflection.h"
#include "diffuse.h"
#include "scene.h"
#include "ext/sexpresso.hpp"

namespace pbrt {

PortalArealight::PortalArealight(const Transform &LightToWorld,
                                 const MediumInterface &mediumInterface, const Spectrum &Le,
                                 int nSamples,
                                 const std::shared_ptr<AAPlaneShape> &light,
                                 std::vector<AAPortal> portals,
                                 const PortalStrategy strategy,
                                 bool twoSided)
        : DiffuseAreaLight(LightToWorld, mediumInterface, Le, nSamples, light, twoSided),
          portals(std::move(portals)),
          selectedPortal(0),
          shape(light),
          strat(strategy) {
}

Spectrum PortalArealight::EstimateDirect(const Interaction &it,
                                         const Point2f &u1, const Point2f &u2,
                                         const Scene &scene, bool specular) {

    // randomly choose the portal
    selectedPortal = std::floor(u1.x * portals.size() + 1) - 1;
    if (selectedPortal >= portals.size()) selectedPortal = portals.size() - 1;

//    for (int i=0; i<portals.size(); i++) {
//        if (portals[i].InFront(it.p))  {
//            selectedPortal = i;
//        }
//    }


    if (!portals[selectedPortal].InFront(it.p)) {
         // Float rgb[3] = {1, 0, 0};
         // return RGBSpectrum::FromRGB(rgb);
         return EstimateDirectLight(it, u1, u2, scene, specular);
    }

    // if not in frustum, return black
    if (!portals[selectedPortal].InFrustum(it.p)) {
         // Float rgb[3] = {0, 1, 0};
         // return RGBSpectrum::FromRGB(rgb);
         return 0;
    }

     // Float rgb[3] = {0, 0, selectedPortal / 10.0f};
     // return RGBSpectrum::FromRGB(rgb);

    if (strat == PortalStrategy::SampleUniformPortal) {
        return EstimateDirectPortal(it, u1, u2, scene, specular);
    }

    else if (strat == PortalStrategy::SampleUniformLight) {
        return EstimateDirectLight(it, u1, u2, scene, specular);
    }

    else if (strat == PortalStrategy::SampleProjection) {
        return EstimateDirectProj(it, u1, u2, scene, specular) / (1.0f / (float) portals.size());
    }

    return 0;

}

Spectrum PortalArealight::EstimateDirectLight(const Interaction &it,
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


Spectrum PortalArealight::EstimateDirectPortal(const Interaction &it,
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

    // SAMPLE PORTAL
    portals[selectedPortal].SamplePortal(it, u1, &wi, &portalPdf);

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

Spectrum PortalArealight::EstimateDirectProj(const Interaction &it,
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
    portals[selectedPortal].SampleProj(ref, u1, &wi, &projPdf);

    LOG(INFO) << "DBG DIR:" << ref.p << ";" << wi;

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

void PortalArealight::Preprocess(const Scene &scene) {
    Light::Preprocess(scene);
}

std::shared_ptr<PortalArealight> CreateAAPortal(
        const Transform &light2world,
        const Medium *medium,
        const ParamSet &paramSet, const std::shared_ptr<AAPlaneShape> &shape) {

    Spectrum L = paramSet.FindOneSpectrum("L", Spectrum(1.0));
    Spectrum sc = paramSet.FindOneSpectrum("scale", Spectrum(1.0));
    int nSamples = paramSet.FindOneInt("samples", paramSet.FindOneInt("nsamples", 1));
    bool twoSided = paramSet.FindOneBool("twosided", false);

    // parse portalData
    std::string portalData = paramSet.FindOneString("portalData", "");
    auto parseTree = sexpresso::parse(portalData).getChild(0);
    std::vector<AAPortal> portals = {};

    for (int i=0; i<parseTree.childCount(); i++) {

        auto portalSexpr = parseTree.getChild(i);
        auto type = portalSexpr.getChild(0).toString();

        if (type == "AA") {
            Float loX = std::stof(portalSexpr.getChild(1).toString());
            Float loY = std::stof(portalSexpr.getChild(2).toString());
            Float loZ = std::stof(portalSexpr.getChild(3).toString());

            Float hiX = std::stof(portalSexpr.getChild(4).toString());
            Float hiY = std::stof(portalSexpr.getChild(5).toString());
            Float hiZ = std::stof(portalSexpr.getChild(6).toString());

            int axis = std::stoi(portalSexpr.getChild(7).toString());
            bool facingFw = portalSexpr.getChild(8).toString() == "+";

            portals.emplace_back(Point3f(loX, loY, loZ), Point3f(hiX, hiY, hiZ), axis, facingFw, *shape);
        }
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
        Warning("AAPortal strategy unknown");
    }


    if (PbrtOptions.quickRender) nSamples = std::max(1, nSamples / 4);


    return std::make_shared<PortalArealight>(light2world, medium, L * sc,
                                             nSamples, shape, portals, strategy, twoSided);
}


}