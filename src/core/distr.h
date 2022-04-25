

#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_CORE_DISTR_H
#define PBRT_CORE_DISTR_H

#include "pbrt.h"
#include "spectrum.h"

namespace pbrt {
/**
 * Discrete probability distribution.
 * 
 * This class can be used to transform uniformly distributed samples
 * to a discrete probabilty distribution, through what is called 
 * inversion transform sampling. Internally, it retains a cumulative
 * distribution function of the probability distribution
 */
template <int nEntries>
struct DiscreteDistribution {
  explicit DiscreteDistribution() { std::fill(c, c + nEntries + 1, 0); }

  // Feed a array of (unnormalized) values of length _nEntries_
  // into the distribution, which ensures normalization is applied 
  // and computes the PDF/CDF 
  inline void Set(const Float *f) {
    c[0] = 0.f;
    for (int i = 0; i < nEntries; ++i) {
      c[i + 1] = c[i] + f[i];
    }
    Float invsum = 1.f / c[nEntries];
    for (int i = 1; i < nEntries; ++i) {
      c[i] *= invsum;
    }
    c[nEntries] = 1.0f;
  }

  // Query the probability density function at a given entry
  inline Float Pdf(int i) const {
    return c[i + 1] - c[i];
  }

  // Query the cumulative density function at a given entry
  inline Float Cdf(int i) const {
    return c[i];
  }

  // Transform a uniformly distributed sample to the 
  // stored distribution
  inline int Sample(Float sample1D) const {
    auto entry = std::lower_bound(c, c + nEntries + 1, sample1D);
    return std::min(nEntries - 1, (int) std::max((ptrdiff_t) 0, entry - c - 1));
  }

  // Transform a uniformly distributed sample to the 
  // stored distribution, and obtain the resulting pdf
  inline int Sample(Float sample1D, Float &pdf) const {
    int i = Sample(sample1D);
    pdf = Pdf(i);
    return i;
  }

  std::string ToString() const {
    std::string str = "[ ";
    for (int i = 0; i < nEntries + 1; ++i) {
      str += StringPrintf("%f", c[i]);
      if (i + 1 < nEntries + 1) str += ", ";
    }
    str += " ]";
    return str;
  }

  friend std::ostream &operator<<(std::ostream &os, const DiscreteDistribution &s) {
    return os << s.ToString();
  }

private:
  Float c[nEntries + 1];
};

struct SpectralDistribution : public DiscreteDistribution<nSpectralSamples> {
  explicit SpectralDistribution() : DiscreteDistribution() {}
  SpectralDistribution(Spectrum s) : SpectralDistribution() {
    Set(&(s[0]));
  }

  inline Float sampleWavelength(Float sample1D, Float &pdf) {
    const int i = Sample(sample1D, pdf);
    const Float minv = Cdf(i), 
                maxv = Cdf(i + 1),
                diff = maxv - minv;
    const Float alpha = (sample1D - minv) / diff;

    return sampledLambdaStart 
         + sampledLambdaRange * ((alpha + (Float) i) / (Float) nSpectralSamples);
  }

  inline Float sampleWavelength(Float sample1D) {
    const int i = Sample(sample1D);
    const Float minv = Cdf(i), 
                maxv = Cdf(i + 1),
                diff = maxv - minv;
    const Float alpha = (sample1D - minv) / diff;

    return sampledLambdaStart 
         + sampledLambdaRange * ((alpha + (Float) i) / (Float) nSpectralSamples);
  }
};

} // namespace pbrt

#endif // PBRT_CORE_DISTR_H