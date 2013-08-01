/**
 * @file Tools/Optimization/CMAES.h
 * Implementation of the Covariance Matrix Adaption Evolution Strategie (CMA-ES) optimization algorithm.
 * @author <a href="afabisch@tzi.de">Alexander Fabisch</a>
 */

#pragma once

#include "Tools/Debugging/Asserts.h"
#include "Tools/Math/MultivariateGaussian.h"
#include "Tools/Math/YaMatrix.h"
#include "Tools/Math/EigenDecomposition.h"
#include <algorithm>
#include <cmath>
#include <limits>

// TODO write/read file

/**
 * @class CMAES
 * @see http://en.wikipedia.org/wiki/CMA-ES
 */
template <typename T, typename R, unsigned N>
class CMAES
{
  class Individual : public YaMatrix<T>
  {
  public:
    R fitness;
    bool fitnessCalculated;
    Individual(const YaMatrix<T>& values) : YaMatrix<T>(values), fitness(), fitnessCalculated(false) {}
    bool operator<(const Individual& other) const { return fitness < other.fitness; }
  };

  //! number of samples per iteration, at least two, generally > 4
  size_t lambda;
  //! number of individuals used to recompute the mean
  size_t mu;
  //! step size
  T sigma;

  //! weights used to recombinate the mean sum up to one
  YaMatrix<T> weights;
  //! variance effective selection mass, should be lambda/4
  T mueff;

  //! cc^-1 (approx. n/4) is the backward time horizon for the evolution path pC and larger than one
  T cc;
  //! csigma^-1 (approx. n/3) is the backward time horizon for the evolution path pSigma and larger than one
  T csigma;
  //! learning rate of the rank-one update of the covariance matrix
  T c1;
  //! learning rate of the rank-mu update of the covariance matrix
  T cmu;
  //! damping parameter for step-size adaption, d = inifinity or 0 means adaption is turned off, usually close to one
  T dsigma;

  // constant terms in the algorithm
  T two_lambda;
  T hsigmin;
  T hsigbasis;
  T sigmaScale;
  T csInv;
  T ccInv;
  T muscale;
  T covScale;
  T covarianceDiscountFactor;
  T csigma_dsigma;
  //! expectation of ||N(0,I)|| == norm(randn(N,1))
  T expectedPSigmaNorm;

  // isotropic evolution path (for step length)
  YaMatrix<T> pSigma;
  // anisotropic evolution path (for covariance)
  YaMatrix<T> pC;
  // mean of the mating pool
  YaMatrix<T> mean;
  // covariance of the mating pool
  YaMatrix<T> covariance;
  // covariance for sampling
  YaMatrix<T> sampleCovariance;

  // pool of individuals in the search space
  std::vector<Individual> matingPool;
  // multivariate gaussian
  MultivariateGaussian<T> distribution;

  bool initialized;
  T minValues[N];
  T maxValues[N];
  T maxStepSize[N];

  R fitnessChange;
  R currentFitness;
  R convergenceThreshold;
  unsigned evaluations;
  unsigned iteration;
  unsigned maxIterations;
  unsigned currentIndividual;

public:

  CMAES(size_t mu = 4 + size_t(3.f* log(float(N))) / 2, size_t lambda = (4 + size_t(3.f* log(float(N)))), T stepSize = 0.5f)
    : lambda(lambda), mu(mu), sigma(stepSize),
      weights(mu, 1),
      pSigma(N, 1), pC(N, 1),
      mean(N, 1), covariance(N, N, 1.0f),
      sampleCovariance(N, N), distribution(mean, covariance),
      initialized(false)
  {
  }

  void getNextParameterSet(T newValues[N])
  {
    matingPool[currentIndividual] = distribution.sample();
    for(unsigned i = 0; i < N; ++i)
    {
      // clipping
      if(matingPool[currentIndividual][i][0] < minValues[i])
        matingPool[currentIndividual][i][0] = minValues[i];
      else if(matingPool[currentIndividual][i][0] > maxValues[i])
        matingPool[currentIndividual][i][0] = maxValues[i];
      if(matingPool[currentIndividual][i][0] - mean[i][0] > maxStepSize[i])
        matingPool[currentIndividual][i][0] = mean[i][0] + maxStepSize[i];
      else if(matingPool[currentIndividual][i][0] - mean[i][0] < -maxStepSize[i])
        matingPool[currentIndividual][i][0] = mean[i][0] - maxStepSize[i];

      newValues[i] = matingPool[currentIndividual][i][0];
    }
  }

  void getBestParameterSetAndReward(T bestValues[N], R& reward)
  {
    for(unsigned i = 0; i < N; ++i)
      bestValues[i] = matingPool[0][i][0];
    reward = currentFitness;
  }

  void setReward(R reward)
  {
    matingPool[currentIndividual++].fitness = reward;
    evaluations++;
    if(currentIndividual == lambda)
    {
      optimizationStep();
      currentIndividual = 0;
    }
  }

  bool hasReachedMinimum() const
  {
    return iteration >= maxIterations || fabs(fitnessChange) <= convergenceThreshold;
  }

  unsigned getIteration()
  {
    return iteration;
  }

  R getCurrentFitness()
  {
    return currentFitness;
  }

  bool isInitialized() const
  {
    return initialized;
  }

  void uninitialize()
  {
    initialized = false;
  }

  void init(T startValues[N], T minValues[N], T maxValues[N], T maxStepSize[N], R convergenceThreshold, int maxIterations)
  {
    ASSERT(mu < lambda);

    // main parameters of CMA-ES
    weights = YaMatrix<T>(mu, 1);
    const T firstWeight = (T)log(mu + 0.5);
    T sumOfWeights = 0.0f;
    for(size_t i = 0; i < mu; i++)
    {
      weights[i][0] = firstWeight - log(T(i + 1));
      ASSERT(weights[i][0] > 0.0f);
      sumOfWeights += weights[i][0];
    }
    weights /= sumOfWeights;

    mueff = 1.0f / weights.squaredVectorLength();

    cc = (4.0f + mueff / (T) N) / ((T) N + 4.0f + 2.0f * mueff / (T) N);
    csigma = (mueff + 2.0f) / ((T) N + mueff + 5.0f);
    c1 = 2.0f / (((T) N + 1.3f) * ((T) N + 1.3f) + mueff);
    cmu = 2.0f * (mueff - 2.0f + 1.0f / mueff) / ((T)((N + 2) * (N + 2)) + mueff);
    dsigma = 1.0f + 2.0f * std::max(0.0f, sqrtf((mueff - 1.0f) / T(N + 1)) - 1.0f) + csigma;

    // precalculated constant terms
    two_lambda = 2.0f / (T) lambda;
    hsigmin = 2.0f / (T)(N + 1);
    hsigbasis = 1.0f - (1.0f - csigma);
    sigmaScale = sqrt(csigma * (2.0f - csigma) * mueff);
    csInv = 1.0f - csigma;
    ccInv = 1.0f - cc;
    muscale = cc * (2.0f - cc);
    covScale = sqrt(muscale * mueff);
    covarianceDiscountFactor = 1.0f - c1 - cmu;
    csigma_dsigma = csigma / dsigma;
    expectedPSigmaNorm = sqrt((T) N) * (1.0f - 1.0f / (4.0f * (T) N) + 1.0f / (21.0f * (T)(N * N)));

    // CMA-ES variables
    pSigma = YaMatrix<T>(N, 1);
    pC = YaMatrix<T>(N, 1);
    covariance = YaMatrix<T>(N, N, 1.0f);
    sampleCovariance = YaMatrix<T>(N, N, sigma * sigma);
    mean = YaMatrix<T>(N, 1);
    for(unsigned i = 0; i < N; i++)
      mean[i][0] = startValues[i];

    matingPool = std::vector<Individual>(lambda, Individual(YaMatrix<T>(N, 1)));
    ASSERT(matingPool[0].M == N);
    ASSERT(matingPool[0].N == 1);
    distribution.setMean(mean);
    distribution.setCovariance(sampleCovariance);

    for(unsigned i = 0; i < N; i++)
    {
      this->minValues[i] = minValues[i];
      this->maxValues[i] = maxValues[i];
    }

    // iteration monitoring variables
    fitnessChange = std::numeric_limits<T>::max();
    currentFitness = std::numeric_limits<T>::max();
    this->convergenceThreshold = convergenceThreshold;
    evaluations = 0;
    iteration = 0;
    this->maxIterations = maxIterations;
    currentIndividual = 0;

    initialized = true;
  }

private:

  void optimizationStep()
  {
    // sort by fitness, ascending -> minimization
    std::sort(matingPool.begin(), matingPool.end());
    fitnessChange = matingPool[0].fitness - currentFitness;
    currentFitness = matingPool[0].fitness;

    // update mean
    YaMatrix<T> tempMean = mean;
    mean = YaMatrix<T>(N, 1);
    for(size_t i = 0; i < mu; i++)
    {
      mean += weights[i][0] * matingPool[i];
      ASSERT(matingPool[i].M == N);
      ASSERT(matingPool[i].N == 1);
    }
    YaMatrix<T> step = (mean - tempMean) / sigma;

    // cumulation: update step-size evolution path
    YaEigenDecomposition<T> eigen(covariance);
    eigen.solve();
    YaMatrix<T> invSqrtCov = eigen.eigenVectors * sqrt(eigen.eigenValues).diagonalInverse() * eigen.eigenVectors.transpose();
    ASSERT(invSqrtCov.M == N);
    ASSERT(invSqrtCov.N == N);
    pSigma = csInv * pSigma + sigmaScale * invSqrtCov * step;

    // cumulation: update covariance evolution path
    const T hsig = T((sqrt(pSigma.squaredVectorLength())
                      / sqrt(pow(hsigbasis, (T) evaluations * two_lambda))
                      / expectedPSigmaNorm) < T(1.4)) + hsigmin;
    pC = ccInv * pC + hsig * covScale * step;

    // adapt covariance matrix
    YaMatrix<T> rankMinMuLambdaMatrix(covariance.N, covariance.N);
    for(size_t i = 0; i < mu; i++)
    {
      YaMatrix<T> temp = (matingPool[i] - tempMean) / sigma;
      rankMinMuLambdaMatrix += weights[i][0] * temp * temp.transpose();
    }
    covariance = covarianceDiscountFactor * covariance // history
                 + c1 * (pC * pC.transpose() // ranke 1 update
                         + (1.0f - hsig) * muscale * covariance) // correction
                 + cmu * rankMinMuLambdaMatrix; // rank mu update

    // cumulative step-size adaption (CSA) / path length control
    T actualPSigmaNorm = sqrt(pSigma.squaredVectorLength());
    sigma *= exp(csigma_dsigma * (actualPSigmaNorm / expectedPSigmaNorm - 1.0f));

    // update distribution
    distribution.setMean(mean);
    sampleCovariance = covariance * sigma * sigma;
    distribution.setCovariance(sampleCovariance);

    iteration++;
  }
};
