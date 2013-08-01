/**
* @file Tools/Optimization/ParticleSwarm.h
* Implementation of a simple particle swarm optimization algorithm.
* @author Colin Graf
*/

#pragma once

#include <vector>
#include <string>

#include "Tools/Range.h"
#include "Tools/Streams/Streamable.h"

class ParticleSwarm : public Streamable
{
public:
  ParticleSwarm(bool minimize = true, unsigned int particleCount = 8,
                const float& velocityFactor = 0.6, const float& bestPositionFactor = 0.8,
                const float& globalBestPositionFactor = 1., const float& randomPositionFactor = 0.02);
  ~ParticleSwarm();

  template <int size> void init(const std::string& file, const float(&values)[size][3], unsigned int rev) {init(file, values, size, rev);}

  bool isInitialized() const;

  void getNextValues(float*& values);

  void setFitness(const float& fitness);

  bool isRated();

  void getBestValues(float*& values);

  float getBestFitness();

private:
  class Particle : public Streamable
  {
  public:
    Particle() {}

    std::vector<float> position;
    std::vector<float> velocity;

    std::vector<float> bestPosition;
    float bestFitness;

    bool rated;

  private:
    virtual void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN();
      STREAM(position);
      STREAM(velocity);
      STREAM(bestPosition);
      STREAM(bestFitness);
      STREAM(rated);
      STREAM_REGISTER_FINISH();
    }
  };

  bool minimize;
  float velocityFactor;
  float bestPositionFactor;
  float globalBestPositionFactor;
  float randomPositionFactor;
  std::string file;
  unsigned int rev;
  std::vector<Range<> > limits;

  std::vector<Particle> particles;
  unsigned int bestParticleIndex;
  unsigned int currentParticleIndex;

  void init(const std::string& file, const float values[][3], unsigned int size, unsigned int rev);
  void updateParticle(Particle& particle);

  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN();
    STREAM(particles);
    STREAM(bestParticleIndex);
    STREAM(currentParticleIndex);
    STREAM_REGISTER_FINISH();
  }
};
