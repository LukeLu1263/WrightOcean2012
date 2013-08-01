/**
* @file Tools/Optimization/ParticleSwarm.h
* Implementation of a simple particle swarm optimization algorithm.
* @author Colin Graf
*/

#include <limits>

#include "ParticleSwarm.h"
#include "Tools/Math/Common.h"
#include "Tools/Math/Random.h"
#include "Tools/Streams/OutStreams.h"
#include "Tools/Streams/InStreams.h"
#include "Platform/BHAssert.h"

ParticleSwarm::ParticleSwarm(bool minimize, unsigned int particleCount,
                             const float& velocityFactor, const float& bestPositionFactor,
                             const float& globalBestPositionFactor, const float& randomPositionFactor) :
  minimize(minimize),
  velocityFactor(velocityFactor), bestPositionFactor(bestPositionFactor),
  globalBestPositionFactor(globalBestPositionFactor), randomPositionFactor(randomPositionFactor),
  particles(particleCount) {}

ParticleSwarm::~ParticleSwarm()
{
  if(!file.empty())
  {
    OutBinaryFile stream(file);
    if(stream.exists())
    {
      stream << rev;
      stream << (unsigned int)limits.size();
      stream << (unsigned int)particles.size();
      stream << *this;
    }
  }
}

bool ParticleSwarm::isInitialized() const
{
  return !limits.empty();
}

void ParticleSwarm::init(const std::string& file, const float values[][3], unsigned int size, unsigned int rev)
{
  this->file = file;
  this->rev = rev;
  limits.resize(size);
  for(unsigned int j = 0; j < size; ++j)
  {
    limits[j].min = limits[j].max = values[j][1];
    limits[j].add(values[j][2]);
    ASSERT(limits[j].isInside(values[j][0])); // start value not in value range!
  }

  if(!file.empty())
  {
    InBinaryFile stream(file);
    if(stream.exists())
    {
      unsigned int fileRev, fileDim, fileParticleCount;
      stream >> fileRev >> fileDim >> fileParticleCount;
      if(rev == fileRev && fileDim == size && fileParticleCount == particles.size())
      {
        stream >> *this;
        ASSERT(particles[0].position.size() == size);
        ASSERT(particles[0].velocity.size() == size);
        ASSERT(particles[0].bestPosition.size() == size);
        for(unsigned int i = 0; i < particles.size(); ++i)
        {
          Particle& particle(particles[i]);
          particle.bestFitness = minimize ? std::numeric_limits<float>::max() : std::numeric_limits<float>::min();
          particle.rated = false;
        }
        bestParticleIndex = 0;
        currentParticleIndex = particles.size() - 1;
        return;
      }
    }
  }

  for(unsigned int i = 0; i < particles.size(); ++i)
  {
    Particle& particle(particles[i]);
    particle.position.resize(size);
    particle.velocity.resize(size);
    for(unsigned int j = 0; j < size; ++j)
    {
      float random = i == 0 ? 0.f : (float) randomFloat(); // the first particles consists of the given start values
      particle.position[j] = values[j][0] + (random >= 0.5f ?
                                             (values[j][1] - values[j][0]) * (random - 0.5f) * 2.f : (values[j][2] - values[j][0]) * random * 2.f);
    }
    particle.bestPosition = particle.position;
    particle.bestFitness = minimize ? std::numeric_limits<float>::max() : std::numeric_limits<float>::min();
    particle.rated = false;
  }
  bestParticleIndex = 0;
  currentParticleIndex = particles.size() - 1;
}

void ParticleSwarm::getNextValues(float*& values)
{
  currentParticleIndex = (currentParticleIndex + 1) % particles.size();
  updateParticle(particles[currentParticleIndex]);
  values = &particles[currentParticleIndex].position[0];
}

void ParticleSwarm::setFitness(const float& fitness)
{
  Particle& particle(particles[currentParticleIndex]);
  if((minimize && fitness < particle.bestFitness) || (!minimize && fitness > particle.bestFitness))
  {
    particle.bestFitness = fitness;
    particle.bestPosition = particle.position;
  }
  particle.rated = true;

  Particle& bestParticle(particles[bestParticleIndex]);
  if((minimize && fitness < bestParticle.bestFitness) || (!minimize && fitness > bestParticle.bestFitness))
    bestParticleIndex = currentParticleIndex;
}

bool ParticleSwarm::isRated()
{
  return particles[currentParticleIndex].rated;
}

void ParticleSwarm::getBestValues(float*& values)
{
  values = &particles[bestParticleIndex].bestPosition[0];
}

float ParticleSwarm::getBestFitness()
{
  return particles[bestParticleIndex].bestFitness;
}

void ParticleSwarm::updateParticle(Particle& particle)
{
  if(!particle.rated)
    return;
  Particle& bestParticle(particles[bestParticleIndex]);
  for(unsigned int i = 0; i < particle.position.size(); ++i)
  {
    particle.velocity[i] = velocityFactor * particle.velocity[i]
                           + bestPositionFactor * randomFloat() * (particle.bestPosition[i] - particle.position[i])
                           + globalBestPositionFactor * randomFloat() * (bestParticle.bestPosition[i] - particle.position[i])
                           + randomPositionFactor * randomFloat() * ((limits[i].min + randomFloat() * (limits[i].max - limits[i].min)) - particle.position[i]);
    if(limits[i].isInside(particle.position[i] + particle.velocity[i]))
      particle.position[i] += particle.velocity[i];
    else
    {
      particle.position[i] = limits[i].limit(particle.position[i] + particle.velocity[i]);
      particle.velocity[i] = 0.;
    }
  }
  particle.rated = false;
}
