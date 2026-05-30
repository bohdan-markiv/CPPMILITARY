#include "config/ComponentFactory.h"
#include "interfaces/IBallisticsSolver.h"
#include "solvers/AnalyticalSolver.h"
#include "providers/JsonTargetProvider.h"
#include "config/FileConfigLoader.h"

#include <stdexcept>

IBallisticSolver *createSolver(SolverType type)
{
  switch (type) {
    case SolverType::ANALYTICAL:
      return new AnalyticalSolver();
  }
  throw std::runtime_error("Unknown solver type");
};
ITargetProvider *createProvider(ProviderType type)
{
  switch (type) {
    case ProviderType::JSON:
      return new JsonTargetProvider();
  }
  throw std::runtime_error("Unknown provider type");
};
IConfigLoader *createLoader(LoaderType type)
{
  switch (type) {
    case LoaderType::FILE:
      return new FileConfigLoader();
  }
  throw std::runtime_error("Unknown loader type");
};
