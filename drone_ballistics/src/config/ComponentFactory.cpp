#include "config/ComponentFactory.h"
#include "interfaces/IBallisticsSolver.h"
#include "solvers/AnalyticalSolver.h"
#include "providers/JsonTargetProvider.h"
#include "config/FileConfigLoader.h"

#include <memory>

std::unique_ptr<IBallisticSolver> createSolver(SolverType type)
{
  switch (type) {
    case SolverType::ANALYTICAL:
      return std::make_unique<AnalyticalSolver>();
  }
  return nullptr;
};
std::unique_ptr<ITargetProvider> createProvider(ProviderType type)
{
  switch (type) {
    case ProviderType::JSON:
      return std::make_unique<JsonTargetProvider>();
  }
  return nullptr;
};
std::unique_ptr<IConfigLoader> createLoader(LoaderType type)
{
  switch (type) {
    case LoaderType::FILE:
      return std::make_unique<FileConfigLoader>();
  }
  return nullptr;
};
