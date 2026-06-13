#pragma once
#include "Types.h"
#include "interfaces/IBallisticsSolver.h"
#include "interfaces/ITargetProvider.h"
#include "interfaces/IConfigLoader.h"

std::unique_ptr<IBallisticSolver> createSolver(SolverType type);
std::unique_ptr<ITargetProvider> createProvider(ProviderType type);
std::unique_ptr<IConfigLoader> createLoader(LoaderType type);