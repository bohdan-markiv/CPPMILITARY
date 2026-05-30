#pragma once
#include "Types.h"
#include "interfaces/IBallisticsSolver.h"
#include "interfaces/ITargetProvider.h"
#include "interfaces/IConfigLoader.h"

IBallisticSolver *createSolver(SolverType type);
ITargetProvider *createProvider(ProviderType type);
IConfigLoader *createLoader(LoaderType type);