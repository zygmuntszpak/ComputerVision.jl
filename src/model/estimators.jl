abstract type AbstractEstimationAlgorithm end
abstract type AbstractProjectiveEstimationAlgorithm <: AbstractEstimationAlgorithm end
struct DirectLinearTransform  <: AbstractProjectiveEstimationAlgorithm end
