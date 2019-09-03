abstract type AbstractCost end

Base.@kwdef struct ApproximateMaximumLikelihood <: AbstractCost end

struct AlgebraicLeastSquares <: AbstractCost end
struct MaximumLikelihood <: AbstractCost end
struct ReprojectionError <: AbstractCost end
Base.@kwdef struct Mahalanobis{T <: Union{AbstractVector, Nothing}} <: AbstractCost
    # Can optionally hold a vector of non-negative definite square root matrices
    ℬ::T = Nothing()
end
