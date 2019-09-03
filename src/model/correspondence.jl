import Base.getindex
import Base.length

abstract type AbstractCorrespondences end

struct Correspondences{N,T} <: AbstractCorrespondences
    x::NTuple{N,T}
end

# TODO add corresponding covariance type

function getindex(correspondences::Correspondences, index)
    getindex(correspondences.x, index)
end

function length(correspondences::Correspondences)
    length(correspondences.x)
end

function extract_subset(data::AbstractCorrespondences, index::AbstractRange)
     data₁ = data[1]
     data₂ = data[2]
     subset₁ = data₁[index]
     subset₂ = data₂[index]
     Correspondences(tuple(subset₁, subset₂))
end
