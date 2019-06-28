import Base.getindex
import Base.length

abstract type AbstractCorrespondences end

struct Correspondences{N,T} <: AbstractCorrespondences
    x::NTuple{N,T}
end

function getindex(correspondences::Correspondences, index)
    getindex(correspondences.x, index)
end

function length(correspondences::Correspondences)
    length(correspondences.x)
end
