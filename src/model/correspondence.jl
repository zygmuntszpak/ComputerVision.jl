import Base.getindex
import Base.length

abstract type AbstractCorrespondence end

struct Correspondence{N,T} <: AbstractCorrespondence
    x::NTuple{N,T}
end

function getindex(correspondence::Correspondence, index)
    getindex(correspondence.x, index)
end

function length(correspondence::Correspondence)
    length(correspondence.x)
end
