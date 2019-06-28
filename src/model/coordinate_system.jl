abstract type AbstractCoordinateSystem end
abstract type AbstractPlanarCoordinateSystem <: AbstractCoordinateSystem end

Base.@kwdef struct PlanarCartesianSystem{T <: AbstractVector} <: AbstractPlanarCoordinateSystem
    𝐞₁::T = Vec(-1.0, 0.0)
    𝐞₂::T = Vec(0.0, 1.0)
end

Base.@kwdef struct CartesianSystem{T <: AbstractVector} <: AbstractCoordinateSystem
    𝐞₁::T = Vec(1.0, 0.0, 0.0)
    𝐞₂::T = Vec(0.0, 1.0, 0.0)
    𝐞₃::T = Vec(0.0, 0.0, 1.0)
end

Base.@kwdef struct RasterSystem{T <: AbstractVector} <: AbstractPlanarCoordinateSystem
    𝐞₁::T = Vec(-1.0, 0.0)
    𝐞₂::T = Vec(0.0, -1.0)
end

Base.@kwdef struct OpticalSystem{T <: AbstractVector} <: AbstractPlanarCoordinateSystem
    𝐞₁::T = Vec(-1.0, 0.0)
    𝐞₂::T = Vec(0.0, -1.0)
end

function get_e₁(param::AbstractCoordinateSystem)
    param.𝐞₁
end

function get_e₂(param::AbstractCoordinateSystem)
    param.𝐞₂
end

function get_e₃(param::AbstractCoordinateSystem)
    param.𝐞₃
end
