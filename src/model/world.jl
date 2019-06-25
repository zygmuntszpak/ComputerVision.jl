abstract type AbstractWorld end

Base.@kwdef mutable struct PrimitiveWorld{T₁ <: Vec3, T₂ <: AbstractVector, T₃ <: Plane} <: AbstractWorld
    xaxis::T₁ = Vec(1.0, 0.0, 0.0)
    yaxis::T₁ = Vec(0.0, 1.0, 0.0)
    zaxis::T₁ = Vec(0.0, 0.0, 1.0)
    #cameras::Vector{T₁} = [DigitalCamera(), DigitalCamera()]
    points::Vector{T₂} = [Point3(rand(-1000.0:1000.0), rand(-1000.0:1000.0), 100.0) for n = 1:5000]
    planes::Vector{T₃} = [Plane(Vec3(0.0, 0.0, 1.0), 100)]
end

function get_xaxis(world::AbstractWorld)
    world.xaxis
end

function set_xaxis!(world::AbstractWorld, xaxis::Vec3)
    world.xaxis = xaxis
end

function get_yaxis(world::AbstractWorld)
    world.yaxis
end

function set_yaxis!(world::AbstractWorld, yaxis::Vec3)
    world.yaxis = yaxis
end

function get_zaxis(world::AbstractWorld)
    world.zaxis
end

function set_zaxis!(world::AbstractWorld, zaxis::Vec3)
    world.zaxis = zaxis
end

# function get_cameras(world::AbstractWorld)
#     world.cameras
# end
#
# function set_cameras!(world::AbstractWorld, cameras::Vector{<: AbstractCamera})
#     world.cameras = cameras
# end

function get_points(world::AbstractWorld)
    world.points
end

function set_points!(world::AbstractWorld, points::Vector{<: AbstractVector})
    world.points = points
end

function get_planes(world::AbstractWorld)
    world.planes
end

function set_planes!(world::AbstractWorld, planes::Vector{<: Plane})
    world.planes = planes
end
