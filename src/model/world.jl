abstract type AbstractWorld end

Base.@kwdef mutable struct PrimitiveWorld{T₁ <: AbstractCoordinateSystem, T₂ <: AbstractVector, T₃ <: Union{Plane, PlaneSegment}, T₄ <: AbstractAllotment} <: AbstractWorld
    coordinate_system::T₁ = CartesianSystem(Point(0.0, 0.0, 0.0), Vec(1.0, 0.0, 0.0), Vec(0.0, 1.0, 0.0), Vec(0.0, 0.0, 1.0))
    points::Vector{T₂} = [Point3(rand(-1000.0:1000.0), rand(-1000.0:1000.0), 500.0) for n = 1:5000]
    groups::Vector{T₄} =  [IntervalAllotment(1:5000)]
    planes::Vector{T₃} = [Plane(Vec3(0.0, 0.0, 1.0), 500)]
end

Base.@kwdef mutable struct PlanarWorld{T₁ <: AbstractCoordinateSystem, T₂ <: AbstractVector, T₃ <: AbstractAllotment, T₄ <:  AbstractPlane} <: AbstractWorld
    coordinate_system::T₁ = CartesianSystem(Point(0.0, 0.0, 0.0), Vec(1.0, 0.0, 0.0), Vec(0.0, 1.0, 0.0), Vec(0.0, 0.0, 1.0))
    points::Vector{T₂} = [Point3(rand(-1000.0:1000.0), rand(-1000.0:1000.0), 0.0) for n = 1:5000]
    groups::Vector{T₃} =  [IntervalAllotment(1:5000)]
    planes::Vector{T₄} = [EuclideanPlane3D(CartesianSystem(Point(0.0, 0.0, 0.0), Vec(1.0, 0.0, 0.0), Vec(0.0, 1.0, 0.0), Vec(0.0, 0.0, 1.0)))]
end


# function get_xaxis(world::AbstractWorld)
#     world.xaxis
# end
#
# function set_xaxis!(world::AbstractWorld, xaxis::Vec3)
#     world.xaxis = xaxis
# end
#
# function get_yaxis(world::AbstractWorld)
#     world.yaxis
# end
#
# function set_yaxis!(world::AbstractWorld, yaxis::Vec3)
#     world.yaxis = yaxis
# end
#
# function get_zaxis(world::AbstractWorld)
#     world.zaxis
# end
#
# function set_zaxis!(world::AbstractWorld, zaxis::Vec3)
#     world.zaxis = zaxis
# end

# function get_cameras(world::AbstractWorld)
#     world.cameras
# end
#
# function set_cameras!(world::AbstractWorld, cameras::Vector{<: AbstractCamera})
#     world.cameras = cameras
# end

function get_coordinate_system(world::AbstractWorld)
    world.coordinate_system
end

function set_coordinate_system!(world::AbstractWorld, coordinate_system::AbstractCoordinateSystem)
    world.coordinate_system = coordinate_system
end

function get_points(world::AbstractWorld)
    world.points
end

function set_points!(world::AbstractWorld, points::Vector{<: AbstractVector})
    world.points = points
end

function set_groups!(world::AbstractWorld, groups::Vector{<: AbstractAllotment})
    world.groups = groups
end

function get_groups(world::AbstractWorld)
    world.groups
end

function get_planes(world::AbstractWorld)
    world.planes
end

function set_planes!(world::AbstractWorld, planes::Vector{<: Union{Plane,EuclideanPlane3D}})
    world.planes = planes
end
