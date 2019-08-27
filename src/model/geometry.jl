Base.@kwdef struct EuclideanPlane3D <: AbstractPlane
    coordinate_system::CartesianSystem = CartesianSystem()
end

function get_coordinate_system(plane::EuclideanPlane3D)
    plane.coordinate_system
end

function normal(plane::EuclideanPlane3D)
    get_e₃(plane.coordinate_system)
end

function origin(plane::EuclideanPlane3D)
    origin(plane.coordinate_system)
end

function distance(plane::EuclideanPlane3D)
    dot(normal(plane), origin(plane))
end

Base.@kwdef struct Line3D{T <: AbstractVector} <: AbstractPlane
    𝐩₁::T = Vec(0.0, 0.0, 0.0)
    𝐩₂::T = Vec(0.0, 0.0, 1.0)
end

function on_line(𝐩::AbstractVector, 𝓁::Line3D; tol = 1e-10)
    𝐯₁ =  𝓁.𝐩₁ - 𝐩
    𝐯₂ =  𝓁.𝐩₂ - 𝐩
    n = norm(cross(𝐯₁, 𝐯₂))
    @show n
    return  n <= tol ? true : false
    # x = 𝐩[1]
    # y = 𝐩[2]
    # z = 𝐩[3]
    #
    # x₁ = 𝓁.𝐩₁[1]
    # y₁ = 𝓁.𝐩₁[2]
    # z₁ = 𝓁.𝐩₁[3]
    #
    # x₂ = 𝓁.𝐩₂[1]
    # y₂ = 𝓁.𝐩₂[2]
    # z₂ = 𝓁.𝐩₂[3]
    #
    # v₁ = x₂ - x₁
    # v₂ = y₂ - y₁
    # v₃ = z₂ - z₁

    #return isapprox(v₁, v₂, tol) && isapprox(v₂, v₃, tol)
end
