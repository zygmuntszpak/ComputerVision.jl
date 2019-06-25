using ComputerVision
using Test
using StaticArrays
using GeometryTypes
using LinearAlgebra

world = PrimitiveWorld()
@inferred PrimitiveWorld()

points = get_points(world)

planes = get_planes(world)
plane = first(planes)
𝐧 = get_normal(plane)
d = get_distance(plane)

# Verify that the default points lie on the plane
𝛑 = push(𝐧, -d) # 𝛑 =[n -d]
for 𝐱 in points
    @test isapprox(dot(𝛑, hom(𝐱)), 0.0; atol = 1e-14)
end
