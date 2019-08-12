using ComputerVision
using Test
using StaticArrays, GeometryTypes

@testset "ComputerVision.jl" begin
    # Write your own tests here.
    @testset "Fundamental Matrix Construction Test" begin include("fundamental_matrix.jl") end
    @testset "Corresponence Test" begin include("correspondence.jl") end
    @testset "Camera Test" begin include("camera.jl") end
    @testset "Planar Scene" begin include("planar_scene.jl") end
    @testset "Triangulate" begin include("triangulate_test.jl") end
end
