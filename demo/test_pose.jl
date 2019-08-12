using ComputerVision
using Test
using StaticArrays
using GeometryTypes
using LinearAlgebra
using PGFPlotsX
using Makie
using Colors



pinhole₁ = Pinhole(intrinsics = IntrinsicParameters(width = 640, height = 480, focal_length = 100))
analogue_image₁ = AnalogueImage(coordinate_system = OpticalSystem())
camera₁ = ComputerVision.Camera(image_type = analogue_image₁, model = pinhole₁)
𝐑₁ = SMatrix{3,3,Float64,9}(rotxyz(90*(pi/180), -110*(pi/180), 0*(pi/180)))
#𝐑₁ = SMatrix{3,3,Float64,9}(rotxyz(0*(pi/180), 0*(pi/180), 0*(pi/180)))
𝐭₁ = [3000.0,0.0, 0.0]
# 𝐑₁ = SMatrix{3,3,Float64,9}(rotxyz(40*(pi/180), -130*(pi/180), 10*(pi/180)))
# 𝐭₁ = [3000.0,0.0, 0.0]

relocate!(camera₁, 𝐑₁, 𝐭₁)

pinhole₂ = Pinhole(intrinsics = IntrinsicParameters(width = 640, height = 480, focal_length = 100))
analogue_image₂ = AnalogueImage(coordinate_system = OpticalSystem())
camera₂ = ComputerVision.Camera(image_type = analogue_image₂, model = pinhole₂)
𝐑₂ = SMatrix{3,3,Float64,9}(rotxyz(90*(pi/180), -110*(pi/180), 0*(pi/180)))
𝐭₂ = [4000.0,0.0, 0.0]
relocate!(camera₂, 𝐑₂, 𝐭₂)

cameraₐ  = deepcopy(camera₁)
cameraᵦ  = deepcopy(camera₂)

default_world_system = CartesianSystem(Point(0.0, 0.0, 0.0), Vec(1.0, 0.0, 0.0), Vec(0.0, 1.0, 0.0), Vec(0.0, 0.0, 1.0))
alternative_world_system = get_coordinate_system(get_extrinsics(get_model(camera₁)))

intrinsics₁ = get_intrinsics(get_model(camera₁))
𝐊₁ = to_matrix(intrinsics₁, get_coordinate_system(analogue_image₁))
intrinsics₂ = get_intrinsics(get_model(camera₂))
𝐊₂ = to_matrix(intrinsics₁, get_coordinate_system(analogue_image₂))


#WorldCoordinateTransformationContext(CoordinateTransformation(source = alternative_world_system, target = alternative_world_system))


transformation_context! = WorldCoordinateTransformationContext(CoordinateTransformation(source = default_world_system, target = alternative_world_system))
transformation_context!(cameraₐ)
transformation_context!(cameraᵦ)

get_coordinate_system(get_extrinsics(get_model(cameraₐ)))
get_coordinate_system(get_extrinsics(get_model(cameraᵦ)))



#alternative_world_system
Fp = matrix(FundamentalMatrix(camera₁, camera₂, default_world_system))
Fp = Fp / norm(Fp)

Fq = matrix(FundamentalMatrix(cameraₐ, cameraᵦ, alternative_world_system))
Fq = Fq / norm(Fq)



A₁, B₁ = ascertain_pose(camera₁, default_world_system)
A₂, B₂ = ascertain_pose(camera₂, default_world_system)
𝐄₁ = [A₁' -A₁'*B₁]
𝐄₂ = [A₂' -A₂'*B₂]
𝐏₁ = 𝐊₁ * 𝐄₁
𝐏₂ = 𝐊₂ * 𝐄₂

F0 = matrix(FundamentalMatrix(Projection(𝐏₁), Projection(𝐏₂)))
F0 = F0 / norm(F0)

A = A₁
B = B₁
𝐄ₐ = [A₁'*A  (A₁'*B  - A₁'*B₁)]
𝐄ᵦ = [A₂'*A  (A₂'*B  - A₂'*B₂)]


𝐏ₐ = 𝐊₁ * 𝐄ₐ
𝐏ᵦ = 𝐊₂ * 𝐄ᵦ

F1 = matrix(FundamentalMatrix(Projection(𝐏ₐ), Projection(𝐏ᵦ)))
F1 = F1 / norm(F1)




F2 = matrix(FundamentalMatrix(cameraₐ, cameraᵦ, alternative_world_system))
F2 = F2 / norm(F2)



Aₐ, Bₐ = ascertain_pose(cameraₐ, alternative_world_system)

Aᵦ, Bᵦ = ascertain_pose(cameraᵦ, alternative_world_system)



F0 = matrix(FundamentalMatrix(camera₁, camera₂, default_world_system))
F0 = F0 / norm(F0)




F2 = matrix(FundamentalMatrix(cameraₐ, cameraᵦ, alternative_world_system))
F2 = F2 / norm(F2)

#
# # We have aligned the world coordinate system with camera 1
# ascertain_pose(camera₁, alternative_world_system)
# ascertain_pose(cameraₐ, alternative_world_system)
#
#
#
#
# ascertain_pose(camera₂, get_coordinate_system(get_extrinsics(get_model(camera₁))))
# ascertain_pose(cameraᵦ, get_coordinate_system(get_extrinsics(get_model(cameraₐ))))
#
# cameraₐ
#
# #alternative_world_system = CartesianSystem(Point(0.0, 0.0, 0.0), Vec(-1.0, 0.0, 0.0), Vec(0.0, -1.0, 0.0), Vec(0.0, 0.0, 1.0))
#
# #z = CoordinateTransformation(source = default_world_system, target = alternative_world_system)
#
# Point{3}(𝐭₁)

#
# ComputerVision.rotation(z)
# ComputerVision.translation(z)
#
# A₁, B₁ = ascertain_pose(camera₁, default_world_system)
# A₂, B₂ = ascertain_pose(camera₂, default_world_system)
#
#
# A, B = ascertain_pose(camera₂, get_coordinate_system(get_extrinsics(get_model(camera₁))))
#
#
# pinholeₐ = Pinhole(intrinsics = IntrinsicParameters(width = 640, height = 480, focal_length = 100))
# analogue_imageₐ = AnalogueImage(coordinate_system = OpticalSystem())
# cameraₐ = ComputerVision.Camera(image_type = analogue_imageₐ, model = pinholeₐ)
#
# pinholeᵦ = Pinhole(intrinsics = IntrinsicParameters(width = 640, height = 480, focal_length = 100))
# analogue_imageᵦ = AnalogueImage(coordinate_system = OpticalSystem())
# cameraᵦ = ComputerVision.Camera(image_type = analogue_imageᵦ, model = pinholeᵦ)
#
# #relative_pose = RelativePose(camera₁, camera₂)
# #𝐑ᵦ = ComputerVision.rotation(relative_pose)
# #𝐭ᵦ = ComputerVision.translation(relative_pose)
# relocate!(cameraᵦ, A , B )
#
# ascertain_pose(cameraᵦ, get_coordinate_system(get_extrinsics(get_model(cameraₐ))))
#
#
# F0 = matrix(FundamentalMatrix(camera₁, camera₂, default_world_system))
# F0 = F0 / norm(F0)
#
#
#
#
# F2 = matrix(FundamentalMatrix(cameraₐ, cameraᵦ, get_coordinate_system(get_extrinsics(get_model(cameraₐ)))))
# F2 = F2 / norm(F2)
