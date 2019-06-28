using ComputerVision
using Test
using StaticArrays
using GeometryTypes
using LinearAlgebra
using PGFPlotsX
using Makie
using Colors
#using Makie # You have to add this as a dependency to your environment.

x₁ = 0.0
x₁′ = 0.0
y₁ = -1000.0
y₁′ = 2000.0
z₁ = -1000.0
z₁′ = 1000.0
points₁ = [Point3(rand(x₁:x₁′), rand(y₁:y₁′), rand(z₁:z₁′)) for n = 1:250]
plane₁ = [Plane(Vec3(1.0, 0.0, 0.0), 0)]
p₁ = [x₁, y₁, z₁]
q₁ = [x₁, y₁′, z₁]
r₁ = [x₁, y₁′, z₁′]
s₁ = [x₁, y₁, z₁′]
segment₁ = [p₁ => q₁ , q₁ => r₁ , r₁ => s₁ , s₁ => p₁]
plane_segment₁ = PlaneSegment(first(plane₁), segment₁)

#segment₁ = [[0, -1000, -1000] => [0, 2000, -1000], [0, 2000, -1000] => [0, 2000, 1000], [0, 2000, 1000] =>   [0, -1000, 1000]]

x₂ = 0.0
x₂′ = 3000.0
y₂ = 2000.0
y₂′ = 2000.0
z₂ = -1000.0
z₂′ = 1000.0
points₂ = [Point3(rand(x₂:x₂′), rand(y₂:y₂′), rand(z₂:z₂′)) for n = 1:250]
plane₂ = [Plane(Vec3(0.0, 1.0, 0.0), 2000)]
p₂ = [x₂, y₂, z₂]
q₂ = [x₂, y₂, z₂′]
r₂ = [x₂′, y₂, z₂′]
s₂ = [x₂′, y₂, z₂]
segment₂ = [p₂ => q₂ , q₂ => r₂ , r₂ => s₂ , s₂ => p₂]
plane_segment₂ = PlaneSegment(first(plane₂), segment₂)

planes = vcat(plane_segment₁, plane_segment₂)
points = vcat(points₁, points₂)

groups = [IntervalAllotment(1:250), IntervalAllotment(251:500)]

world = PrimitiveWorld(points = points, planes = planes, groups = groups)
@inferred PrimitiveWorld()

points = get_points(world)

planes = get_planes(world)
plane = first(planes)
𝐧 = get_normal(plane)
d = get_distance(plane)

𝛑 = push(𝐧, -d) # 𝛑 =[n -d]
for 𝐱 in points[1:250]
    @test isapprox(dot(𝛑, hom(𝐱)), 0.0; atol = 1e-14)
end


pinhole₁ = Pinhole(intrinsics = IntrinsicParameters(width = 640, height = 480, focal_length = 100))
analogue_image₁ = AnalogueImage(coordinate_system = OpticalSystem())
camera₁ = ComputerVision.Camera(image_type = analogue_image₁, model = pinhole₁)
𝐑₁ = SMatrix{3,3,Float64,9}(rotxyz(90*(pi/180), -110*(pi/180), 0*(pi/180)))
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



#axis = @pgf PGFPlotsX.Axis({axis_equal="true", view="{60}{30}"});
axis = Scene()
visualize₂ = VisualizeWorldContext(; scene = axis, visual_properties = MakieVisualProperties(scale = 150, markersize = 25))
visualize₂(world, [camera₁, camera₂])
scene₂ = get_scene(visualize₂)
display(scene₂)

aquire = AquireImageContext()
ℳ = aquire(world, camera₁)
ℳ′ = aquire(world, camera₂)


#RelativePose(camera₂, camera₁)

default_world_system = CartesianSystem(Vec(1.0, 0.0, 0.0), Vec(0.0, 1.0, 0.0), Vec(0.0, 0.0, 1.0))
alternative_world_system = CartesianSystem(Vec(-1.0, 0.0, 0.0), Vec(0.0, -1.0, 0.0), Vec(0.0, 0.0, 1.0))


F0 = matrix(FundamentalMatrix(camera₁, camera₂, default_world_system))
F0 = F0 / norm(F0)

F1 = matrix(FundamentalMatrix(camera₁, camera₂, alternative_world_system))
F1 = F1 / norm(F1)

pinholeₐ = Pinhole(intrinsics = IntrinsicParameters(width = 640, height = 480, focal_length = 100))
analogue_imageₐ = AnalogueImage(coordinate_system = OpticalSystem())
cameraₐ = ComputerVision.Camera(image_type = analogue_imageₐ, model = pinholeₐ)

pinholeᵦ = Pinhole(intrinsics = IntrinsicParameters(width = 640, height = 480, focal_length = 100))
analogue_imageᵦ = AnalogueImage(coordinate_system = OpticalSystem())
cameraᵦ = ComputerVision.Camera(image_type = analogue_imageᵦ, model = pinholeᵦ)
relative_pose = RelativePose(camera₁, camera₂)
𝐑ᵦ = ComputerVision.rotation(relative_pose)
𝐭ᵦ = ComputerVision.translation(relative_pose)
relocate!(cameraᵦ, 𝐑ᵦ , 𝐭ᵦ )

camera_model₁ = get_model(camera₁)
camera_model₂ = get_model(camera₂)
intrinsics₁ =  get_intrinsics(camera_model₁)
intrinsics₂ =  get_intrinsics(camera_model₂)
extrinsics₁ =  get_extrinsics(camera_model₁)
extrinsics₂ =  get_extrinsics(camera_model₂)




Projection(cameraₐ)
Projection(cameraᵦ)
F2 = matrix(FundamentalMatrix(cameraₐ, cameraᵦ, alternative_world_system))

F2 = F2 / norm(F2)


F1
# axis₃ = @pgf PGFPlotsX.Axis({axis_equal="true", view="{60}{30}"});
# visualize₃ = VisualizeWorldContext(; scene = axis₃, visual_properties = PGFPlotsVisualProperties(scale = 150, markersize = 25))
# visualize₃(ℳ, groups)
# scene₃ = get_scene(visualize₃)
# display(scene₃)
#
# axis₄ = @pgf PGFPlotsX.Axis({axis_equal="true", view="{60}{30}"});
# visualize₄ = VisualizeWorldContext(; scene = axis₄, visual_properties = PGFPlotsVisualProperties(scale = 150, markersize = 25))
# visualize₄(ℳ′, groups)
# scene₄ = get_scene(visualize₄)
# display(scene₄)

# 𝐇 = matrix(HomographyMatrix(camera₁, camera₂, first(plane₁)))
# 𝐇 = 𝐇 / norm(𝐇)
#
# determine_pose = PoseFromSingleHomographyContext(get_intrinsics(pinhole₁), analogue_image₁, MalisVargasDecomposition())
# poses = determine_pose(HomographyMatrix(camera₁, camera₂, first(plane₁)))
#
# 𝒞 = Correspondences((ℳ,ℳ′))
# valid_poses = determine_pose(HomographyMatrix(camera₁, camera₂, first(plane₁)), 𝒞)
#
# RelativePose(camera₁, camera₂)
# RelativePose(camera₂, camera₁)
#
#
# pinhole₁ = Pinhole(intrinsics = IntrinsicParameters(width = 640, height = 480, focal_length = 100))
# analogue_image₁ = AnalogueImage(coordinate_system = OpticalSystem())
# canonical_camera₁ = ComputerVision.Camera(image_type = analogue_image₁, model = pinhole₁)
#
# pinhole₂ = Pinhole(intrinsics = IntrinsicParameters(width = 640, height = 480, focal_length = 100))
# analogue_image₂ = AnalogueImage(coordinate_system = OpticalSystem())
# reference_camera₂ = ComputerVision.Camera(image_type = analogue_image₂, model = pinhole₂)
#
# 𝐑₂ = rotation(RelativePose(camera₁, camera₂))
# 𝐭₂ = ComputerVision.translation(RelativePose(camera₁, camera₂))
# relocate!(reference_camera₂, 𝐑₂, 𝐭₂)
#
# triangulate_points =  TriangulateContext(DirectLinearTriangulation())
# pts3D = triangulate_points(canonical_camera₁, reference_camera₂, 𝒞)
#
# reconstructed_world = PrimitiveWorld(points = pts3D, planes =  planes, groups = groups)
# axis = @pgf PGFPlotsX.Axis({axis_equal="true", view="{60}{30}"});
# visualize₂ = VisualizeWorldContext(; scene = axis, visual_properties = PGFPlotsVisualProperties(scale = 150, markersize = 25))
# visualize₂(reconstructed_world, [canonical_camera₁, reference_camera₂])
# scene₂ = get_scene(visualize₂)
# display(scene₂)

#
# pinhole₁ = Pinhole(intrinsics = IntrinsicParameters(width = 640, height = 480, focal_length = 100))
# analogue_image₁ = AnalogueImage(coordinate_system = OpticalSystem())
# camera₁ = ComputerVision.Camera(image_type = analogue_image₁, model = pinhole₁)
#
# pinhole₂ = Pinhole(intrinsics = IntrinsicParameters(width = 640, height = 480, focal_length = 100))
# analogue_image₂ = AnalogueImage(coordinate_system = OpticalSystem())
# camera₂ = ComputerVision.Camera(image_type = analogue_image₂, model = pinhole₂)
# 𝐑₂ = valid_poses[1][1]
# 𝐭₂ = valid_poses[1][2]
# relocate!(camera₂, inv(𝐑₂), -𝐭₂)
#
# 𝒞 = Correspondences((ℳ[1:250],ℳ′[1:250]))
# triangulate_points =  TriangulateContext(DirectLinearTriangulation())
# pts3D = triangulate_points(camera₁, camera₂, 𝒞) * 1
# # 𝐧 = valid_poses[2][3]
# # d = -1
# #
# # 𝛑 = push(𝐧, -d) # 𝛑 =[n -d]
# # for 𝐱 in pts3D[1:250]
# #     @test isapprox(dot(𝛑, hom(𝐱)), 0.0; atol = 1e-14)
# # end
#
#
#
# reconstructed_world = PrimitiveWorld(points = pts3D, planes =  [Plane(Vec3(valid_poses[2][3]...), 1.0)], groups = [IntervalAllotment(1:250)])
# axis = @pgf PGFPlotsX.Axis({axis_equal="true", view="{60}{30}"});
# visualize₂ = VisualizeWorldContext(; scene = axis, visual_properties = PGFPlotsVisualProperties(scale = 150, markersize = 25))
# visualize₂(reconstructed_world, [camera₁, camera₂])
# scene₂ = get_scene(visualize₂)
# display(scene₂)

# for couple in zip(pts3D, points)
#     @test isapprox(norm(first(couple)-last(couple)), 0.0; atol = 1e-7)
# end



#
# reconstructed_world = PrimitiveWorld(points = pts3D, planes = planes, groups = groups)
# axis = @pgf PGFPlotsX.Axis({axis_equal="true", view="{60}{30}"});
# visualize₂ = VisualizeWorldContext(; scene = axis, visual_properties = PGFPlotsVisualProperties(scale = 150, markersize = 25))
# visualize₂(reconstructed_world, [camera₁, camera₂])
# scene₂ = get_scene(visualize₂)
# display(scene₂)

@show 𝒞
# R, t, n = poses[3]
# svd(𝐇)
#
# R'*R
#
# det(R)
#
# get_intrinsics(pinhole₁)










get_intrinsics(pinhole₂)
