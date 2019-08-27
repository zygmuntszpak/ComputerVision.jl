using ComputerVision
using Test
using StaticArrays
using GeometryTypes
using LinearAlgebra
using PGFPlotsX
using Makie
using Colors
#using Makie # You have to add this as a dependency to your environment.

# points = [Point3(rand(0:1000.0), rand(-1000.0:1000.0), 1000.0) for n = 1:250]
# planes = [Plane(Vec3(0.0, 0.0, 1.0), 1000)]

# points₁ = [Point3(0.0, rand(-1000.0:1000.0), rand(0:1000.0)) for n = 1:250]
# planes₁ = [Plane(Vec3(1.0, 0.0, 0.0), 0)]
#
# points₂ = [Point3(rand(0:1000.0),  1000.0, rand(-1000.0:1000.0)) for n = 1:250]
# planes₂ = [Plane(Vec3(0.0, 0.0, 1.0), 1000)]

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

#planes = vcat(plane₁, plane₂)
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

# Verify that the default points lie on the plane
# 𝛑 = push(𝐧, -d) # 𝛑 =[n -d]
# for 𝐱 in points[251:500]
#     @test isapprox(dot(𝛑, hom(𝐱)), 0.0; atol = 1e-14)
# end

# camera₁ = ComputerVision.Camera(image_type = AnalogueImage(coordinate_system = OpticalSystem()))
# camera₂ = ComputerVision.Camera(image_type = AnalogueImage(coordinate_system = RasterSystem()))
# camera₃ = ComputerVision.Camera(image_type = AnalogueImage(coordinate_system = PlanarCartesianSystem()))



pinhole₁ = Pinhole(intrinsics = IntrinsicParameters(width = 640, height = 480, focal_length = 100))
analogue_image₁ = AnalogueImage(coordinate_system = OpticalSystem())
camera₁ = ComputerVision.Camera(image_type = analogue_image₁, model = pinhole₁)
𝐑₁ = SMatrix{3,3,Float64,9}(rotxyz(90*(pi/180), -130*(pi/180), 0*(pi/180)))
𝐭₁ = [3000.0,0.0, 0.0]
relocate!(camera₁, 𝐑₁, 𝐭₁)

# Q, q = ascertain_pose(camera₁, CartesianSystem(Vec(1.0, 0.0, 0.0), Vec(0.0, 1.0, 0.0), Vec(0.0, 0.0, 1.0)))
# inv(Q)
# Q = to_matrix(get_extrinsics(get_model(camera₁)), CartesianSystem(Vec(1.0, 0.0, 0.0), Vec(0.0, 1.0, 0.0), Vec(0.0, 0.0, 1.0)))
# inv(Q[1:3,1:3]) * Q[:,4]

pinhole₂ = Pinhole(intrinsics = IntrinsicParameters(width = 640, height = 480, focal_length = 150))
analogue_image₂ = AnalogueImage(coordinate_system = OpticalSystem())
camera₂ = ComputerVision.Camera(image_type = analogue_image₂, model = pinhole₂)
𝐑₂ = SMatrix{3,3,Float64,9}(rotxyz(90*(pi/180), -110*(pi/180), 0*(pi/180)))
𝐭₂ = [4000.0,0.0, 0.0]
relocate!(camera₂, 𝐑₂, 𝐭₂)


# p = first(project(Projection(camera₁), [SVector(𝐭₁...)]))
# 𝐊₁ = to_matrix(get_intrinsics(pinhole₁), get_coordinate_system(analogue_image₁))

# 𝐏 = to_matrix(Projection(camera₁))
# 𝐏 * hom(SVector(𝐭₁...))
# 𝐊₁ * hom(p)


# Q = to_matrix(get_extrinsics(get_model(camera₂)), CartesianSystem(Vec(1.0, 0.0, 0.0), Vec(0.0, 1.0, 0.0), Vec(0.0, 0.0, 1.0)))
# inv(Q[1:3,1:3]) * Q[:,4]

# to_matrix(get_intrinsics(get_model(camera₂)))
# get_intrinsics(get_model(camera₁))

# visualize = VisualizeWorldContext(; visual_properties = MakieVisualProperties(scale = 150, markersize = 25))
# visualize(world, [camera₁, camera₂])
# scene = get_scene(visualize)
# axis = scene[Axis]
# axis[:showaxis] = (false, false, false)
# axis[:showgrid] = (false, false, false)
# display(scene)

axis = @pgf PGFPlotsX.Axis({axis_equal="true", view="{60}{30}"});
visualize₂ = VisualizeWorldContext(; scene = axis, visual_properties = PGFPlotsVisualProperties(scale = 150, markersize = 25))
visualize₂(world, [camera₁, camera₂])
scene₂ = get_scene(visualize₂)
display(scene₂)


aquire = AquireImageContext()
ℳ = aquire(world, camera₁)
ℳ′ = aquire(world, camera₂)

# Verify back projection...
# L₁ = back_project(camera₁, ℳ)
# L₂ = back_project(camera₂, ℳ′)
# world.points[1]
# on_line(world.points[end], last(L₁); tol = 1e-2)
# on_line(world.points[end], last(L₂); tol = 1e-2)


axis₃ = @pgf PGFPlotsX.Axis({axis_equal="true", view="{60}{30}"});
visualize₃ = VisualizeWorldContext(; scene = axis₃, visual_properties = PGFPlotsVisualProperties(scale = 150, markersize = 25))
visualize₃(ℳ, groups)
scene₃ = get_scene(visualize₃)
display(scene₃)

axis₄ = @pgf PGFPlotsX.Axis({axis_equal="true", view="{60}{30}"});
visualize₄ = VisualizeWorldContext(; scene = axis₄, visual_properties = PGFPlotsVisualProperties(scale = 150, markersize = 25))
visualize₄(ℳ′, groups)
scene₄ = get_scene(visualize₄)
display(scene₄)


𝒞 = Correspondences((ℳ,ℳ′))
normalize_data = HartleyNormalizeDataContext(𝒞 )
𝒞′ = normalize_data(𝒞)

estimate = EstimateHomographyContext()
estimate(𝒞′, DirectLinearTransform())
estimate(𝒞′, DirectLinearTransform())


#typeof(𝒞)

# # Wrong formula?!
# 𝐅 = matrix(FundamentalMatrix(camera₁, camera₂))
# 𝐅 = 𝐅 / norm(𝐅)
#
# 𝐅2 = matrix(FundamentalMatrix(Projection(camera₁), Projection(camera₂)))
# 𝐅2 = 𝐅2 / norm(𝐅2)
#
#
# for i in 1:length(ℳ)
#     𝐦₁ = hom(ℳ[i])
#     𝐦₂ = hom(ℳ′[i])
#     r = 𝐦₂' *  𝐅 * 𝐦₁
# end
#
# r = [hom(ℳ′[i])' * 𝐅  * hom(ℳ[i]) for i = 1:length(ℳ)]
#
# 𝐇 = matrix(HomographyMatrix(camera₁, camera₂, first(plane₁)))
# 𝐇 = 𝐇 / norm(𝐇)
#
# s = [vec2antisym(hom(ℳ′[i])) * 𝐇  * hom(ℳ[i]) for i = 1:length(ℳ)]
#
# project(Projection(camera₂), [SVector{3}(𝐭₁), SVector{3}(𝐭₂)])
#
# r[5]
# s[end]
#
# z = [vec2antisym(hom(ℳ′[i])) * 𝐇  * hom(ℳ[i]) for i = 1:250]
#
# 𝐇₂ = matrix(HomographyMatrix(camera₁, camera₂, first(plane₂)))
#
#
#
# 𝐇₂ = 𝐇₂ / norm(𝐇₂)
#
#
# z = [vec2antisym(hom(ℳ′[i])) * 𝐇₂  * hom(ℳ[i]) for i = 251:500]
#
# 𝐇'*𝐅 + 𝐅'*𝐇
#
# 𝐇₂'*𝐅 + 𝐅'*𝐇₂
#
#
# Projection(camera₁)
