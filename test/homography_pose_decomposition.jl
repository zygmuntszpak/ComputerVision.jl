using ComputerVision
using Test
using StaticArrays
using GeometryTypes
using LinearAlgebra
using PGFPlotsX
using Makie
using Colors

# Generate points on two planar surfaces
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

planes = vcat(plane₁, plane₂)
#planes = vcat(plane_segment₁, plane_segment₂)
points = vcat(points₁, points₂)

groups = [IntervalAllotment(1:250), IntervalAllotment(251:500)]

world = PrimitiveWorld(points = points, planes = planes, groups = groups)

pinhole₁ = Pinhole(intrinsics = IntrinsicParameters(width = 640, height = 480, focal_length = 100))
analogue_image₁ = AnalogueImage(coordinate_system = OpticalSystem())
camera₁ = ComputerVision.Camera(image_type = analogue_image₁, model = pinhole₁)
𝐑₁ = SMatrix{3,3,Float64,9}(rotxyz(90*(pi/180), -110*(pi/180), 0*(pi/180)))
𝐭₁ = [3000.0,0.0, 0.0]
relocate!(camera₁, 𝐑₁, 𝐭₁)

pinhole₂ = Pinhole(intrinsics = IntrinsicParameters(width = 640, height = 480, focal_length = 100))
analogue_image₂ = AnalogueImage(coordinate_system = OpticalSystem())
camera₂ = ComputerVision.Camera(image_type = analogue_image₂, model = pinhole₂)
𝐑₂ = SMatrix{3,3,Float64,9}(rotxyz(90*(pi/180), -110*(pi/180), 0*(pi/180)))
𝐭₂ = [4000.0,0.0, 0.0]
relocate!(camera₂, 𝐑₂, 𝐭₂)


aquire = AquireImageContext()

# Project 3D points onto the cameras.
ℳ = aquire(world, camera₁)
ℳ′ = aquire(world, camera₂)

cameraₐ  = deepcopy(camera₁)
cameraᵦ  = deepcopy(camera₂)
world₂ = deepcopy(world)

default_world_system = CartesianSystem(Point(0.0, 0.0, 0.0), Vec(1.0, 0.0, 0.0), Vec(0.0, 1.0, 0.0), Vec(0.0, 0.0, 1.0))
alternative_world_system = get_coordinate_system(get_extrinsics(get_model(camera₁)))

transformation_context! = WorldCoordinateTransformationContext(CoordinateTransformation(source = default_world_system, target = alternative_world_system))
transformation_context!(cameraₐ)
transformation_context!(cameraᵦ)
transformation_context!(world₂)

# Project transformed 3D points onto the cameras.
ℳₐ = aquire(world₂, cameraₐ)
ℳᵦ′ = aquire(world₂, cameraᵦ)


# Verify that the original 3D points lie on their corresponding planes.
points3D = get_points(world)
planes3D = get_planes(world)
for (i, plane3D) in enumerate(planes3D)
    subset = points3D[get_interval(groups[i])]
    for pt in subset
        @test on_plane(pt, plane3D; tol = 1e-10)
    end
end

# Verify that the transformed 3D points lie on their corresponding transformed planes.
points3Dᵦ = get_points(world₂)
planes3Dᵦ = get_planes(world₂)
for (i, plane3Dᵦ) in enumerate(planes3Dᵦ)
    subset = points3Dᵦ[get_interval(groups[i])]
    for pt in subset
        @test on_plane(pt, plane3Dᵦ; tol = 1e-10)
    end
end


ℋ = matrices(HomographyMatrices(camera₁, camera₂, get_planes(world)))

# Verify that the homographies are compatible with the projected points
for (i, group) in enumerate(groups)
    allotment = get_interval(group)
    for corresponding_pair in zip(ℳ[allotment], ℳ′[allotment])
        𝐇ᵢ = ℋ[i]
        𝐦 = corresponding_pair[1]
        𝐦′  = hom⁻¹(𝐇ᵢ * hom(𝐦))
        @test isapprox(norm(𝐦′ - corresponding_pair[2]), 0.0; atol = 1e-10)
    end
end


extract_pose = PoseFromSingleHomographyContext(intrinsics = get_intrinsics(get_model(cameraₐ)), image_type = analogue_image₁, algorithm = MalisVargasDecomposition())
for (i, group) in enumerate(groups)
    # Obtain the ground truth
    relative_pose = RelativePose(cameraₐ, cameraᵦ)
    R = ComputerVision.rotation(relative_pose)
    t = ComputerVision.translation(relative_pose)
    planes = get_planes(world₂)
    plane = planes[i]
    𝐧 = get_normal(plane)
    d = get_distance(plane)
    # Note the convention used by Malis and Vargas in their homography decomposition paper.
    𝐑 = R'
    𝐭 = (-R'*t) / d

    # Extract the corresponding points associated with the current plane
    allotment = get_interval(groups[i])
    𝒞 = Correspondences((ℳₐ[allotment],ℳᵦ′[allotment]))

    # Construct two potential solutions
    potential_poses = extract_pose(HomographyMatrix(cameraₐ, cameraᵦ, plane), 𝒞)
    # One of the poses should correspond with the truth (taking into account numerical errors)
    A₁, b₁, v₁ =  potential_poses[1]
    A₂, b₂, v₂ =  potential_poses[2]
    test₁ = norm(A₁ - 𝐑) < 1e-7 && norm(b₁ - 𝐭) < 1e-7 && norm(v₁ - 𝐧) < 1e-7
    test₂ = norm(A₂ - 𝐑) < 1e-7 && norm(b₂ - 𝐭) < 1e-7 && norm(v₂ - 𝐧) < 1e-7
    @test test₁ || test₂
end
