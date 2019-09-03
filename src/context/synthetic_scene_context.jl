abstract type AbstractSyntheticScene end


struct SyntheticSceneContext{T₁ <: AbstractSyntheticScene, T₂ <: AbstractWorld, T₃ <: AbstractCamera} <: AbstractContext
    scene_type::T₁
    world::T₂
    cameras::Vector{T₃}
end

# SyntheticScene(synthetic_scene::AbstractSynthethicScene) =
# function SyntheticSceneContext(synthetic_scene::AbstractSynthethicScene)
#     image_width = 640
#     image_height = 480
#     f = 400
#
#     pinhole₁ = Pinhole(intrinsics = IntrinsicParameters(width = image_width, height = image_height, focal_length = f))
#     analogue_image₁ = AnalogueImage(coordinate_system = OpticalSystem())
#     camera₁ = ComputerVision.Camera(image_type = analogue_image₁, model = pinhole₁)
#     𝐑₁ = SMatrix{3,3,Float64,9}(rotxyz(0*(pi/180), 0*(pi/180), 0*(pi/180)))
#     𝐭₁ = [-100.0, -200.0, 0.0]
#     relocate!(camera₁, 𝐑₁, 𝐭₁)
#
#     pinhole₂ = Pinhole(intrinsics = IntrinsicParameters(width = image_width, height = image_height, focal_length = f))
#     analogue_image₂ = AnalogueImage(coordinate_system = OpticalSystem())
#     camera₂ = ComputerVision.Camera(image_type = analogue_image₂, model = pinhole₂)
#     v₁ = rand(-10:10)
#     v₂ = rand(-10:10)
#     v₃ = rand(-10:10)
#     𝐑₂ = SMatrix{3,3,Float64,9}(rotxyz(v₁ * (pi/180), v₂ * (pi/180), v₃ * (pi/180)))
#     𝐭₂ = [-200.0, -200.0, -700.0]
#     relocate!(camera₂, 𝐑₂, 𝐭₂)
#     new(synthetic_scene, camera₁, camera₂)
# end




Base.@kwdef struct PlanarSyntheticScene{T₁ <: HyperRectangle, T₂ <: Number} <: AbstractSyntheticScene
    total_planes::Int = 2
    regions_of_interest::Vector{T₁} = [HyperRectangle(Vec(0.0, 0.0), Vec(50.0, 50.0)), HyperRectangle(Vec(0.0, 0.0), Vec(50.0, 50.0))]
    points_per_region::Vector{T₂} =  [12, 12]
end

# TODO Potentially take camera parameters as input?
function (context::SyntheticSceneContext{<:PlanarSyntheticScene})()



    #planes = [generate_random_plane(𝐭₁, 𝐭₂) for k = 1:context.total_planes]
    #𝒳 = [sample_points_on_random_plane(planes[k]) for k = 1:context.total_planes]
    #
    # for k = 1:context.total_planes
    #     plane = generate_random_plane(𝐭₁, 𝐭₂)
    #     𝒳ₖ = sample_points_on_random_plane(plane)
    # end
    # plane = generate_random_plane(𝐭₁, 𝐭₂)

    boxes = context.scene_type.regions_of_interest
    points_per_region = context.scene_type.points_per_region
    K =  context.scene_type.total_planes
    planes = [generate_random_plane(𝐭₁, 𝐭₂) for k = 1:K]
    D = [sample_points_on_random_plane(points_per_region[k], planes[k], image_width, image_height, boxes[k], camera₁, camera₂) for k = 1:K]
    # Determine which set of points correspond to which planar structure
    cummulative = OffsetArray(cumsum(vcat([0],points_per_region)), -1)
    span = [ (cummulative[k-1]+1):cummulative[k] for k = 1:K ]
    intervals = [IntervalAllotment(span[k]) for k = 1:K]
    #intervals = [IntervalAllotment(1:sum(points_per_region))]
    𝒳  = collect(Iterators.flatten(D))

    # @show length(𝒳)
    # @show intervals

    # # Choose a region of interest with specified size that falls within the
    # # image dimensions.
    # box = context.scene_type.regions_of_interest[1]
    # box_dimensions = widths(box)
    # # span₁ = box_dimensions[1]:(image_width - box_dimensions[1])
    # # span₂ = box_dimensions[2]:(image_height - box_dimensions[2])
    #
    # # Keep in mind that the origin is at the center of the image.
    # half_width = image_width / 2
    # half_height = image_height / 2
    # span₁ = -half_width:(half_width - box_dimensions[1])
    # span₂ = -half_height:(half_height - box_dimensions[2])
    # i = rand(span₁)
    # j = rand(span₂)
    #
    #
    # N = context.scene_type.points_per_region[1]
    # ℳ = [ Point(i + rand(0.0:box_dimensions[1]),j + rand(0.0:box_dimensions[2])) for n = 1:N]
    # ℒ = back_project(camera₁, ℳ)
    # determine_intersection = IntersectionContext()
    # 𝒳₁ = [ determine_intersection(plane, ℒ[n]) for n = 1:N]
    # 𝒳₂ = [ℒ[n].𝐩₂ for n = 1:N]
    #
    # ℳ′ = project(Projection(camera₂), 𝒳₁)
    # ℒ′  = back_project(camera₂, ℳ′)
    # 𝒳₃ = [ℒ′[n].𝐩₂ for n = 1:N]

    #
    # ℳ = Vector{Point{2,Float64}}(undef, 4)
    # # Clockwise
    # ℳ[1] = Point(i, j)
    # ℳ[2] = Point(i + box_dimensions[1], j)
    # ℳ[3] = Point(i + box_dimensions[1], j + box_dimensions[2])
    # ℳ[4] = Point(i, j + box_dimensions[2])
    #
    # display(ℳ)
    #
    # ℒ = back_project(camera₁, ℳ)
    # display(ℒ)
    #
    # determine_intersection = IntersectionContext()
    # 𝒳₁ = Vector{Point{3,Float64}}(undef, 4)
    # 𝒳₁[1] = determine_intersection(plane, ℒ[1])
    # 𝒳₁[2] = determine_intersection(plane, ℒ[2])
    # 𝒳₁[3] = determine_intersection(plane, ℒ[3])
    # 𝒳₁[4] = determine_intersection(plane, ℒ[4])
    # #
    # 𝒳₂ = Vector{Point{3,Float64}}(undef, 4)
    # 𝒳₂[1] = ℒ[1].𝐩₂
    # 𝒳₂[2] = ℒ[2].𝐩₂
    # 𝒳₂[3] = ℒ[3].𝐩₂
    # 𝒳₂[4] = ℒ[4].𝐩₂
    #
    # @show "Second View"
    # ℳ′ = project(Projection(camera₂), 𝒳₁)
    # ℒ′  = back_project(camera₂, ℳ′)
    # display(ℳ′)
    # display(ℒ′)
    #
    # 𝒳₃ = Vector{Point{3,Float64}}(undef, 4)
    # 𝒳₃[1] = ℒ′[1].𝐩₂
    # 𝒳₃[2] = ℒ′[2].𝐩₂
    # 𝒳₃[3] = ℒ′[3].𝐩₂
    # 𝒳₃[4] = ℒ′[4].𝐩₂


    #𝒳 = vcat(𝒳₁, 𝒳₂, 𝒳₃)
    #display(𝒳)

    PlanarWorld(; points = 𝒳, groups =  intervals,  planes = planes), [camera₁, camera₂]
    #𝐊₁ = to_matrix(get_intrinsics(pinhole₁), get_coordinate_system(analogue_image₁))
    #PlanarWorld(; points = 𝒳, groups =  [IntervalAllotment(1:N*3)],  planes = [plane]), [camera₁, camera₂]
    #PlanarWorld(coordinate_system = CartesianSystem(Point(0.0, 0.0, 0.0), Vec(1.0, 0.0, 0.0), Vec(0.0, 1.0, 0.0), Vec(0.0, 0.0, 1.0)), points = 𝒳, groups = [IntervalAllotment(1:4)], planes = plane)

end

function generate_random_plane(𝐭₁::AbstractVector, 𝐭₂::AbstractVector)
    # Random plane situated in front of both cameras
    v₁ = rand(-20:20)
    v₂ = rand(-20:20)
    v₃ = rand(-20:20)
    v₄ = rand(1000:2500)
    𝐑 = SMatrix{3,3,Float64,9}(rotxyz(v₁ *(pi/180), v₂ * (pi/180), v₃ *(pi/180)))
    𝐜 = (𝐭₁ + 𝐭₂) / 2
    𝐞₁ = Vec(1.0, 0.0, 0.0)
    𝐞₂ = Vec(0.0, 1.0, 0.0)
    𝐞₃ = Vec(0.0, 0.0, 1.0)
    𝐞₁′ = 𝐑*𝐞₁
    𝐞₂′ = 𝐑*𝐞₂
    𝐞₃′ = 𝐑*𝐞₃
    𝐨 = 𝐜 + [0.0, 0.0, v₄]
    EuclideanPlane3D(CartesianSystem(𝐨, 𝐞₁′,𝐞₂′,𝐞₃′))
end

function sample_points_on_random_plane(N::Number, plane::EuclideanPlane3D, image_width, image_height, box, camera₁, camera₂)
    # Choose a region of interest with specified size that falls within the
    # image dimensions.
    box_dimensions = widths(box)

    # Keep in mind that the origin is at the center of the image.
    half_width = image_width / 2
    half_height = image_height / 2
    span₁ = -half_width:(half_width - box_dimensions[1])
    span₂ = -half_height:(half_height - box_dimensions[2])
    i = rand(span₁)
    j = rand(span₂)


    #N = context.scene_type.points_per_region[1]
    ℳ = [ Point(i + rand(0.0:box_dimensions[1]),j + rand(0.0:box_dimensions[2])) for n = 1:N]
    ℒ = back_project(camera₁, ℳ)
    determine_intersection = IntersectionContext()
    𝒳₁ = [ determine_intersection(plane, ℒ[n]) for n = 1:N]
    # 𝒳₂ = [ℒ[n].𝐩₂ for n = 1:N]
    #
    # ℳ′ = project(Projection(camera₂), 𝒳₁)
    # ℒ′  = back_project(camera₂, ℳ′)
    # 𝒳₃ = [ℒ′[n].𝐩₂ for n = 1:N]
end

# Base.@kwdef mutable struct PlanarWorld{T₁ <: AbstractCoordinateSystem, T₂ <: AbstractVector, T₃ <: AbstractPlane, T₄ <: AbstractAllotment} <: AbstractWorld
#     coordinate_system::T₁ = CartesianSystem(Point(0.0, 0.0, 0.0), Vec(1.0, 0.0, 0.0), Vec(0.0, 1.0, 0.0), Vec(0.0, 0.0, 1.0))
#     points::Vector{T₂} = [Point3(rand(-1000.0:1000.0), rand(-1000.0:1000.0), 0.0) for n = 1:5000]
#     groups::Vector{T₄} =  [IntervalAllotment(1:5000)]
#     planes::Vector{T₃} = [EuclideanPlane(CartesianSystem(Point(0.0, 0.0, 0.0), Vec(1.0, 0.0, 0.0), Vec(0.0, 1.0, 0.0), Vec(0.0, 0.0, 1.0)))]
# end


# ℳ = Vector{Point{2,Float64}}(undef, 4)
# # Clockwise
# ℳ[1] = Point(i, j)
# ℳ[2] = Point(i + box_dimensions[1], j)
# ℳ[3] = Point(i + box_dimensions[1], j + box_dimensions[2])
# ℳ[4] = Point(i, j + box_dimensions[2])
#
# display(ℳ)
#
# ℒ = back_project(camera₁, ℳ)
# display(ℒ)
#
# determine_intersection = IntersectionContext()
# 𝒳₁ = Vector{Point{3,Float64}}(undef, 4)
# 𝒳₁[1] = determine_intersection(plane, ℒ[1])
# 𝒳₁[2] = determine_intersection(plane, ℒ[2])
# 𝒳₁[3] = determine_intersection(plane, ℒ[3])
# 𝒳₁[4] = determine_intersection(plane, ℒ[4])
# #
# 𝒳₂ = Vector{Point{3,Float64}}(undef, 4)
# 𝒳₂[1] = ℒ[1].𝐩₂
# 𝒳₂[2] = ℒ[2].𝐩₂
# 𝒳₂[3] = ℒ[3].𝐩₂
# 𝒳₂[4] = ℒ[4].𝐩₂
#
# @show "Second View"
# ℳ′ = project(Projection(camera₂), 𝒳₁)
# ℒ′  = back_project(camera₂, ℳ′)
# display(ℳ′)
# display(ℒ′)
#
# 𝒳₃ = Vector{Point{3,Float64}}(undef, 4)
# 𝒳₃[1] = ℒ′[1].𝐩₂
# 𝒳₃[2] = ℒ′[2].𝐩₂
# 𝒳₃[3] = ℒ′[3].𝐩₂
# 𝒳₃[4] = ℒ′[4].𝐩₂
