abstract type AbstractSyntheticScene end
abstract type AbstractSceneParameters end


Base.@kwdef struct SyntheticScene{T₁ <: AbstractSceneParameters, T₂ <: AbstractWorld, T₃ <: AbstractCamera} <: AbstractSyntheticScene
    scene_parameters::T₁
    world::T₂
    cameras::Vector{T₃}
end

function get_scene_parameters(scene::SyntheticScene)
    scene.scene_parameters
end

function get_world(scene::SyntheticScene)
    scene.world
end

function get_cameras(scene::SyntheticScene)
    scene.cameras
end

# function SyntheticScene{T₁,  T₂ , T₃}(scene_parameters::T₁) where {T₁ <: AbstractSceneParameters, T₂ <: AbstractWorld, T₃ <: AbstractCamera}
#         image_width = 640
#         image_height = 480
#         f = 400
#
#         pinhole₁ = Pinhole(intrinsics = IntrinsicParameters(width = image_width, height = image_height, focal_length = f))
#         analogue_image₁ = AnalogueImage(coordinate_system = OpticalSystem())
#         camera₁ = ComputerVision.Camera(image_type = analogue_image₁, model = pinhole₁)
#         𝐑₁ = SMatrix{3,3,Float64,9}(rotxyz(0*(pi/180), 0*(pi/180), 0*(pi/180)))
#         𝐭₁ = [-100.0, -200.0, 0.0]
#         relocate!(camera₁, 𝐑₁, 𝐭₁)
#
#         pinhole₂ = Pinhole(intrinsics = IntrinsicParameters(width = image_width, height = image_height, focal_length = f))
#         analogue_image₂ = AnalogueImage(coordinate_system = OpticalSystem())
#         camera₂ = ComputerVision.Camera(image_type = analogue_image₂, model = pinhole₂)
#         v₁ = rand(-10:10)
#         v₂ = rand(-10:10)
#         v₃ = rand(-10:10)
#         𝐑₂ = SMatrix{3,3,Float64,9}(rotxyz(v₁ * (pi/180), v₂ * (pi/180), v₃ * (pi/180)))
#         𝐭₂ = [-200.0, -200.0, -700.0]
#         relocate!(camera₂, 𝐑₂, 𝐭₂)
#
#         world = construct_synthetic_scene(scene_parameters, camera₁, camera₂)
#         new(synthetic_scene, world, [camera₁ camera₂])
# end

SyntheticScene(scene_parameters::AbstractSceneParameters)  = SyntheticScene(construct_scene(scene_parameters)...)


function construct_scene(scene_parameters::AbstractSceneParameters)
        image_width = 640
        image_height = 480
        f = 400

        pinhole₁ = Pinhole(intrinsics = IntrinsicParameters(width = image_width, height = image_height, focal_length = f))
        analogue_image₁ = AnalogueImage(coordinate_system = OpticalSystem())
        camera₁ = ComputerVision.Camera(image_type = analogue_image₁, model = pinhole₁)
        𝐑₁ = SMatrix{3,3,Float64,9}(rotxyz(0*(pi/180), 0*(pi/180), 0*(pi/180)))
        𝐭₁ = [-100.0, -200.0, 0.0]
        relocate!(camera₁, 𝐑₁, 𝐭₁)

        pinhole₂ = Pinhole(intrinsics = IntrinsicParameters(width = image_width, height = image_height, focal_length = f))
        analogue_image₂ = AnalogueImage(coordinate_system = OpticalSystem())
        camera₂ = ComputerVision.Camera(image_type = analogue_image₂, model = pinhole₂)
        v₁ = rand(-10:10)
        v₂ = rand(-10:10)
        v₃ = rand(-10:10)
        𝐑₂ = SMatrix{3,3,Float64,9}(rotxyz(v₁ * (pi/180), v₂ * (pi/180), v₃ * (pi/180)))
        𝐭₂ = [-200.0, -200.0, -700.0]
        relocate!(camera₂, 𝐑₂, 𝐭₂)

        world = construct_synthetic_scene(scene_parameters, camera₁, camera₂)
        return scene_parameters, world, [camera₁, camera₂]
end


Base.@kwdef struct PlanarSceneParameters{T₁ <: HyperRectangle, T₂ <: Number} <: AbstractSceneParameters
    total_planes::Int = 2
    regions_of_interest::Vector{T₁} = [HyperRectangle(Vec(0.0, 0.0), Vec(50.0, 50.0)), HyperRectangle(Vec(0.0, 0.0), Vec(50.0, 50.0))]
    points_per_region::Vector{T₂} =  [12, 12]
end


function construct_synthetic_scene(scene_type::PlanarSceneParameters, camera₁::AbstractCamera, camera₂::AbstractCamera)
    boxes = scene_type.regions_of_interest
    points_per_region = scene_type.points_per_region
    K =  scene_type.total_planes
    # Here we assume that the instrinsic image width and height for both cameras is the same.
    image_width = get_width(get_intrinsics(get_model(camera₁)))
    image_height = get_width(get_intrinsics(get_model(camera₁)))
    𝐜₁ = get_origin(get_extrinsics(get_model(camera₁)))
    𝐜₂ = get_origin(get_extrinsics(get_model(camera₂)))
    planes = [generate_random_plane(𝐜₁, 𝐜₂) for k = 1:K]
    D = [sample_points_on_random_plane(points_per_region[k], planes[k], image_width, image_height, boxes[k], camera₁, camera₂) for k = 1:K]
    # Determine which set of points correspond to which planar structure
    cummulative = OffsetArray(cumsum(vcat([0],points_per_region)), -1)
    span = [ (cummulative[k-1]+1):cummulative[k] for k = 1:K ]
    intervals = [IntervalAllotment(span[k]) for k = 1:K]
    𝒳  = collect(Iterators.flatten(D))
    PlanarWorld(; points = 𝒳, groups =  intervals,  planes = planes)
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

    ℳ = [ Point(i + rand(0.0:box_dimensions[1]),j + rand(0.0:box_dimensions[2])) for n = 1:N]
    ℒ = back_project(camera₁, ℳ)
    determine_intersection = IntersectionContext()
    𝒳₁ = [determine_intersection(plane, ℒ[n]) for n = 1:N]
end
