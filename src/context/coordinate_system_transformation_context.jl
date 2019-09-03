abstract type AbstractCoordinateTransformationContext <: AbstractContext end


struct WorldCoordinateTransformationContext{T <: CoordinateTransformation} <: AbstractCoordinateTransformationContext
     coordinate_transformation::T
end

function (context::WorldCoordinateTransformationContext)(camera::AbstractCamera)
    𝐑 = rotation(context.coordinate_transformation)
    𝐭 = translation(context.coordinate_transformation)
    model = get_model(camera)
    extrinsics = get_extrinsics(model)
    coordinate_system = get_coordinate_system(extrinsics)
    𝐞₁ = get_e₁(coordinate_system)
    𝐞₂ = get_e₂(coordinate_system)
    𝐞₃ = get_e₃(coordinate_system)
    𝐨 = get_origin(coordinate_system)
    𝐞₁′ = 𝐑' * 𝐞₁
    𝐞₂′ = 𝐑' * 𝐞₂
    𝐞₃′ = 𝐑' * 𝐞₃
    𝐨′ =  𝐑' * (𝐨 - 𝐭)
    set_coordinate_system!(extrinsics,CartesianSystem(𝐨′, 𝐞₁′, 𝐞₂′, 𝐞₃′))
end

function (context::WorldCoordinateTransformationContext)(world::AbstractWorld)
    𝐑 = rotation(context.coordinate_transformation)
    𝐭 = translation(context.coordinate_transformation)

    points = get_points(world)
    planes = get_planes(world)

    points′ = transform_3D_points(𝐑, 𝐭, points)
    planes′ = transform_planes(𝐑, 𝐭, planes)
    # TODO transform the planes
    set_points!(world, points′)
    set_planes!(world, planes′)
    return nothing
end

function transform_3D_points(𝐑::AbstractMatrix, 𝐭::AbstractVector, points::AbstractVector)
    map(points) do 𝐗
        𝐑' * (𝐗 - 𝐭)
    end
end

function transform_planes(𝐑::AbstractMatrix, 𝐭::AbstractVector, planes::Vector{<: Union{Plane, PlaneSegment}})
    [transform_plane(𝐑, 𝐭, planes[k]) for k = 1:length(planes)]
end

function transform_plane(𝐑::AbstractMatrix, 𝐭::AbstractVector, plane::T) where T <: Union{Plane, PlaneSegment}
    𝐧 = get_normal(plane)
    d = get_distance(plane)
    𝐚 = construct_point_on_plane(𝐧, d)

    𝐧′ = 𝐑' * 𝐧
    𝐚′ = 𝐑' * (𝐚 - 𝐭)
    d′ = dot(𝐚′, 𝐧′)

    # Ensure that our plane representation always follows the "outward normal" convention
    if d′ < 0
        T(Vec3(-𝐧′...), -d′)
    else
        T(Vec3(𝐧′...), d′)
    end
end

function transform_planes(𝐑::AbstractMatrix, 𝐭::AbstractVector, planes::Vector{<: EuclideanPlane3D})
    [transform_plane(𝐑, 𝐭, planes[k]) for k = 1:length(planes)]
end

function transform_plane(𝐑::AbstractMatrix, 𝐭::AbstractVector, plane::T) where T <: Union{EuclideanPlane3D}
    coordinate_system = get_coordinate_system(plane)

    𝐞₁ = get_e₁(coordinate_system)
    𝐞₂ = get_e₂(coordinate_system)
    𝐞₃ = get_e₃(coordinate_system)
    𝐨 = get_origin(coordinate_system)
    𝐞₁′ = 𝐑' * 𝐞₁
    𝐞₂′ = 𝐑' * 𝐞₂
    𝐞₃′ = 𝐑' * 𝐞₃
    𝐨′ =  𝐑' * (𝐨 - 𝐭)
    plane′ = EuclideanPlane3D(CartesianSystem(𝐨′, 𝐞₁′, 𝐞₂′, 𝐞₃′))
    d′ = distance(plane′)

    if d′ < 0
        return EuclideanPlane3D(CartesianSystem(-𝐨′, 𝐞₁′, 𝐞₂′, -𝐞₃′))
    else
        plane′
    end
end

function construct_point_on_plane(𝐧::AbstractVector, d::Number)
    if 𝐧[1] != 0
        a = d / 𝐧[1]
        𝐚 = [a, 0.0, 0.0]
        return 𝐚
    elseif 𝐧[2] != 0
        a = d / 𝐧[2]
        𝐚 = [0.0, a, 0.0]
        return 𝐚
    else
        a = d / 𝐧[3]
        𝐚 = [0.0, 0.0, a]
        return 𝐚
    end
end
