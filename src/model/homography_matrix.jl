struct HomographyMatrix{T₁ <: AbstractMatrix} <: ProjectiveEntity
    𝐇::T₁
end

struct HomographyMatrices{T₁ <: AbstractVector{<: HomographyMatrix}} <: ProjectiveEntity
    ℋ::T₁
end

function matrix(entity::HomographyMatrix)
    entity.𝐇
end

function matrices(entity::HomographyMatrices)
    map(x->matrix(x), entity.ℋ)
end


HomographyMatrices(camera₁::AbstractCamera, camera₂::AbstractCamera, planes::AbstractVector{<:Plane}) = HomographyMatrices(construct_homography_matrices(camera₁, camera₂, planes,  CartesianSystem(Point(0.0, 0.0, 0.0), Vec(1.0, 0.0, 0.0), Vec(0.0, 1.0, 0.0), Vec(0.0, 0.0, 1.0))))

#HomographyMatrix(camera₁::AbstractCamera, camera₂::AbstractCamera, plane::Plane) = HomographyMatrix(camera₁, camera₂, plane, CartesianSystem(Point(0.0, 0.0, 0.0), Vec(1.0, 0.0, 0.0), Vec(0.0, 1.0, 0.0), Vec(0.0, 0.0, 1.0)))
HomographyMatrix(camera₁::AbstractCamera, camera₂::AbstractCamera, plane::Plane) = HomographyMatrix(construct_homography_matrix(camera₁, camera₂, plane, CartesianSystem(Point(0.0, 0.0, 0.0), Vec(1.0, 0.0, 0.0), Vec(0.0, 1.0, 0.0), Vec(0.0, 0.0, 1.0))))


# function construct_homography_matrices(camera₁::AbstractCamera, camera₂::AbstractCamera, planes::AbstractVector{<:Plane}, reference_system::AbstractCoordinateSystem)
#     𝐀, b, 𝐀₁, 𝐛₁  = establish_inherent_homography_variables(camera₁, camera₂, reference_system)
#     ℋ = [HomographyMatrix(construct_homography_matrix(𝐀, b, 𝐀₁, b₁, plane[i])) for i = 1:length(planes)]
#     return ℋ
# end
#
# function partition_projection_matrix(camera, reference_system)
#     P = Projection(camera, reference_system)
#     𝐏 = to_matrix(P)
#     𝐀 = 𝐏[1:3,1:3]
#     𝐛 = 𝐏[1:3, 4]
#     if rank(𝐀) != 3
#         error("The 3x3 submatrix of the projection matrix is not invertible.")
#     end
#     return 𝐀, 𝐛
# end
#
# function establish_inherent_homography_variables(camera₁::AbstractCamera, camera₂::AbstractCamera, reference_system::AbstractCoordinateSystem)
#     𝐀₁, 𝐛₁ = partition_projection_matrix(camera₁, reference_system)
#     𝐀₂, 𝐛₂ = partition_projection_matrix(camera₂, reference_system)
#     𝐀 = 𝐀₂/𝐀₁
#     𝐛 = 𝐛₂ - (𝐀₂/𝐀₁)*𝐛₁
#     return 𝐀, 𝐛, 𝐀₁, 𝐛₁
# end
#
# function construct_homography_matrix(camera₁::AbstractCamera, camera₂::AbstractCamera, plane::Plane, reference_system::AbstractCoordinateSystem)
#     𝐀, 𝐛, 𝐀₁, 𝐛₁  = establish_inherent_homography_variables(camera₁, camera₂, reference_system)
#     𝐇 = construct_homography_matrix(𝐀, 𝐛, 𝐀₁, 𝐛₁, plane)
#     return 𝐇
# end
#
# function construct_homography_matrix(𝐀::AbstractArray, 𝐛::AbstractArray, 𝐀₁::AbstractArray, 𝐛₁::AbstractArray, plane::Plane)
#     # We assume that the plane is given by the vector 𝛑 =[n -d], where n is the outward
#     # normal to the plane and d is the distance from the plane to the origin of the
#     # coordinate system.
#     𝐧 = get_normal(plane)
#     d = get_distance(plane)
#     w = d + 𝐧'*inv(𝐀₁)*𝐛₁
#     𝐯 = inv(𝐀₁')*𝐧
#     𝐇 =  w*𝐀 + 𝐛*𝐯'
#     return 𝐇
# end

function construct_homography_matrices(camera₁::AbstractCamera, camera₂::AbstractCamera, planes::AbstractVector{<:Plane}, reference_system::AbstractCoordinateSystem)
    ℋ = [HomographyMatrix(construct_homography_matrix(camera₁, camera₂, planes[i], reference_system)) for i = 1:length(planes)]
    return ℋ
end

function construct_homography_matrix(camera₁::AbstractCamera, camera₂::AbstractCamera, plane::Plane, reference_system::AbstractCoordinateSystem)
    model₁ = get_model(camera₁)
    model₂ = get_model(camera₂)
    image_type₁ = get_image_type(camera₁)
    image_system₁ = get_coordinate_system(image_type₁)
    image_type₂ = get_image_type(camera₂)
    image_system₂ = get_coordinate_system(image_type₂)
    construct_homography_matrix(model₁, model₂, plane, reference_system, image_system₁, image_system₂)
end

function construct_homography_matrix(model₁::AbstractCameraModel, model₂::AbstractCameraModel,  plane::Plane,  reference_system::AbstractCoordinateSystem, image_system₁::AbstractPlanarCoordinateSystem, image_system₂::AbstractPlanarCoordinateSystem)
    intrinsics₁ = get_intrinsics(model₁)
    𝐊₁ = to_matrix(intrinsics₁, image_system₁)
    extrinsics₁ = get_extrinsics(model₁)
    𝐑₁′, 𝐭₁′ = ascertain_pose(extrinsics₁, reference_system)
    𝐑₁ = transpose(𝐑₁′)
    𝐭₁ = 𝐭₁′
    # Our projection matrix should decompose as [𝐑 -𝐑*𝐭]

    intrinsics₂ = get_intrinsics(model₂)
    𝐊₂ = to_matrix(intrinsics₂, image_system₂)
    extrinsics₂ = get_extrinsics(model₂)
    𝐑₂′, 𝐭₂′ = ascertain_pose(extrinsics₂, reference_system)
    # Our projection matrix should decompose as [𝐑 -𝐑*𝐭]
    𝐑₂ = transpose(𝐑₂′)
    𝐭₂ = 𝐭₂′

    # We assume that the plane is given by the vector 𝛑 =[n -d], where n is the outward
    # normal to the plane and d is the distance from the plane to the origin of the
    # coordinate system.
    𝐧 = get_normal(plane)
    d = get_distance(plane)

    𝐀 = 𝐊₂*𝐑₂/𝐑₁/𝐊₁
    𝐛 = 𝐊₂*𝐑₂*(𝐭₁ - 𝐭₂)
    w = d - 𝐧'*𝐭₁
    𝐯 = inv(𝐊₁')*𝐑₁*𝐧

    𝐇 =  w*𝐀 + 𝐛*𝐯'
end
