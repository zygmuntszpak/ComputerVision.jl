struct AquireImageContext <: AbstractContext end
abstract type TwoViewContext <: AbstractContext end
struct TwoViewExperimentContext <: TwoViewContext end



# function (aquire::AquireImageContext)(world::PrimitiveWorld, camera::AbstractCamera)
#     model = get_model(camera)
#     image_type = get_image_type(camera)
#     image_coordinate_system = get_coordinate_system(image_type)
#     world_coordinate_system = get_coordinate_system(world)
#     points = get_points(world)
#     image_points = project(model, points, world_coordinate_system, image_coordinate_system)
#     return image_points
# end

function (aquire::AquireImageContext)(world::PrimitiveWorld, camera::AbstractCamera)
    world_coordinate_system = get_coordinate_system(world)
    points = get_points(world)
    image_points = project(Projection(camera, world_coordinate_system), points)
    return image_points
end



# function project(model::Pinhole, 𝒳::Vector{<: AbstractVector}, world_coordinate_system::AbstractCoordinateSystem, image_coordinate_system::AbstractPlanarCoordinateSystem)
#     𝐏 = to_matrix(Projection(model, world_coordinate_system, image_coordinate_system))
#     ℳ = map(𝒳) do 𝐗
#         𝐦 = hom⁻¹(𝐏 * hom(𝐗))
#     end
#     return ℳ
# end

# function project(e::Pinhole, 𝐏::AbstractArray, 𝒳::Vector{<:AbstractArray})
#
#     if size(𝐏) != (3,4)
#         throw(ArgumentError("Expect 3 x 4 projection matrix."))
#     end
#     ℳ = map(𝒳) do 𝐗
#         𝐦 = hom⁻¹(𝐏 * hom(𝐗))
#     end
# end
