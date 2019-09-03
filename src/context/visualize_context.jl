Base.@kwdef mutable struct VisualizeWorldContext{T₀ <: Union{AbstractPlotting.Scene, PGFPlotsX.Axis}, T₁ <: AbstractVisualProperties} <: AbstractContext
    scene::T₀ = Scene()
    visual_properties::T₁ = MakieVisualProperties()
end

function get_scene(context::VisualizeWorldContext)
    context.scene
end

function set_scene!(context::VisualizeWorldContext, scene::AbstractPlotting.Scene)
    context.scene = scene
end

function get_visual_properties(context::VisualizeWorldContext)
    context.visual_properties
end

function set_visual_properties!(context::VisualizeWorldContext, visual_properties::AbstractVisualProperties)
    context.visual_properties = visual_properties
end

# function get_scale(context::VisualizeWorldContext)
#     context.scale
# end
#
# function set_scale!(context::VisualizeWorldContext, scale::Real)
#     context.scale = scale
# end
#
# function get_linewidth(context::VisualizeWorldContext)
#     context.linewidth
# end
#
# function set_linewidth!(context::VisualizeWorldContext, linewidth::Number)
#     context.linewidth = linewidth
# end
#
# function get_markersize(context::VisualizeWorldContext)
#     context.markersize
# end
#
# function set_markersize!(context::VisualizeWorldContext, markersize::Number)
#     context.markersize = markersize
# end

function (visualize::VisualizeWorldContext)(world::AbstractWorld, camera::AbstractCamera)
    visualize(world)
    visualize(camera)
end

function (visualize::VisualizeWorldContext)(world::AbstractWorld, cameras::Vector{<:AbstractCamera})
    visualize(world)
    visualize(cameras)
end

function (visualize::VisualizeWorldContext)(cameras::Vector{<:AbstractCamera})
    for camera in cameras
        visualize(camera)
    end
end

function (visualize::VisualizeWorldContext)(camera::AbstractCamera)
    scene = get_scene(visualize)
    model = get_model(camera)
    extrinsics = get_extrinsics(model)
    intrinsics = get_intrinsics(model)
    visual_properties = get_visual_properties(visualize)
    scale = get_scale(visual_properties)
    linewidth = get_linewidth(visual_properties)
    draw_camera!(scene, scale, linewidth, intrinsics, extrinsics)
end


function (visualize::VisualizeWorldContext)(world::AbstractWorld)
    scene = get_scene(visualize)
    visual_properties = get_visual_properties(visualize)
    scale = get_scale(visual_properties)
    linewidth = get_linewidth(visual_properties)
    markersize = get_markersize(visual_properties)
    draw_world!(scene, world, scale, markersize)
end

function (visualize::VisualizeWorldContext)(points::AbstractVector{<: AbstractVector}, groups::Vector{<:IntervalAllotment})
    scene = get_scene(visualize)
    visual_properties = get_visual_properties(visualize)
    scale = get_scale(visual_properties)
    linewidth = get_linewidth(visual_properties)
    markersize = get_markersize(visual_properties)
    draw_point_groups!(scene, markersize, points, groups)
end

function (visualize::VisualizeWorldContext)(points::AbstractVector{<: AbstractVector})
    scene = get_scene(visualize)
    visual_properties = get_visual_properties(visualize)
    scale = get_scale(visual_properties)
    linewidth = get_linewidth(visual_properties)
    markersize = get_markersize(visual_properties)
    draw_points!(scene, markersize, points)
end

# groups::Vector{<:IntervalAllotment}
#TODO remove unnecessary parameters
function draw_world!(scene::Union{AbstractPlotting.Scene, PGFPlotsX.Axis}, world::AbstractWorld, scale::Real, markersize::Number)
    world_coordinate_system = get_coordinate_system(world)
    points = get_points(world)
    groups = get_groups(world)
    planes = get_planes(world)
    draw_coordinate_system!(scene, scale)
    draw_point_groups!(scene, markersize, points, groups)
    #draw_planes!(scene, planes)
    # We want to draw a minimum enclosing border around the points that lie on a plane
    draw_planes!(scene, planes, points)
end


function draw_planes!(scene::Union{AbstractPlotting.Scene, PGFPlotsX.Axis}, planes::Vector{<: EuclideanPlane3D}, points::Vector{<: AbstractVector})
    for plane in planes
        draw_plane_segment!(scene, plane, points)
    end
end

function draw_planes!(scene::Union{AbstractPlotting.Scene, PGFPlotsX.Axis}, planes::Vector{<: Plane})

end

function draw_planes!(scene::Union{AbstractPlotting.Scene, PGFPlotsX.Axis}, planes::Vector{<: PlaneSegment})
    #@show "plane segment!"
    for plane in planes
        draw_plane_segment!(scene, plane)
    end
end

function construct_segments(plane::EuclideanPlane3D, points::Vector{<: AbstractVector})
    # Determine which points fall on this particular plane
    𝐧 = normal(plane)
    d = distance(plane)
    valid = [(norm(dot(𝐧,points[k]) - d) < 1e-8 ? true : false) for k = 1:length(points) ]
    #display(valid)
    valid_points = points[valid]

    # Determine coordinates of planar points with respect to the planes coordinate system
    coordinate_system = get_coordinate_system(plane)
    𝐨 = origin(coordinate_system)
    𝐞₁ = get_e₁(coordinate_system)
    𝐞₂ = get_e₂(coordinate_system)
    𝐞₃ = get_e₃(coordinate_system)


    # v₁ = dot(𝐞₁, valid_points[1] - 𝐨)
    # v₂ = dot(𝐞₂, valid_points[1] - 𝐨)
    # @show 𝐞₁, 𝐞₂
    # @show v₁, v₂
    # @show valid_points[1]
    # @show  v₁*𝐞₁ + v₂*𝐞₂
    # @show 𝐨 + v₁*𝐞₁ + v₂*𝐞₂


    # Determine the bounds that encompass the points
    w = [dot(𝐞₁, valid_points[k] - 𝐨) for k = 1:length(valid_points)]
    h = [dot(𝐞₂, valid_points[k] - 𝐨) for k = 1:length(valid_points)]
    #zs = [dot(𝐞₃, valid_points[k]) for k = 1:length(valid_points)]
    w₀ = minimum(w)
    wμ = mean(w)
    w₁ = maximum(w)
    h₀ = minimum(h)
    hμ = mean(h)
    h₁ = maximum(h)

    depth = 100
    #@show w
    #@show h


    p = 𝐨 + w₀*𝐞₁ + h₀*𝐞₂
    q = 𝐨 +  w₁*𝐞₁ + h₀*𝐞₂
    r =  𝐨 + w₁*𝐞₁ + h₁*𝐞₂
    s =  𝐨 + w₀*𝐞₁ + h₁*𝐞₂

    t =  𝐨 + wμ*𝐞₁ + hμ*𝐞₂

    # w = 1000
    # h = 800
    # d = 100
    # p = 𝐨 + -w*𝐞₁ + h*𝐞₂
    # q =  𝐨 + w*𝐞₁ + h*𝐞₂
    # r = 𝐨 + w*𝐞₁ + -h*𝐞₂
    # s =  𝐨 + -w*𝐞₁ + -h*𝐞₂
    #segment = [p => q , q => r , r => s , s => p, 𝐨 => 𝐨 + depth*𝐞₃]
    segment = [p => q , q => r , r => s , s => p, t => t + depth*𝐞₃]
end

function draw_plane_segment!(scene::AbstractPlotting.Scene, plane::EuclideanPlane3D, points::Vector{<: AbstractVector})
    segment = construct_segments(plane, points)
    @show segment
    #Makie.linesegments!(scene, segment, color = :black)
    #linesegments!(scene, segment, [colorant"black" for i = 1:length(segment)])
    segment₂ = [Point3f0(first(segment[i])) => Point3f0(last(segment[i])) for i = 1:length(segment)]
    Makie.linesegments!(scene, segment₂, color = :black, linewidth = 2)
end

function draw_plane_segment!(scene::AbstractPlotting.Scene, plane::PlaneSegment)
    segment = get_segment(plane)
    @show segment
    #Makie.linesegments!(scene, segment, color = :black)
    #linesegments!(scene, segment, [colorant"black" for i = 1:length(segment)])
    segment₂ = [Point3f0(first(segment[i])) => Point3f0(last(segment[i])) for i = 1:length(segment)]
    Makie.linesegments!(scene, segment₂, color = :black, linewidth = 2)
end

function draw_plane_segment!(scene::PGFPlotsX.Axis, plane::EuclideanPlane3D, points::Vector{<: AbstractVector})
    #segment = get_segment(plane)
    segment = construct_segments(plane, points)
    linesegments!(scene, segment, [colorant"black" for i = 1:length(segment)])
end

function draw_plane_segment!(scene::PGFPlotsX.Axis, plane::EuclideanPlane3D)
    segment = get_segment(plane)
    linesegments!(scene, segment, [colorant"black" for i = 1:length(segment)])
end

# function draw_world!(scene::PGFPlotsX.Axis, world::PrimitiveWorld, scale::Real, markersize::Number)
#     world_coordinate_system = get_coordinate_system(world)
#     points = get_points(world)
#     draw_coordinate_system!(scene, scale)
#     draw_points!(scene, markersize, points)
# end

function draw_coordinate_system!(scene::AbstractPlotting.Scene, scale::Real)
    x = Vec3f0(0); baselen = 0.2f0 * scale ; dirlen = 1f0 * scale
    # create an array of differently colored boxes in the direction of the 3 axes
    rectangles = [
        (HyperRectangle(Vec3f0(x), Vec3f0(dirlen, baselen, baselen)), RGBAf0(1,0,0,1)),
        (HyperRectangle(Vec3f0(x), Vec3f0(baselen, dirlen, baselen)), RGBAf0(0,1,0,1)),
        (HyperRectangle(Vec3f0(x), Vec3f0(baselen, baselen, dirlen)), RGBAf0(0,0,1,1))
    ]
    meshes = map(GLNormalMesh, rectangles)
    mesh!(scene, merge(meshes))
end

function draw_coordinate_system!(scene::PGFPlotsX.Axis, scale::Real)
    linewidth = 1
    linesegment!(scene, [0, 0, 0] => [scale*3, 0, 0], colorant"red", linewidth = linewidth)
    linesegment!(scene, [0, 0, 0] => [0, scale*3, 0], colorant"green", linewidth = linewidth)
    linesegment!(scene, [0, 0, 0] => [0, 0, scale*3], colorant"blue", linewidth = linewidth)

end

function linesegments!(scene::PGFPlotsX.Axis, segments::AbstractVector, segment_colors::AbstractVector; linewidth = 1)
    for (i, pair) in enumerate(segments)
        linesegment!(scene, pair, segment_colors[i], linewidth = linewidth)
    end
end

function linesegment!(scene::PGFPlotsX.Axis, pair::Pair, c::Color; linewidth = 1)
    segment = @pgf Plot3Inc(
                {
                    "solid",
                    mark = "none",
                    color => c,
                    line_width = string(linewidth)*"pt"
                },
                 Coordinates([tuple(pair.first...),  tuple(pair.second...)])
            )
    push!(scene, segment)
end

function draw_points!(scene::AbstractPlotting.Scene, markersize::Real, points::Vector{<: AbstractVector})
    scatter!(points, markersize = markersize)
end

function draw_point_groups!(scene::Union{AbstractPlotting.Scene, PGFPlotsX.Axis}, markersize::Real, points::Vector{<: AbstractVector}, groups::Vector{<:IntervalAllotment})
    group_colors = distinguishable_colors(length(groups)+1)
    @show group_colors
    for (i, allotment) in enumerate(groups)
        group = get_interval(allotment)
        draw_points!(scene, markersize, points[group], group_colors[i])
    end
end

function draw_points!(scene::Union{AbstractPlotting.Scene, PGFPlotsX.Axis}, markersize::Real, points::Vector{<: AbstractVector}, col = colorant"red")
    d = length(first(points))
    d == 2 ? draw_2D_points!(scene, markersize, points, col) : draw_3D_points!(scene, markersize, points, col)
end

function draw_2D_points!(scene::PGFPlotsX.Axis, markersize::Real, points::Vector{<: AbstractVector}, col = colorant"red")
    N = length(points)
    M = reshape(reinterpret(Float64, points), 2, N)
    x = M[1,:]
    y = M[2,:]
    p = @pgf PlotInc(
                {
                    "only marks",
                    mark_size = "1pt",
                    mark = "*",
                },
                Table(x, y)
            )
    push!(scene, p)
end

function draw_2D_points!(scene::AbstractPlotting.Scene, markersize::Real, points::Vector{<: AbstractVector}, col = colorant"red")
    scatter!(points, markersize = markersize, color = col)
end

function draw_3D_points!(scene::AbstractPlotting.Scene, markersize::Real, points::Vector{<: AbstractVector}, col = colorant"red")
    scatter!(points, markersize = markersize, color = col)
end

function draw_3D_points!(scene::PGFPlotsX.Axis, markersize::Real, points::Vector{<: AbstractVector}, col = colorant"red")
    N = length(points)
    M = reshape(reinterpret(Float64, points), 3, N)
    x = M[1,:]
    y = M[2,:]
    z = M[3,:]
    # color => "red",
    p = @pgf Plot3Inc(
                {
                    "only marks",
                    mark_size = "1pt",
                    mark = "*",
                },
                Table(x, y, z)
            )
    push!(scene, p)
end

function draw_camera!(scene::Union{AbstractPlotting.Scene, PGFPlotsX.Axis}, scale::Real, linewidth::Number, intrinsics::IntrinsicParameters, extrinsics::ExtrinsicParameters)

    image_width = get_width(intrinsics)
    image_height = get_height(intrinsics)
    f = get_focal_length(intrinsics)
    camera_system = get_coordinate_system(extrinsics)
    optical_center = Point3f0(get_origin(camera_system))
    𝐞₁ = get_e₁(camera_system)
    𝐞₂ = get_e₂(camera_system)
    𝐞₃ = get_e₃(camera_system)
    bottom_right = optical_center + Point3f0((image_width/2)     * 𝐞₁ + (image_height/2)  * 𝐞₂ + f*𝐞₃)
    top_right =  optical_center   + Point3f0((image_width/2)     * 𝐞₁ + (-image_height/2) * 𝐞₂ + f*𝐞₃)
    top_left = optical_center     + Point3f0((-image_width/2)    * 𝐞₁ + (-image_height/2) * 𝐞₂ + f*𝐞₃)
    bottom_left = optical_center  + Point3f0((-image_width/2)    * 𝐞₁ + (image_height/2)  * 𝐞₂ + f*𝐞₃)
    centroid2film = [
            optical_center  => bottom_right;
            optical_center  => top_right;
            optical_center  => top_left;
            optical_center  => bottom_left;
    ]

    film = [
            bottom_right => top_right;
            top_right => top_left;
            top_left => bottom_left;
            bottom_left =>  bottom_right;
            ]


    coordinate_system = [
        optical_center => optical_center + Point3f0(scale*𝐞₁);
        optical_center => optical_center + Point3f0(scale*𝐞₂);
        optical_center => optical_center + Point3f0(scale*𝐞₃);
    ]
    draw_camera_components!(scene, centroid2film, film, coordinate_system, linewidth)
end

function draw_camera_components!(scene::AbstractPlotting.Scene, centroid2film, film, coordinate_system, linewidth)
    Makie.linesegments!(scene, centroid2film, color = :black, linewidth = linewidth)
    Makie.linesegments!(scene, film, color = :black, linewidth = linewidth)
    Makie.linesegments!(scene, coordinate_system, color = [:red, :green, :blue ], linewidth = linewidth)
end

function draw_camera_components!(scene::PGFPlotsX.Axis, centroid2film, film, coordinate_system, linewidth)
    linesegments!(scene, centroid2film, [colorant"black" for i = 1:length(centroid2film)])
    linesegments!(scene, film, [colorant"black" for i = 1:length(film)])
    linesegments!(scene, coordinate_system, [colorant"red", colorant"green", colorant"blue"])
    # linesegments!(scene, centroid2film, color = :black, linewidth = linewidth)
    # linesegments!(scene, film, color = :black, linewidth = linewidth)
    # linesegments!(scene, coordinate_system, color = [:red, :green, :blue ], linewidth = linewidth)
end
