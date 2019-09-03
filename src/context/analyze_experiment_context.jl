abstract type AbstractExperimentAnalysisContext <: AbstractContext end
abstract type AbstractErrorAnalysis  end
struct ReconstructionErrorAnalysis <: AbstractErrorAnalysis end

struct ReprojectionErrorAnalysis{T <: Type{HomographyMatrix}}  <: AbstractErrorAnalysis
end

struct ParameterErrorAnalysis{T <: ProjectiveEntity} <: AbstractErrorAnalysis
    projective_entity::T
end

struct ExperimentAnalysisContext{T₁ <: AbstractSyntheticScene, T₂ <: AbstractErrorAnalysis,} <: AbstractExperimentAnalysisContext
    synthetic_scene::T₁
    analysis::T₂
end


function (context::ExperimentAnalysisContext)(estimates::AbstractVector, runtimes::AbstractVector)
    synthetic_scene = context.synthetic_scene
    analyze = context.analysis

    world = get_world(synthetic_scene)
    cameras = get_cameras(synthetic_scene)
    camera₁ = cameras[1]
    camera₂ = cameras[2]

    if typeof(analyze) <: ReconstructionErrorAnalysis
        transformed_camera₁  = deepcopy(camera₁)
        transformed_camera₂  = deepcopy(camera₂)
        transformed_world = deepcopy(world)

        # Transform the coordinate system so that the first camera represents the origin of the world coordinate system.
        # This is required because the algorithm for extracting pose from a single homography matrix assumes that
        # the first camera represents origin of the world coordinate system.
        default_world_system = CartesianSystem(Point(0.0, 0.0, 0.0), Vec(1.0, 0.0, 0.0), Vec(0.0, 1.0, 0.0), Vec(0.0, 0.0, 1.0))
        alternative_world_system = get_coordinate_system(get_extrinsics(get_model(camera₁)))
        transformation_context! = WorldCoordinateTransformationContext(CoordinateTransformation(source = default_world_system, target = alternative_world_system))
        transformation_context!(transformed_camera₁)
        transformation_context!(transformed_camera₂)
        transformation_context!(transformed_world)
        R = length(estimates)
        return [analyze(transformed_camera₁, transformed_camera₂, transformed_world, estimates[r]) for r = 1:R]
    else
        R = length(estimates)
        return [analyze(camera₁, camera₂, world, estimates[r]) for r = 1:R]
    end
end



function (::ReconstructionErrorAnalysis)(camera₁::AbstractCamera, camera₂::AbstractCamera, world::AbstractWorld, estimates::AbstractVector)
    analogue_image₁ = get_image_type(camera₁)
    analogue_image₂ = get_image_type(camera₂)

    # Project 3D points onto the cameras.
    aquire = AquireImageContext()
    ℳ = aquire(world, camera₁)
    ℳ′ = aquire(world, camera₂)

    extract_pose = PoseFromSingleHomographyContext(intrinsics = get_intrinsics(get_model(camera₁)), image_type = analogue_image₁, algorithm = MalisVargasDecomposition())
    rms_errors = zeros(Float64, length(estimates))
    for (i, estimate) in enumerate(estimates)
        𝐇ᵢ = estimate
        allotment = get_interval(world.groups[i])
        𝒞 = Correspondences((ℳ[allotment],ℳ′[allotment]))



        potential_poses = extract_pose(HomographyMatrix(𝐇ᵢ), 𝒞)

        relative_pose = RelativePose(camera₁, camera₂)
        R = ComputerVision.rotation(relative_pose)
        t = ComputerVision.translation(relative_pose)
        planes = get_planes(world)
        plane = planes[i]
        𝐧 = get_normal(plane)
        d = get_distance(plane)
        # Convention used by Malis and Vargas in their homography decomposition paper.
        𝐑 = R'
        𝐭 = (-R'*t) / d

        A₁, b₁, v₁ =  potential_poses[1]
        A₂, b₂, v₂ =  potential_poses[2]

        cameraₐ = deepcopy(camera₁)
        cameraᵦ = deepcopy(camera₁)
        relocate!(cameraᵦ, A₁', -A₁'*(b₁*d))
        cameraᵧ = deepcopy(camera₁)
        relocate!(cameraᵧ, A₂', -A₂'*(b₂*d))

        triangulate_points =  TriangulateContext(DirectLinearTriangulation())
        #estimated_points = triangulate_points(camera₁, camera₂, 𝒞)
        estimated_points₁ = triangulate_points(cameraₐ, cameraᵦ, 𝒞)
        estimated_points₂ = triangulate_points(cameraₐ, cameraᵧ, 𝒞)
        world_points = get_points(world)
        reference_points = world_points[allotment]

        N = length(𝒞)
        squared_error₁ = compute_squared_error(estimated_points₁, reference_points)
        squared_error₂ = compute_squared_error(estimated_points₂, reference_points)
        μ₁ = mean(squared_error₁)
        μ₂ = mean(squared_error₂)
        rms₁ = sqrt(sum((1/(3*N)) .* squared_error₁))
        rms₂ = sqrt(sum((1/(3*N)) .* squared_error₂))
        rms = min(rms₁, rms₂)
        rms_errors[i] = rms
    end
    rms_errors
end

function compute_squared_error(estimated_points::AbstractVector, reference_points::AbstractVector)
    [norm(first(couple)-last(couple))^2 for couple in zip(estimated_points, reference_points)]
end

function (::ReprojectionErrorAnalysis{<:Type{HomographyMatrix}})(camera₁::AbstractCamera, camera₂::AbstractCamera, world::AbstractWorld, estimates::AbstractVector)
    analogue_image₁ = get_image_type(camera₁)
    analogue_image₂ = get_image_type(camera₂)

    # Project 3D points onto the cameras.
    aquire = AquireImageContext()
    ℳ = aquire(world, camera₁)
    ℳ′ = aquire(world, camera₂)
    rms_errors = zeros(Float64, length(estimates))
    for (i, estimate) in enumerate(estimates)
        𝐇ᵢ = estimate
        # Extract the corresponding points associated with the current planar structure.
        allotment = get_interval(world.groups[i])
        𝒪 = ℳ[allotment]
        𝒪′ = ℳ′[allotment]
        N = length(𝒪)

        # Construct a length-(2*N) vector consisting of N two-dimensional points in the
        # first view.
        𝛉 = Vector{Float64}(undef, N*2)
        k = 1
        for n = 1:N
            𝛉[k:k+1] = @view 𝒪[n][1:2]
            k = k + 2
        end

        index₁ = SVector(1,2)
        index₂ = SVector(3,4)
        pts = Matrix{Float64}(undef,4,N)

        for n = 1:N
             pts[index₁,n] = 𝒪[n][index₁]
             pts[index₂,n] = 𝒪′[n][index₁]
        end

        #fit = curve_fit(model_homography,  𝐇, reshape(reinterpret(Float64,vec(pts)),(4*N,)) , 𝛉; show_trace = false, maxIter = 2)
        fit = curve_fit(model_homography, jacobian_model_homography, 𝐇ᵢ, reshape(reinterpret(Float64,vec(pts)),(4*N,)) , 𝛉;  show_trace = false)
        # TODO Investigate NaN for initial values of Jacobian
        #fit = curve_fit(model_homography!, jacobian_model_homography!, 𝐇, reshape(reinterpret(Float64,vec(pts)),(4*N,)) , 𝛉;  inplace = true, show_trace = false, maxIter = 5)
        rms_errors[i] = sqrt(mean(sum(fit.resid.^2)))
    end
    rms_errors
end

function model_homography(𝐇,𝛉)
    # 2 parameters per 2D point.
    N = Int(length(𝛉)/ 2)
    index₁ = SVector(1,2)
    index₂ = SVector(3,4)
    reprojections = Matrix{Float64}(undef,4,N)
    i = 1
    for n = 1:N
        # Extract 2D point and convert to homogeneous coordinates
        𝐦 = hom(SVector{2,Float64}(𝛉[i],𝛉[i+1]))
        reprojections[index₁,n] = hom⁻¹(𝐦)
        reprojections[index₂,n] = hom⁻¹(𝐇 * 𝐦)
        i = i + 2
    end
    reshape(reinterpret(Float64,vec(reprojections)),(4*N,))
end

function model_homography!(reprojections::Array{Float64,1},𝐇,𝛉)
    # 2 parameters per 2D point.
    N = Int(length(𝛉)/ 2)
    index₁ = SVector(1,2)
    index₂ = SVector(3,4)
    reprojections_view = reshape(reinterpret(Float64,reprojections),(4,N))
    i = 1
    for n = 1:N
        # Extract 2D point and convert to homogeneous coordinates
        𝐦 = hom(SVector{2,Float64}(𝛉[i],𝛉[i+1]))
        reprojections_view[index₁,n] = hom⁻¹(𝐦)
        reprojections_view[index₂,n] = hom⁻¹(𝐇 * 𝐦)
        i = i + 2
    end
    reprojections
end

function jacobian_model_homography(𝐇,𝛉)
    # 2 parameters per 2D point.
    N = Int(length(𝛉) / 2)
    index₁ = SVector(1,2)
    index₂ = SVector(3,4)
    𝐉 = zeros(4*N,2*N)
    # Create a view of the jacobian matrix 𝐉 and reshape it so that
    # it will be more convenient to index into the appropriate entries
    # whilst looping over all of the data points.
    𝐉v = reshape(reinterpret(Float64,𝐉), 4, N, 2*N)
    𝐀 = SMatrix{2,3,Float64,6}(1,0,0,1,0,0)
    𝐈₃ = SMatrix{3,3}(1.0I)
    i = 1
    for n = 1:N
        # Extract 3D point and convert to homogeneous coordinates.
        𝐦 = hom(SVector{2,Float64}(𝛉[i], 𝛉[i+1]))

        # Derivative of residual in first and second image w.r.t 2D point in the
        # first image.
        ∂𝐫₁_d𝐦 = 𝐀 * 𝐈₃
        ∂𝐫₂_d𝐦 = 𝐀 * ∂hom⁻¹(𝐇 * 𝐦) * 𝐇
    @.  𝐉v[index₁,n,i:i+1] = ∂𝐫₁_d𝐦[:,index₁]
    @.  𝐉v[index₂,n,i:i+1] = ∂𝐫₂_d𝐦[:,index₁]
        i = i + 2
    end
    𝐉
end

# TODO Investigate NaNs
function jacobian_model_homography!(𝐉::Array{Float64,2}, 𝐇,𝛉)
    Base.display(𝐉)
    pause
    # 2 parameters per 2D point.
    N = Int(length(𝛉) / 2)
    index₁ = SVector(1,2)
    index₂ = SVector(3,4)
    # Create a view of the jacobian matrix 𝐉 and reshape it so that
    # it will be more convenient to index into the appropriate entries
    # whilst looping over all of the data points.
    𝐉v = reshape(reinterpret(Float64,𝐉), 4, N, 2*N)
    𝐀 = SMatrix{2,3,Float64,6}(1,0,0,1,0,0)
    𝐈₃ = SMatrix{3,3}(1.0I)
    i = 1
    for n = 1:N
        # Extract 3D point and convert to homogeneous coordinates.
        𝐦 = hom(SVector{2,Float64}(𝛉[i], 𝛉[i+1]))

        # Derivative of residual in first and second image w.r.t 2D point in the
        # first image.
        ∂𝐫₁_d𝐦 = 𝐀 * 𝐈₃
        ∂𝐫₂_d𝐦 = 𝐀 * ∂hom⁻¹(𝐇 * 𝐦) * 𝐇
    @.  𝐉v[index₁,n,i:i+1] = ∂𝐫₁_d𝐦[:,index₁]
    @.  𝐉v[index₂,n,i:i+1] = ∂𝐫₂_d𝐦[:,index₁]
        i = i + 2
    end
    𝐉
end



#TODO Verify this...
function (::ParameterErrorAnalysis)(camera₁::AbstractCamera, camera₂::AbstractCamera, world::AbstractWorld, estimates::AbstractVector)
    analogue_image₁ = get_image_type(camera₁)
    analogue_image₂ = get_image_type(camera₂)

    # Project 3D points onto the cameras.
    aquire = AquireImageContext()
    ℳ = aquire(world, camera₁)
    ℳ′ = aquire(world, camera₂)

    errors = zeros(Float64, length(estimates))
    ℋ = matrices(HomographyMatrices(camera₁, camera₂, get_planes(world)))
    for (i, estimate) in enumerate(estimates)
        𝐇ᵢ = estimate
        𝐡ᵢ = vec(𝐇ᵢ)
        𝛉 = vec(ℋ[i])
        𝐏ₜ = UniformScaling(9) -  norm(𝛉)^-2 * (𝛉*𝛉')
        errors[i] = norm(𝐏ₜ * 𝐡ᵢ)^2
    end
    errors
end
