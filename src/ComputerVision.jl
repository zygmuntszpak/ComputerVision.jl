module ComputerVision

using StaticArrays, GeometryTypes, LinearAlgebra, Optim, Makie, PGFPlotsX, Colors

abstract type ProjectiveEntity end

include("util.jl")
include("rotations.jl")
include("model/correspondence.jl")
include("model/coordinate_system.jl")
include("model/allotment.jl")
include("model/image.jl")
include("model/camera.jl")
include("model/plane.jl")
include("model/world.jl")
include("model/projection.jl")
include("model/pose.jl")
include("model/fundamental_matrix.jl")
include("model/essential_matrix.jl")
include("model/homography_matrix.jl")
include("model/estimators.jl")
include("view/visualize_properties.jl")
include("context/context.jl")
include("context/aquire_context.jl")
include("context/normalize_data_context.jl")
include("context/estimate_homography_context.jl")
include("context/decompose_homography_context.jl")
include("context/triangulate_context.jl")
include("context/coordinate_system_transformation_context.jl")
include("context/visualize_context.jl")


export AbstractCoordinateTransformationContext,
       WorldCoordinateTransformationContext


export AbstractAllotment,
       IntervalAllotment,
       get_allotment

export AbstractCorrespondences,
       Correspondences

export hartley_transformation,
       hartley_normalization

export FundamentalMatrix,
       EssentialMatrix,
       HomographyMatrix,
       HomographyMatrices,
       to_matrix,
       matrix,
       matrices

export TriangulateContext,
       AbstractTriangulationAlgorithm,
       DirectLinearTriangulation

export AbstractPose,
       RelativePose,
       CoordinateTransformation,
       AbstractCoordinateTransformation,
       rotation,
       translation

export PoseFromSingleHomographyContext,
       MalisVargasDecomposition


export Projection,
       AquireImageContext,
       AbstractContext,
       NormalizeDataContext,
       HartleyNormalizeDataContext,
       EstimateHomographyContext,
       VisualizeWorldContext,
       MakieVisualProperties,
       PGFPlotsVisualProperties,
       AbstractVisualProperties

export DirectLinearTransform

export project

export get_visualize_properties,
       set_scene!,
       get_scene,
       get_linewidth,
       set_linewidth!,
       get_markersize,
       set_markersize!,
       get_scale,
       set_scale!

export AbstractCoordinateSystem,
       AbstractPlanarCoordinateSystem,
       PlanarCartesianSystem,
       CartesianSystem,
       RasterSystem,
       OpticalSystem

export get_coordinate_system



export AbstractAnalgoueImage,
       AbstractDigitalImage,
       AbstractImage,
       AnalogueImage

#export  get_data

export  AbstractCamera,
        AbstractCameraModel,
        Pinhole,
        AbstractIntrinsicParameters,
        AbstractExtrinsicParameters,
        IntrinsicParameters,
        ExtrinsicParameters,
        Camera

export AbstractPlane,
       Plane,
       PlaneSegment,
       AbstractWorld,
       PrimitiveWorld


export  get_points,
        set_points!,
        get_planes,
        set_planes!,
        get_groups,
        set_groups,
        get_normal,
        get_distance,
        get_plane,
        set_plane,
        get_segment

export  hom⁻¹,
        hom,
        vec2antisym,
        smallest_eigenpair

export  get_focal_length,
        set_focal_length!,
        get_width,
        set_width!,
        get_height!,
        set_height!,
        #get_centroid,
        #set_centroid!,
        get_origin,
        get_basis_vectors,
        get_intrinsics,
        set_intrinsics,
        get_extrinsics,
        set_extrinsics,
        get_e₁,
        set_e₁!,
        get_e₂,
        set_e₂!,
        get_e₃,
        set_e₃!,
        rotate!,
        translate!,
        relocate!,
        rotxyz,
        #get_origin,
        #set_origin,
        get_model,
        set_model!,
        get_image_type,
        set_image_type!,
        ascertain_pose







end # module
