using ComputerVision
using Test
using StaticArrays
using GeometryTypes
using LinearAlgebra
using PGFPlotsX
using Makie
using Colors


simulation_setup = VaryPlanesSimulationSetup()
experiment_conditions = construct_simulation_data(simulation_setup)

objective = ApproximateMaximumLikelihood()
solver = FundamentalNumericalScheme()
seed_estimator = ProjectiveEstimationAlgorithm(; objective = objective, solver = solver)

ConstrainedMahalanobis(; seed_estimator = seed_estimator)


#estimator = ProjectiveEstimationAlgorithm(AlgebraicLeastSquares(), DirectLinearTransform(), true)
#estimator = ProjectiveEstimationAlgorithm(ApproximateMaximumLikelihood(), FundamentalNumericalScheme(), true)
#estimator = ProjectiveEstimationAlgorithm(ReprojectionError(), BundleAdjustment(), true)
#estimator = ProjectiveEstimationAlgorithm(Mahalanobis(), ConstrainedMahalanobis(), true)
#estimator = ProjectiveEstimationAlgorithm(Mahalanobis(), ConstrainedMahalanobis(; seed_estimator = seed_estimator), true)
estimator = ProjectiveEstimationAlgorithm(Mahalanobis(), ConstrainedMahalanobis(; seed_estimator = seed_estimator, constraints = ExplicitChojnackiSzpak()), true)
#estimator = ProjectiveEstimationAlgorithm(ReprojectionError(), ConstrainedBundleAdjustment(), true)

simulation_estimates = execute_simulation(experiment_conditions, estimator)
#simulation_estimates = execute_simulation(experiment_conditions, DirectLinearTransform())
#simulation_estimates = execute_simulation(experiment_conditions, BundleAdjustment(DirectLinearTransform()))
#simulation_estimates = execute_simulation(experiment_conditions, ConstrainedBundleAdjustment(DirectLinearTransform(apply_normalisation = true),  ExplicitChojnackiSzpak()))
#simulation_estimates = execute_simulation(experiment_conditions, ConstrainedBundleAdjustment(DirectLinearTransform(apply_normalisation = true),   ImplicitChojnackiSzpak()))
#simulation_estimates = execute_simulation(experiment_conditions, ConstrainedMahalanobis(DirectLinearTransform(apply_normalisation = true),   ImplicitChojnackiSzpak()))
#results = analyze_simulation(experiment_conditions, simulation_estimates, ReconstructionErrorAnalysis())
error_analysis =  ReprojectionErrorAnalysis{Type{HomographyMatrix}}()
results = analyze_simulation(experiment_conditions, simulation_estimates, error_analysis)

axis = @pgf PGFPlotsX.Axis();
summarize = SummarizeExperimentContext(axis)

#summarize(simulation_setup, results, ReconstructionErrorAnalysis())

C = summarize(simulation_setup, results, error_analysis)

A

context = experiment_conditions[1][1]
context.correspondences[1]
estimate_context = EstimateHomographyContext()
Q = estimate_context(context.correspondences[1], DirectLinearTransform())
Q = Q / norm(Q)



H = estimate_context(context.correspondences[1], BundleAdjustment(DirectLinearTransform()))
H = H / norm(H)

context = experiment_conditions[4][5]
context.correspondences[1]
context.correspondences

cameras = context.synthetic_scene.cameras
world = context.synthetic_scene.world
structure_allotment = [get_interval(world.groups[i]) for i = 1:length(world.groups)]

camera₁, camera₂ = cameras
aquire = AquireImageContext()
ℳ = aquire(world, camera₁)
ℳ′ = aquire(world, camera₂)

f = estimate_context(context.correspondences[1], structure_allotment, ConstrainedBundleAdjustment(DirectLinearTransform()))

context.correspondences[1]
f

experiment_conditions[4]

#analyze_experiment_context()

#typeof(experiment_conditions[1][1])
#typeof(simulation_estimates[1][1])
#simulation_estimates[1][1]


# Test synthetic planar scene generation
# rois = [HyperRectangle(Vec(0.0, 0.0), Vec(50.0, 50.0)),
#         HyperRectangle(Vec(0.0, 0.0), Vec(80.0, 50.0)),
#         HyperRectangle(Vec(0.0, 0.0), Vec(50.0, 80.0)),
#         HyperRectangle(Vec(0.0, 0.0), Vec(150.0, 150.0)),
#         HyperRectangle(Vec(0.0, 0.0), Vec(80.0, 80.0))]
# rois = [HyperRectangle(Vec(0.0, 0.0), Vec(150.0, 150.0)),
#         HyperRectangle(Vec(0.0, 0.0), Vec(180.0, 150.0)),
#         HyperRectangle(Vec(0.0, 0.0), Vec(150.0, 180.0)),
#         HyperRectangle(Vec(0.0, 0.0), Vec(250.0, 250.0)),
#         HyperRectangle(Vec(0.0, 0.0), Vec(80.0, 80.0))]
# #pts = [8, 12, 16, 20, 24]
# pts = [24, 20, 16, 12, 8]

# rois = [HyperRectangle(Vec(0.0, 0.0), Vec(150.0, 150.0)), HyperRectangle(Vec(0.0, 0.0), Vec(150.0, 150.0))]
# #pts = [8, 12, 16, 20, 24]
# pts = [24, 24]
#
# #planar_scene = PlanarSyntheticScene(total_planes = 5, regions_of_interest = rois, points_per_region = pts)
# planar_scene_parameters = PlanarSceneParameters(total_planes = 2, regions_of_interest = rois, points_per_region = pts)
#
# synthetic_scene = SyntheticScene(planar_scene_parameters)
#
# apply_noise_context = ApplyNoiseContext(perturb = HomogeneousGaussianNoise(0.1), replications = 10)
# conduct_experiment_context = ExperimentContext(synthetic_scene, apply_noise_context, EstimateHomographyContext())
# estimates, runtimes = conduct_experiment_context(DirectLinearTransform())
# analyze_experiment = ExperimentAnalysisContext(synthetic_scene, apply_noise_context, ReconstructionErrorAnalysis())
# results = analyze_experiment(DirectLinearTransform(), estimates, runtimes)
