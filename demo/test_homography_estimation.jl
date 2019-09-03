using ComputerVision
using Test
using StaticArrays
using GeometryTypes
using LinearAlgebra
using PGFPlotsX
using Makie
using Colors


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

rois = [HyperRectangle(Vec(0.0, 0.0), Vec(150.0, 150.0)), HyperRectangle(Vec(0.0, 0.0), Vec(150.0, 150.0))]
#pts = [8, 12, 16, 20, 24]
pts = [24, 24]

#planar_scene = PlanarSyntheticScene(total_planes = 5, regions_of_interest = rois, points_per_region = pts)
planar_scene_parameters = PlanarSceneParameters(total_planes = 2, regions_of_interest = rois, points_per_region = pts)

synthetic_scene = SyntheticScene(planar_scene_parameters)

apply_noise_context = ApplyNoiseContext(perturb = HomogeneousGaussianNoise(0.1), replications = 10)
conduct_experiment_context = ExperimentContext(synthetic_scene, apply_noise_context, EstimateHomographyContext())
estimates, runtimes = conduct_experiment_context(DirectLinearTransform())
analyze_experiment = ExperimentAnalysisContext(synthetic_scene, apply_noise_context, ReconstructionErrorAnalysis())
results = analyze_experiment(DirectLinearTransform(), estimates, runtimes)
