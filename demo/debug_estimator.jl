using ComputerVision
using Test
using StaticArrays
using GeometryTypes
using LinearAlgebra
using PGFPlotsX
using Makie
using Colors


planes = [2]
roi_dimensions = [[400.0, 400.0]]
noise_per_roi = [[1.0, 1.0]]
points_per_roi = [[24, 24]]
replications = 1
trials = 1

simulation_setup = VaryPlanesSimulationSetup(planes = planes,
                                             roi_dimensions = roi_dimensions,
                                             noise_per_roi = noise_per_roi,
                                             points_per_roi = points_per_roi,
                                             replications = replications,
                                             trials = trials)
experiment_conditions = construct_simulation_data(simulation_setup)
experiment_context = experiment_conditions[1][1]
synthetic_scene = experiment_context.synthetic_scene
world = synthetic_scene.world
cameras = synthetic_scene.cameras
camera₁, camera₂ = cameras
aquire = AquireImageContext()
ℳ = aquire(world, camera₁)
ℳ′ = aquire(world, camera₂)

axis = Scene()
visualize = VisualizeWorldContext(; scene = axis, visual_properties = MakieVisualProperties(scale = 150, markersize = 25))
visualize(world, cameras)
scene = get_scene(visualize)
display(scene)

structure_allotment = [get_interval(world.groups[i]) for i = 1:length(world.groups)]
correspondences = Correspondences(tuple(ℳ,ℳ′))

σ = 1
σ² = σ^2
apply_noise = ApplyNoiseContext(perturb = HomogeneousGaussianNoise(σ))
noisy_correspondences = first(apply_noise(world, camera₁, camera₂))
estimate_context = EstimateHomographyContext()

objective = ApproximateMaximumLikelihood()
solver = FundamentalNumericalScheme()
seed_estimator = ProjectiveEstimationAlgorithm(; objective = objective, solver = solver)
#estimator = ProjectiveEstimationAlgorithm(Mahalanobis(), ConstrainedMahalanobis(; seed_estimator = seed_estimator), true)
estimator = ProjectiveEstimationAlgorithm(Mahalanobis(), ConstrainedMahalanobis(; seed_estimator = seed_estimator, constraints = ExplicitChojnackiSzpak()), true)


#estimator = ProjectiveEstimationAlgorithm(AlgebraicLeastSquares(), DirectLinearTransform(), true)
#estimator = ProjectiveEstimationAlgorithm(ApproximateMaximumLikelihood(), FundamentalNumericalScheme(), true)
#estimator = ProjectiveEstimationAlgorithm(ReprojectionError(), BundleAdjustment(ProjectiveEstimationAlgorithm()), true)
#f = estimate_context(noisy_correspondences,  estimator)
#f = estimate_context(noisy_correspondences,  estimator)

# ConstrainedBundleAdjustment()
# seed_estimator = ProjectiveEstimationAlgorithm()
# estimator = ProjectiveEstimationAlgorithm(ReprojectionError(), ConstrainedBundleAdjustment(), true)
f = estimate_context(noisy_correspondences,  structure_allotment,  estimator)

typeof(ConstrainedBundleAdjustment()) <: ComputerVision.AbstractConstrainedProjectiveOptimizationScheme

#f = estimate_context(context.correspondences[1], structure_allotment, ConstrainedBundleAdjustment(DirectLinearTransform()))
#f = estimate_context(correspondences₀, structure_allotment, ConstrainedBundleAdjustment(DirectLinearTransform()))
#g = estimate_context(correspondences₁, structure_allotment, ConstrainedBundleAdjustment(DirectLinearTransform()))

#f = estimate_context(noisy_correspondences, structure_allotment, ConstrainedBundleAdjustment(DirectLinearTransform(), ExplicitChojnackiSzpak()))
#f = estimate_context(noisy_correspondences, structure_allotment, ConstrainedMahalanobis(DirectLinearTransform(), ImplicitChojnackiSzpak()))


Θ = [vec(f[1]), vec(f[2])]
cost_context = CostContext(ApproximateMaximumLikelihood(), EstimateHomographyContext())
cost = cost_context(Θ, structure_allotment, noisy_correspondences)
sqrt(cost[2] / (24-8))

uncertainty_context = UncertaintyContext()

npts = first(sum.(points_per_roi))
Λ = [SMatrix{2,2}(σ² * Matrix(Diagonal([1.0,1.0]))) for i = 1:npts]
Λ′ = [SMatrix{2,2}(σ² * Matrix(Diagonal([1.0,1.0]))) for i = 1:npts]
𝚲 = Correspondences(tuple(Λ, Λ′))
uncertainty_context([HomographyMatrix(f[1]), HomographyMatrix(f[2])],structure_allotment, noisy_correspondences, 𝚲)




𝐅 = matrix(FundamentalMatrix( camera₁, camera₂))
H₀ = f[1]/ norm(f[1])
H₁ = f[2]/ norm(f[2])
H₀' * 𝐅 + 𝐅'*H₀
H₁' * 𝐅 + 𝐅'*H₁


eigen(Array(inv(H₀) * 𝐅))
eigen(Array(inv(H₁) * 𝐅))

eigen(Array(30*𝐅 * inv(H₀)))

eigen(Array(inv(H₁) * 𝐅 * inv(H₁)))


e = (eigen(Array(inv(H₀) * H₁)))
real.(e.values)


s₀ = structure_allotment[1]
s₁ = structure_allotment[2]
H₀ = estimate_context(Correspondences(tuple(ℳ[s₀], ℳ′[s₀])), DirectLinearTransform())
H₁ = estimate_context(Correspondences(tuple(ℳ[s₁], ℳ′[s₁])), DirectLinearTransform())


𝒫₀ = noisy_correspondences[1][s₀]
𝒫₀′ = noisy_correspondences[2][s₀]

𝒫₁  = noisy_correspondences[1][s₁]
𝒫₁′ = noisy_correspondences[2][s₁]

# G₀ = estimate_context(Correspondences(tuple(noisy_correspondences[1][s₀], noisy_correspondences[2][s₀])),  DirectLinearTransform())
# G₁ = estimate_context(Correspondences(tuple(noisy_correspondences[1][s₁], noisy_correspondences[2][s₁])),  DirectLinearTransform())
# G₀ = estimate_context(Correspondences(tuple(𝒫₀, 𝒫₀′)),  DirectLinearTransform())
# G₁ = estimate_context(Correspondences(tuple(𝒫₁ , 𝒫₁′)),  DirectLinearTransform())
G₀ = estimate_context(Correspondences(tuple(𝒫₀, 𝒫₀′)),  BundleAdjustment(DirectLinearTransform()))
G₁ = estimate_context(Correspondences(tuple(𝒫₁ , 𝒫₁′)),  BundleAdjustment(DirectLinearTransform()))

H₀ / norm(H₀)
H₁ / norm(H₁)





G₀ / norm(G₀)
G₁ / norm(G₁)

score = zeros(length(𝒫₀))
for n = 1:length(𝒫₀)
    𝐦 = hom(𝒫₀[n])
    m′ = 𝒫₀′[n]
    #𝐲 = hom⁻¹(H₀  * 𝐦)
    𝐲 = hom⁻¹(G₀  * 𝐦)
    @show m′, 𝐲, norm(m′ - 𝐲)^2
end

for n = 1:length(𝒫₁)
    𝐦 = hom(𝒫₁[n])
    m′ = 𝒫₁′[n]
    #𝐲 = hom⁻¹(H₀  * 𝐦)
    𝐲 = hom⁻¹(G₁   * 𝐦)
    @show m′, 𝐲, norm(m′ - 𝐲)^2
end
#correspondences₀

# Visualize the set of corresponding points
M₀ = reshape(reinterpret(Float64,ℳ[s₀]),(2,length(ℳ[s₀])))
M₁ = reshape(reinterpret(Float64,ℳ[s₁]),(2,length(ℳ[s₁])))
M₀′ = reshape(reinterpret(Float64,ℳ′[s₀]),(2,length(ℳ′[s₀])))
M₁′ = reshape(reinterpret(Float64,ℳ′[s₁]),(2,length(ℳ′[s₁])))

P₀ = reshape(reinterpret(Float64,𝒫₀),(2,length(𝒫₀)))
P₀′ = reshape(reinterpret(Float64,𝒫₀′),(2,length(𝒫₀′)))
P₁ = reshape(reinterpret(Float64,𝒫₁),(2,length(𝒫₁)))
P₁′ = reshape(reinterpret(Float64,𝒫₁′),(2,length(𝒫₁′)))

# # Truth...
# scene = Scene()
# scatter!(scene, M₀[1,:], M₀[2,:], markersize = 10, color = :red, limits = FRect(-320, -200, 640, 480.0))
# scatter!(scene, M₁[1,:], M₁[2,:], markersize = 10, color = :blue, limits = FRect(-320, -200, 640, 480.0))
#
# scatter!(scene, M₀′[1,:], M₀′[2,:], markersize = 10, color = :green, limits = FRect(-320, -200, 640, 480.0))
# scatter!(scene, M₁′[1,:], M₁′[2,:], markersize = 10, color = :black, limits = FRect(-320, -200, 640, 480.0))


scene = Scene()
scatter!(scene, M₀[1,:], M₀[2,:], markersize = 10, color = :red, limits = FRect(-320, -200, 640, 480.0))
scatter!(scene, M₁[1,:], M₁[2,:], markersize = 10, color = :blue, limits = FRect(-320, -200, 640, 480.0))

scatter!(scene, P₀[1,:],P₀[2,:], markersize = 10, color = :green, limits = FRect(-320, -200, 640, 480.0))
scatter!(scene, P₁[1,:], P₁[2,:], markersize = 10, color = :black, limits = FRect(-320, -200, 640, 480.0))


scene = Scene()
scatter!(scene, M₀′[1,:], M₀′[2,:], markersize = 10, color = :red, limits = FRect(-320, -200, 640, 480.0))
scatter!(scene, M₁′[1,:], M₁′[2,:], markersize = 10, color = :blue, limits = FRect(-320, -200, 640, 480.0))

scatter!(scene, P₀′[1,:],P₀′[2,:], markersize = 10, color = :green, limits = FRect(-320, -200, 640, 480.0))
scatter!(scene, P₁′[1,:], P₁′[2,:], markersize = 10, color = :black, limits = FRect(-320, -200, 640, 480.0))


noisy_correspondences

𝐈₃ = SMatrix{3,3}(1.0I)

kron(b, ones(3,3))


a = zeros(9,12)
b = ones(9,3)
k = 4
s = (k-1) * 3 + 1
e = s + 2
a[:,s:e] .= b

a
