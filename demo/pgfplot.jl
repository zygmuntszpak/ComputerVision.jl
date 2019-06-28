using ComputerVision
using Test
using StaticArrays
using GeometryTypes
using LinearAlgebra
using PGFPlotsX
#using Makie
#using Makie # You have to add this as a dependency to your environment.

# points = [Point3(rand(0:1000.0), rand(-1000.0:1000.0), 1000.0) for n = 1:250]
# planes = [Plane(Vec3(0.0, 0.0, 1.0), 1000)]

# points₁ = [Point3(0.0, rand(-1000.0:1000.0), rand(0:1000.0)) for n = 1:250]
# planes₁ = [Plane(Vec3(1.0, 0.0, 0.0), 0)]
#
# points₂ = [Point3(rand(0:1000.0), rand(-1000.0:1000.0), 1000.0) for n = 1:250]
# planes₂ = [Plane(Vec3(0.0, 0.0, 1.0), 1000)]
#
# planes = vcat(planes₁, planes₂)
# points = vcat(points₁, points₂)
#
# Coordinates([])
#
# @pgf TikzPicture(
#         Axis(
#             PlotInc({ only_marks },
#                 Table(; x = 1:2, y = 3:4)),
#             PlotInc(
#                 Table(; x = 5:6, y = 1:2))))
#
# p = points₁[1]
#
# M₁ = reshape(reinterpret(Float64, points₁),3, 250)
# x₁ = M₁[1,:]
# y₁ = M₁[2,:]
# z₁ = M₁[3,:]
#
# M₂ = reshape(reinterpret(Float64, points₂),3, 250)
# x₂ = M₂[1,:]
# y₂ = M₂[2,:]
# z₂ = M₂[3,:]
#
# #coords = Coordinates(x, y, z)
# #col = RGB{N0f8}(1.0,1.0,0.384)
# cols = distinguishable_colors(3)
# @pgf Axis(
#     {
#
#     },
#     Plot3(
#         {
#             "only marks",
#             shader = "flat",
#             color => cols[3]
#         },
#         Table(x₁, y₁, z₁)
#     )
# )
#
# @pgf Plot3Inc(
#     {
#         "only marks",
#         shader = "flat",
#         color => cols[3]
#     },
#     Table(x₂, y₂, z₂)
# )
#
# a = @pgf Plot3(
#         {
#             "only marks",
#             shader = "flat",
#             color => cols[2]
#         },
#         Table(x₁, y₁, z₁)
#     )
#
# b = @pgf Plot3(
#             {
#                 "only marks",
#                 shader = "flat",
#                 color => cols[3]
#             },
#             Table(x₂, y₂, z₂)
#         );
#
# Axis(a,b)
#
# c =  Axis();
#
#
# a = Point3f0(0,0,0)
# b = Point3f0(1,0,0)
#
# c = [Pair(a,b)]
#
# Coordinates(c...)

 # for (i, p) in enumerate([Pair(rand(1),rand(1)), Pair(rand(1),rand(1))])
 #     @show i, p

n₁ = [1 0 0]'
d₁ = 0.0

n₂ = [0 1 0]'
d₂ = 2000.0

 K₁ = [100 0 0; 0 100 0; 0 0 1]
 R₁ = [0.642788 0.766044 0 ; 0 0 -1; -0.766044 0.642788 0]
 t₁ = [3000 0 0]'

 K₂ = [150 0 0; 0 150 0; 0 0 1]

 R₂ = [0.34202 0.939693 0 ; 0 0 -1; -0.939693 0.34202  0]

 t₂ = [4000 0 0 ]';

P₁ =  K₁*R₁*[1 0 0 -3000; 0 1 0 0; 0 0 1 0]
Projection(camera₁)

P₂ =  K₂*R₂*[1 0 0 -4000; 0 1 0 0; 0 0 1 0]


Projection(camera₂)




H₁ = (d₁ - dot(n₁,t₁))* K₂*R₂/R₁/K₁ + K₂*R₂*(t₁ - t₂) * (inv(K₁')*R₁*n₁)'

H₂ = (d₂ - dot(n₂,t₁))* K₂*R₂/R₁/K₁ + K₂*R₂*(t₁ - t₂) * (inv(K₁')*R₁*n₂)'


F = vec2antisym(K₂*R₂*(t₁ - t₂)) * K₂ * R₂ / R₁ / K₁


H₁ = H₁ / norm(H₁)
H₂ = H₂ / norm(H₂)

H₁' * F + F'*H₁

H₂' * F + F'*H₂

# 𝐇'*𝐅 + 𝐅'*𝐇

H₂ / H₂[3,3]



𝐇₂ / 𝐇₂[3,3]





K₂*R₂*(t₁ - t₂)


(d₁ - dot(n₁,t₁))

(d₁ - n₁'*t₁)

K₂*R₂/R₁/K₁

K₂*R₂*(t₁ - t₂)

inv(K₁')*R₁*n₁


Q, q = ascertain_pose(camera₂, CartesianSystem(Vec(1.0, 0.0, 0.0), Vec(0.0, 1.0, 0.0), Vec(0.0, 0.0, 1.0)))
Q'




 R₂
