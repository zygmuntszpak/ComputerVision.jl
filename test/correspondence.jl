using GeometryTypes
using ComputerVision

𝐦 = [Point(1,2), Point(1,2)]
𝐦′ = [Point(3,4), Point(3,4)]
c = Correspondence((𝐦,𝐦′))

c[2]

length(c)
