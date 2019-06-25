using Documenter, ComputerVision

makedocs(;
    modules=[ComputerVision],
    format=Documenter.HTML(),
    pages=[
        "Home" => "index.md",
    ],
    repo="https://github.com/zygmuntszpak/ComputerVision.jl/blob/{commit}{path}#L{line}",
    sitename="ComputerVision.jl",
    authors="Dr. Zygmunt L. Szpak",
    assets=String[],
)

deploydocs(;
    repo="github.com/zygmuntszpak/ComputerVision.jl",
)
