abstract type AbstractImage end
abstract type AbstractAnalogueImage <: AbstractImage end
abstract type AbstractDigitalImage <: AbstractImage end

struct AnalogueImage <: AbstractAnalogueImage end

# function get_data(image::AnalogueImage)
#     image.data
# end
