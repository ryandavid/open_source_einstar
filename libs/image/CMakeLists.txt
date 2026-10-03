# Photos: decoding (JPEG, HEIC, PNG ... with EXIF orientation), metadata, encoding and simple drawing, through
# the system's ImageIO / CoreGraphics / CoreText.
add_library(einstar_image src/image.cpp)
add_library(einstar::image ALIAS einstar_image)
target_include_directories(einstar_image PUBLIC include)
target_link_libraries(einstar_image PUBLIC einstar::core
                      PRIVATE "-framework CoreGraphics" "-framework ImageIO" "-framework CoreText" "-framework CoreFoundation")
einstar_target_defaults(einstar_image)
