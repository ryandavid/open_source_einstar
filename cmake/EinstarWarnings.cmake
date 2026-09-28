# Common compile options for first-party targets.
function(einstar_target_defaults target)
  target_compile_features(${target} PUBLIC cxx_std_23)
  target_compile_options(${target} PRIVATE
    -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-sign-conversion
    -Wno-missing-field-initializers
    $<$<CONFIG:Release,RelWithDebInfo>:-O3>)
endfunction()
