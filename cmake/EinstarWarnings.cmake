# Common compile options for first-party targets.
function(einstar_target_defaults target)
  target_compile_features(${target} PUBLIC cxx_std_23)
  target_compile_options(${target} PRIVATE
    -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-sign-conversion
    -Wno-missing-field-initializers
    $<$<CONFIG:Release,RelWithDebInfo>:-O3>)
  # Static libraries reached through several dependency paths are listed more than once; the
  # Apple linker warns about it although the link is correct.
  if(APPLE)
    get_target_property(type ${target} TYPE)
    if(type STREQUAL "EXECUTABLE")
      target_link_options(${target} PRIVATE -Wl,-no_warn_duplicate_libraries)
    endif()
  endif()
endfunction()
