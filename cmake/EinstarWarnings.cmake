# Common compile options for first-party targets.
function(einstar_target_defaults target)
  target_compile_features(${target} PUBLIC cxx_std_23)
  target_compile_options(${target} PRIVATE
    -Wall -Wextra -Wpedantic
    -Wconversion -Wsign-conversion
    -Wuninitialized -Wconditional-uninitialized
    -Wmissing-field-initializers -Wmissing-designated-field-initializers
    -Wshadow -Wshadow-uncaptured-local
    -Wswitch-enum -Wimplicit-fallthrough -Wloop-analysis -Wheader-hygiene
    -Wno-expansion-to-defined
    $<$<CONFIG:Release,RelWithDebInfo>:-O3>)
  # -Werror; `cmake --compile-no-warning-as-error` builds anyway (e.g. a newer compiler with new warnings).
  set_target_properties(${target} PROPERTIES COMPILE_WARNING_AS_ERROR ON)
  # Static libraries reached through several dependency paths are listed more than once; the
  # Apple linker warns about it although the link is correct.
  if(APPLE)
    get_target_property(type ${target} TYPE)
    if(type STREQUAL "EXECUTABLE")
      target_link_options(${target} PRIVATE -Wl,-no_warn_duplicate_libraries)
    endif()
  endif()
endfunction()
