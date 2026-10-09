set(NOSIM_TESTS
  geodesy
  astro
  arinc424
  procedural
  phenology
  traffic
  photometry
  lod)

foreach(name IN LISTS NOSIM_TESTS)
  add_executable(test_${name} test_${name}.cpp)
  target_link_libraries(test_${name} PRIVATE nosim::nosim)
  target_compile_options(test_${name} PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall -Wextra -Wpedantic -Wconversion -Werror>)
  add_test(NAME ${name} COMMAND test_${name})
endforeach()
