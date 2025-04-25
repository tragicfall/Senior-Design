#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "roam_bot::roam_bot" for configuration ""
set_property(TARGET roam_bot::roam_bot APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(roam_bot::roam_bot PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libroam_bot.so"
  IMPORTED_SONAME_NOCONFIG "libroam_bot.so"
  )

list(APPEND _cmake_import_check_targets roam_bot::roam_bot )
list(APPEND _cmake_import_check_files_for_roam_bot::roam_bot "${_IMPORT_PREFIX}/lib/libroam_bot.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
