#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "libarchive::archive" for configuration "Release"
set_property(TARGET libarchive::archive APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(libarchive::archive PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/libarchive.dll.a"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/libarchive.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS libarchive::archive )
list(APPEND _IMPORT_CHECK_FILES_FOR_libarchive::archive "${_IMPORT_PREFIX}/lib/libarchive.dll.a" "${_IMPORT_PREFIX}/bin/libarchive.dll" )

# Import target "libarchive::archive_static" for configuration "Release"
set_property(TARGET libarchive::archive_static APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(libarchive::archive_static PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libarchive_static.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS libarchive::archive_static )
list(APPEND _IMPORT_CHECK_FILES_FOR_libarchive::archive_static "${_IMPORT_PREFIX}/lib/libarchive_static.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
