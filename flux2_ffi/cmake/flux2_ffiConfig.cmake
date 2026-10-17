# flux2::ffi links the shared library, so armo and other uprobe tools find libflux2_ffi.so in the
# process's maps. flux2::ffi_static is the static archive.
get_filename_component(_flux2_prefix "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)

if(NOT TARGET flux2::ffi)
  file(STRINGS "${_flux2_prefix}/include/flux2.h" _flux2_abi REGEX "^#define FLUX2_ABI_VERSION ")
  # A consumer configures again when the installed ABI changes, or it links the old soname.
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_flux2_prefix}/include/flux2.h")
  string(REGEX REPLACE ".* " "" _flux2_abi "${_flux2_abi}")
  add_library(flux2::ffi SHARED IMPORTED)
  set_target_properties(flux2::ffi PROPERTIES
    IMPORTED_LOCATION "${_flux2_prefix}/lib/libflux2_ffi.so.${_flux2_abi}"
    IMPORTED_SONAME "libflux2_ffi.so.${_flux2_abi}"
    INTERFACE_INCLUDE_DIRECTORIES "${_flux2_prefix}/include")
endif()

if(NOT TARGET flux2::ffi_static)
  add_library(flux2::ffi_static STATIC IMPORTED)
  set_target_properties(flux2::ffi_static PROPERTIES
    IMPORTED_LOCATION "${_flux2_prefix}/lib/libflux2_ffi.a"
    INTERFACE_INCLUDE_DIRECTORIES "${_flux2_prefix}/include"
    INTERFACE_LINK_LIBRARIES "pthread;dl;m")
endif()

unset(_flux2_prefix)
unset(_flux2_abi)
