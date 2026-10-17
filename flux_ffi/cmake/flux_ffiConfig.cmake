# flux::ffi links the shared library, so armo and other uprobe tools find libflux_ffi.so in the
# process's maps. flux::ffi_static is the static archive.
get_filename_component(_flux_prefix "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)

if(NOT TARGET flux::ffi)
  file(STRINGS "${_flux_prefix}/include/flux.h" _flux_abi REGEX "^#define FLUX_ABI_VERSION ")
  # A consumer configures again when the installed ABI changes, or it links the old soname.
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_flux_prefix}/include/flux.h")
  string(REGEX REPLACE ".* " "" _flux_abi "${_flux_abi}")
  add_library(flux::ffi SHARED IMPORTED)
  set_target_properties(flux::ffi PROPERTIES
    IMPORTED_LOCATION "${_flux_prefix}/lib/libflux_ffi.so.${_flux_abi}"
    IMPORTED_SONAME "libflux_ffi.so.${_flux_abi}"
    INTERFACE_INCLUDE_DIRECTORIES "${_flux_prefix}/include")
endif()

if(NOT TARGET flux::ffi_static)
  add_library(flux::ffi_static STATIC IMPORTED)
  set_target_properties(flux::ffi_static PROPERTIES
    IMPORTED_LOCATION "${_flux_prefix}/lib/libflux_ffi.a"
    INTERFACE_INCLUDE_DIRECTORIES "${_flux_prefix}/include"
    INTERFACE_LINK_LIBRARIES "pthread;dl;m")
endif()

unset(_flux_prefix)
unset(_flux_abi)
