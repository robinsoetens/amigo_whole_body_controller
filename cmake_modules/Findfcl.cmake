  # Find fcl
#
# This sets the following variables:
# fcl_FOUND
# fcl_INCLUDE_DIRS
# fcl_LIBRARIES

find_package(PkgConfig QUIET)

message("Searching for fcl...")
pkg_check_modules(PC_fcl fcl)
message("\tPC_fcl_INCLUDEDIR: " 	${PC_fcl_INCLUDEDIR})
message("\tPC_fcl_LIBDIR: " 		${PC_fcl_LIBDIR})
message("\tCMAKE_INSTALL_PREFIX: " ${CMAKE_INSTALL_PREFIX})

find_path(fcl_INCLUDE_DIR fcl/collision.h
          PATHS "${PC_fcl_INCLUDEDIR}"
          PATHS "${CMAKE_INSTALL_PREFIX}/include"
          NO_DEFAULT_PATH
)
set(fcl_INCLUDE_DIRS ${fcl_INCLUDE_DIR})
message("\tfcl_INCLUDE_DIRS: " ${fcl_INCLUDE_DIRS})

if(MSVC)
    set(fcl_LIBRARIES optimized fcl debug fcld ccd)
else()
    find_library(fcl_LIBRARY fcl
    	PATHS "${PC_fcl_LIBDIR}"
    	PATHS "${CMAKE_INSTALL_PREFIX}/lib"
    	NO_DEFAULT_PATH
    )
    # find_library(CCD_LIBRARY ccd)
    set(fcl_LIBRARIES ${fcl_LIBRARY} ${CCD_LIBRARY})
endif()
message("\tfcl_LIBRARIES: " ${fcl_LIBRARIES})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(fcl DEFAULT_MSG fcl_INCLUDE_DIR)

mark_as_advanced(fcl_INCLUDE_DIR fcl_LIBRARY CCD_LIBRARY)

message("fcl search done!\n")