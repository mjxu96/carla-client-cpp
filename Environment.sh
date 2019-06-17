#! /bin/bash

# Sets the environment for other shell scripts.

set -e

# source $(dirname "$0")/Vars.mk


# ==============================================================================
# -- Environment variable definitions ------------------------------------------
# ==============================================================================

# Here CURDIR is assumed to be the root folder of the project.

CARLA_ROOT_FOLDER=${CURDIR}
CARLA_BUILD_FOLDER=${CURDIR}/lib

LIBCARLA_ROOT_FOLDER=${CURDIR}/src/libcarla


CMAKE_CONFIG_FILE=${CARLA_BUILD_FOLDER}/CMakeLists.txt.in

LIBCARLA_TEST_CONTENT_FOLDER=${CARLA_BUILD_FOLDER}/test-content

# libcarla variables

LIBCARLA_BUILD_PATH=${CURDIR}/build
LIBCARLA_LIB_INSTALL_PATH=${CURDIR}
LIBCARLA_HEADER_INSTALL_PATH=${CURDIR}
LIBCARLA_BUILD_TOOLCHAIN=${CURDIR}/lib/Toolchain.cmake

CARLA_BUILD_CONCURRENCY=`nproc --all`

# ==============================================================================
# -- Useful function definitions -----------------------------------------------
# ==============================================================================

function log {
  echo -e "\033[1;35m`basename "$0"`: $1\033[0m"
}

function fatal_error {
  echo -e >&2 "\033[0;31m`basename "$0"`: ERROR: $1\033[0m"
  exit 2
}


function get_git_repository_version {
  echo -e "0.9.5-158-g114d54d8"
  # git describe --tags --dirty --always
}

function copy_if_changed {
  mkdir -p $(dirname $2)
  rsync -cIr --out-format="%n" $1 $2
}

function move_if_changed {
  copy_if_changed $1 $2
  rm -f $1
}

