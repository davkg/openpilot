#!/usr/bin/env bash

export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export VECLIB_MAXIMUM_THREADS=1

if [ -z "$AGNOS_VERSION" ]; then
  export AGNOS_VERSION="12.4"
fi

export STAGING_ROOT="/data/safe_staging"

export FINGERPRINT="ALFA_ROMEO_STELVIO_1ST_GEN"
export SKIP_FW_QUERY=1
