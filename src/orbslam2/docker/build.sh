#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

sudo docker build \
  -f "${REPO_ROOT}/docker/Dockerfile" \
  -t orbslam2_fixed:latest \
  "${REPO_ROOT}"
