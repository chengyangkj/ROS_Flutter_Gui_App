#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BACKEND="${ROOT}/backend"
APP="${ROOT}/app"
PROTO_ROOT="${ROOT}/protocol"
PB_OUT="${APP}/lib/protobuf"

cd "${ROOT}"

if ! command -v protoc >/dev/null 2>&1; then
  echo "error: protoc not found (e.g. apt install protobuf-compiler)" >&2
  exit 1
fi

export PATH="${PATH}:${HOME}/.pub-cache/bin"
if ! command -v protoc-gen-dart >/dev/null 2>&1; then
  echo "error: protoc-gen-dart not found. Run: dart pub global activate protoc_plugin" >&2
  echo "  and ensure ~/.pub-cache/bin is on PATH." >&2
  exit 1
fi

if [[ ! -d "${PROTO_ROOT}" ]]; then
  echo "error: protocol directory missing: ${PROTO_ROOT}" >&2
  exit 1
fi


shopt -s nullglob
mapfile -t PROTO_FILES < <(find "${PROTO_ROOT}" -maxdepth 1 -name '*.proto' -print | sort)
shopt -u nullglob
if [[ ${#PROTO_FILES[@]} -eq 0 ]]; then
  echo "error: no .proto files under ${PROTO_ROOT}" >&2
  exit 1
fi

mkdir -p "${PB_OUT}"
protoc --experimental_allow_proto3_optional \
  -I"${PROTO_ROOT}" \
  --dart_out="${PB_OUT}" \
  "${PROTO_FILES[@]}"
echo "[build] Dart protobuf -> ${PB_OUT}"

if ! command -v flutter >/dev/null 2>&1; then
  echo "error: flutter not installed or not on PATH; cannot build app" >&2
  exit 1
fi

(cd "${APP}" && flutter pub get && flutter gen-l10n && flutter build web)
echo "[build] Flutter web done (${APP}/build/web)"

cp -r  "${APP}/build/web" "${BACKEND}/src/app/install/bin/dist"

bash "${ROOT}/backend/build.sh" "$@"
echo "[build] backend done (${BACKEND}/build)"