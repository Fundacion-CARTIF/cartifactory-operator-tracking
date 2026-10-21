#!/usr/bin/env bash
set -euo pipefail

# Ejecutar siempre desde el directorio que contiene el Compose.
cd "$(dirname "${BASH_SOURCE[0]}")"

COMPOSE_FILE="docker-compose-mapping.yml"
export MAPPER_UID="$(id -u)"
export MAPPER_GID="$(id -g)"

action="${1:-start}"

prepare_output() {
  mkdir -p fiware/out
  if [[ ! -w fiware/out ]]; then
    echo "ERROR: no tienes permisos de escritura en fiware/out"
    echo "Comprueba: ls -ld fiware/out && ls -l fiware/out"
    exit 1
  fi
}

case "$action" in
  start|up)
    prepare_output
    docker compose -f "$COMPOSE_FILE" up -d --build
    docker compose -f "$COMPOSE_FILE" ps
    printf '\nInterfaz web (en este equipo): http://localhost:3001\n'
    ;;
  stop|down)
    docker compose -f "$COMPOSE_FILE" down
    ;;
  status|ps)
    docker compose -f "$COMPOSE_FILE" ps
    ;;
  logs)
    docker compose -f "$COMPOSE_FILE" logs -f
    ;;
  rebuild)
    prepare_output
    docker compose -f "$COMPOSE_FILE" build --no-cache
    docker compose -f "$COMPOSE_FILE" up -d --force-recreate
    docker compose -f "$COMPOSE_FILE" ps
    ;;
  *)
    echo "Uso: bash start-mapping.sh [start|status|logs|stop|rebuild]"
    exit 2
    ;;
esac
