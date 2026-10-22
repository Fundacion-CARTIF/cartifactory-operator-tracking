#!/usr/bin/env bash
set -e

source /opt/vulcanexus/jazzy/setup.bash

if [ -f /cartifactory_ws/install/setup.bash ]; then
    source /cartifactory_ws/install/setup.bash
fi

# Copiar automáticamente los archivos nuevos al volumen Linux.
if [ -d /ros_bags_source ] && [ -d /ros_bags ]; then
    echo "[entrypoint] Importando grabaciones nuevas..."
    rsync -a --ignore-existing /ros_bags_source/ /ros_bags/
    echo "[entrypoint] Grabaciones disponibles en /ros_bags."
fi

exec "$@"