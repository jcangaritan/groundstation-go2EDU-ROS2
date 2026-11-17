#!/usr/bin/env bash
# save_map.sh - Pide el guardado del mapa al servicio std_srvs/Trigger (por defecto /map_save).
# Uso: ./save_map.sh            o con otro nombre:  MAP_SERVICE=/otro_servicio ./save_map.sh
set -u
SERVICE="${MAP_SERVICE:-/map_save}"
TIMEOUT="${MAP_TIMEOUT:-60}"

if ! ros2 service list 2>/dev/null | grep -qx "$SERVICE"; then
  echo "El servicio $SERVICE no existe. Servicios que mencionan map/save:"
  ros2 service list 2>/dev/null | grep -i -E "map|save|pcd" || echo "  (ninguno)"
  exit 1
fi

echo "[save_map] llamando a $SERVICE (máx ${TIMEOUT}s)..."
OUT="$(timeout "$TIMEOUT" ros2 service call "$SERVICE" std_srvs/srv/Trigger "{}" 2>&1)"
RC=$?
echo "$OUT"
[ $RC -eq 124 ] && { echo "[save_map] ERROR: tiempo agotado."; exit 1; }
echo "$OUT" | grep -q "success=True" && { echo "[save_map] OK"; exit 0; }
echo "[save_map] ERROR: el servicio respondió sin éxito."; exit 1
