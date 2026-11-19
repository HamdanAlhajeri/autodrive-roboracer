#!/usr/bin/env bash
# Set steering_angle_to_servo_offset in the F1TENTH vesc.yaml, with a backup, and show before and after.
#
#   bash tools/set-steering-offset.sh              # sets 0.420, the measured centre of this car's servo
#   bash tools/set-steering-offset.sh 0.431        # another value
#   bash tools/set-steering-offset.sh --dry-run    # show what would change, write nothing
#   bash tools/set-steering-offset.sh --file PATH  # edit this vesc.yaml instead of the default places
#
# It changes only the value on lines that start with steering_angle_to_servo_offset (the key appears
# once for ackermann_to_vesc and once for vesc_to_odom; both must agree). Comments and every other
# line stay as they are. The old file is saved next to it as vesc.yaml.bak-<date>. It never starts or
# stops ROS and never sends a command: restart the bringup yourself afterwards.
set -uo pipefail

VALUE=0.420
DRY=0
FILES=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --dry-run) DRY=1 ;;
    --file) shift; [[ $# -gt 0 ]] || { echo "--file needs a path" >&2; exit 2; }; FILES+=("$1") ;;
    -h|--help) sed -n 2,13p "$0"; exit 0 ;;
    -*) echo "unknown option: $1" >&2; exit 2 ;;
    *) VALUE="$1" ;;
  esac
  shift
done

if ! [[ "$VALUE" =~ ^[0-9]+(\.[0-9]+)?$ ]] || ! awk -v v="$VALUE" 'BEGIN{exit !(v>=0 && v<=1)}'; then
  echo "value must be a number from 0 to 1 (a servo position), got: $VALUE" >&2
  exit 2
fi

WS="${F1TENTH_WS:-$HOME/f1tenth_ws}"
if [[ ${#FILES[@]} -eq 0 ]]; then
  FILES=("$WS/src/f1tenth_system/f1tenth_stack/config/vesc.yaml"
         "$WS/install/f1tenth_stack/share/f1tenth_stack/config/vesc.yaml"
         "$WS/build/f1tenth_stack/config/vesc.yaml")
fi

# Resolve links and edit each real file once.
REAL=()
for f in "${FILES[@]}"; do
  [[ -e "$f" ]] || continue
  r="$(readlink -f "$f" 2>/dev/null || realpath "$f")"
  skip=0
  for seen in "${REAL[@]:-}"; do [[ "$seen" == "$r" ]] && skip=1; done
  [[ $skip -eq 0 ]] && REAL+=("$r")
done
if [[ ${#REAL[@]} -eq 0 ]]; then
  echo "no vesc.yaml found. Looked in:" >&2
  printf '  %s\n' "${FILES[@]}" >&2
  echo "Pass the file with --file PATH." >&2
  exit 1
fi

STAMP="$(date +%Y%m%d-%H%M%S)"
status=0
for file in "${REAL[@]}"; do
  echo "File: $file"
  if [[ ! -w "$file" ]]; then
    echo "  not writable (permissions); nothing changed" >&2; status=1; continue
  fi
  if ! grep -Eq '^[[:space:]]*steering_angle_to_servo_offset[[:space:]]*:' "$file"; then
    echo "  no steering_angle_to_servo_offset line; nothing changed" >&2; status=1; continue
  fi
  echo "  before:"; grep -En '^[[:space:]]*steering_angle_to_servo_offset[[:space:]]*:' "$file" | sed 's/^/    /'
  if [[ $DRY -eq 1 ]]; then
    echo "  (dry run: would set the value to $VALUE)"; continue
  fi
  cp -p "$file" "$file.bak-$STAMP" || { echo "  could not write the backup; nothing changed" >&2; status=1; continue; }
  # Replace only the value; keep the indentation and any trailing comment (and a Windows line ending if present).
  sed -E -i "s/^([[:space:]]*steering_angle_to_servo_offset[[:space:]]*:[[:space:]]*)[-+0-9.eE]+/\1$VALUE/" "$file"
  echo "  after:";  grep -En '^[[:space:]]*steering_angle_to_servo_offset[[:space:]]*:' "$file" | sed 's/^/    /'
  echo "  backup: $file.bak-$STAMP"
done

if [[ $DRY -eq 0 && $status -eq 0 ]]; then
  echo
  echo "Done. Restart the bringup so it takes effect (wheels off the ground), then check that the wheels"
  echo "point straight at a zero steering command. To undo: copy the .bak-$STAMP file back."
fi
exit $status
