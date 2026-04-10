#!/bin/bash
set -e

BAG_DIR="${HOME}/cougars-dev/bags"

# --- Selection ---
leaf_dirs=$(cd "${BAG_DIR}" && find . -name "metadata.yaml" -exec dirname {} \; | sed 's|^\./||')

dir_list="bags\n"
for d in ${leaf_dirs}; do
  p="${d}"
  while [ "${p}" != "." ] && [ "${p}" != "" ]; do
    dir_list="${dir_list}${p}\n"
    p=$(dirname "${p}")
  done
done

selected_dir=$(echo -e "${dir_list}" | sort -u | gum filter --placeholder "Select directory or bag to plot ('bags' for all)..." || exit 0)
[ -z "${selected_dir}" ] && exit 0

if [ "${selected_dir}" == "bags" ]; then
  target_dir="${BAG_DIR}"
else
  target_dir="${BAG_DIR}/${selected_dir}"
fi

# --- Plots ---
python3 $(dirname "$0")/trace_plot.py "${target_dir}"
