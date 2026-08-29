#!/bin/bash

#Add LVGL #if LV_USE_THORVG_INTERNAL guard
#Usage
#   find -name "*.cpp" | xargs ./add_lvgl_if.sh
#   find -name "t*.h" | xargs ./add_lvgl_if.sh

sed -i '0,/\*\/$/ {/\*\/$/ {n; s|^|\n#include "../../lv_conf_internal.h"\n#if LV_USE_THORVG_INTERNAL\n|}}' "$@"

sed -i -e '$a\
\
#endif /* LV_USE_THORVG_INTERNAL */' "$@"
