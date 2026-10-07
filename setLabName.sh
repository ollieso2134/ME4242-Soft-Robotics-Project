#!/bin/bash
if [ $# -eq 0 ]; then
        echo "You need to give a lab name: './setLabName.sh myLabName'"
else
    mv lab_empty.py lab_$1.py
    mv lab_empty.md lab_$1.md
    sed -i 's/Lab Empty/Lab '$1'/g' lab_$1.md
    echo "Done renaming lab: '$1'"
fi

