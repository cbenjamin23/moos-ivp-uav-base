#!/bin/bash

# Function to merge compile_commands.json files
merge_compile_commands() {
    output_file="$1"
    shift
    echo "[" > "$output_file"

    first=true
    for file in "$@"; do
        if [ -f "$file" ]; then
            if [ "$first" = false ]; then
                echo "," >> "$output_file"
            fi
            # Remove the surrounding brackets [] from the JSON array and append
            sed '1d;$d' "$file" >> "$output_file"
            first=false
        else
            echo "File $file does not exist."
        fi
    done

    echo "]" >> "$output_file"
}

# Usage
# The first argument is the output file, and the following arguments are the input files to be merged.
merge_compile_commands "../build/combined_compile_commands.json" "../build/compile_commands.json" "../MAVSDK/build/default/compile_commands.json"
