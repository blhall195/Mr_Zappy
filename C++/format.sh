#!/usr/bin/env bash

if ! command -v clang-format > /dev/null 2>&1; then
    echo "clang-format not installed"
    echo "To install with apt:"
    echo "sudo apt update && sudo apt install clang-format"
    exit 1
fi

while IFS= read -r -d '' file; do
    echo "Formatting $file"
    clang-format -i "$file"
done < <(find . -name '.pio' -prune -o \( -name "*.cpp" -o -name "*.h" \) -print0)

echo "Done"
