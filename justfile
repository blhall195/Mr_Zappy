build target="main":
    ./build.sh {{target}}

lsp:
    cd "C++/Main Board C++" && pio run -t compiledb

test:
    cd "C++/Main board C++" && pio test -e native

format:
    cd "C++" && ./format.sh
