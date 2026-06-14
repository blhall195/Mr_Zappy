build target="main":
    ./build.sh {{target}}

lsp:
    cd "C++/Main Board C++" && pio run -t compiledb
