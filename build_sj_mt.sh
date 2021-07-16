source ~/emsdk/emsdk_env.sh
export CC=emcc
export CFLAGS="-I/home/gamboa/emsdk/upstream/emscripten/system/local/include/ -s USE_PTHREADS=1"
make clean
make
mv simavr/obj-wasm32-unknown-emscripten/ simavr/obj-wasm32-unknown-emscripten_mt
