source ~/emsdk/emsdk_env.sh
export CC=emcc
export CFLAGS="-I/home/gamboa/emsdk/upstream/emscripten/system/local/include/"
#export CFLAGS="-I/home/gamboa/emsdk/emscripten/1.38.8/system/local/include/ -s USE_PTHREADS=1 -s PTHREAD_POOL_SIZE=2 -s WASM=1"
make clean
make
