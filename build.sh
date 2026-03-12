
sudo apt-get install libsdl2-dev libsdl2-image-dev -y
cmake . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=./build/install \
  $@
cmake --build build

cmake --build build --target install