mkdir build
cd build
cmake ..
make

konsole -e ./Play &

cd ..

gcc ./arduino_get.cpp -o arduino_get.o -lstdc++

konsole -e ./arduino_get.o &

