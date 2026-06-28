
echo "Configuring and building ORB_SLAM2 ..."

mkdir build
cd build
cmake .. \
  -DROS_BUILD_TYPE=Release \
  -DPYTHON_EXECUTABLE:FILEPATH=/usr/bin/python3 \
  -DCMAKE_CXX_STANDARD_LIBRARIES="-lboost_system"

make -j
make install

cp ../lib/libDBoW2.so /usr/local/lib
cp ../Thirdparty/g2o/lib/libg2o.so /usr/local/lib


echo "Uncompress vocabulary ..."

cd ../Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..
