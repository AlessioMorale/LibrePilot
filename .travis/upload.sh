#! /bin/bash
local_file="$(ls $TRAVIS_BUILD_DIR/build/*.deb | head -n 1)"
zip_file="$local_file".7z
echo $TRAVIS_TEST_RESULT > $TRAVIS_BUILD_DIR/build/results.log
7z a "$zip_file" "$local_file"
7z u "$zip_file" $TRAVIS_BUILD_DIR/build/results.log

target_url='ftp://188.121.43.18/builds/'

echo "Uploading $zip_file to $target_url"
curl -u $FTP_USER:$FTP_PASSWORD -T "$zip_file" "$target_url"
