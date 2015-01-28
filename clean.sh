files="$files $(find . -name "CMakeCache.txt" -perm -u+w)"
files="$files $(find . -name "*.cmake" -perm -u+w)"
files="$files $(find . -name "Makefile" -perm -u+w)"
files="$files $(find . -name "CMakeFiles" -perm -u+w)"
echo $files
rm -r $files
