files="$files $(find . -name "CMakeCache.txt" -perm -u+w)"
files="$files $(find . -name "*.cmake" -perm -u+w)"
files="$files $(find . -name "Makefile" -perm -u+w)"
files="$files $(find . -name "CMakeFiles" -perm -u+w)"
files="$files $(find . -name "*.ninja" -perm -u+w)"
files="$files $(find . -name ".ninja_deps" -perm -u+w)"
files="$files $(find . -name ".ninja_log" -perm -u+w)"
files="$files $(find . -name ".settings" -perm -u+w)"
files="$files $(find . -name ".project" -perm -u+w)"
files="$files $(find . -name ".cproject" -perm -u+w)"
rm -r $files
exit 0
