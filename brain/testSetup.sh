DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
LINUX_PORT=${DIR}/build/virtual_linux_serial_port
CREATE_PORT=${DIR}/build/virtual_create_serial_port
killall socat
socat pty,raw,link=${LINUX_PORT},echo=0 pty,raw,link=${CREATE_PORT},echo=0 &
echo "Virtual ports created"
echo "iRobot Create: ${CREATE_PORT}"
echo "linux: ${LINUX_PORT}"
