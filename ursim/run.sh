IMAGE=universalrobots/ursim_e-series:5.13.1

docker pull "${IMAGE}" || exit 1

if [ -z "$1" ]; then
  MODEL=UR5
else
  MODEL="$1"
fi

docker run -it --rm \
  -p 5900:5900 -p 6080:6080 -p 29999-30004:29999-30004 \
  -e ROBOT_MODEL="${MODEL}" "${IMAGE}"
