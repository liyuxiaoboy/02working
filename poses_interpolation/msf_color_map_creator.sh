#! /bin/bash
set -e

if [ $# -lt 2 ]; then
    echo "Usage: map_color_map_creator.sh [bags folder] [lidar_gnss_extrinsics] [zone_id] [map folder] [camera_extrinsics] [camera_intrinsics] [points_range, 0 means no limitation]"
    exit 1;
fi

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "${DIR}/.."

source "${DIR}/apollo_base.sh"

GNSS_LOC_TOPIC="/apollo/localization/msf_gnss"
LIDAR_LOC_TOPIC="/apollo/localization/msf_lidar"
FUSION_LOC_TOPIC="/apollo/localization/pose"  
ODOMETRY_LOC_TOPIC="/apollo/sensor/gnss/odometry"
CLOUD_TOPIC=$CLOUD_TOPIC
if [ -z $CLOUD_TOPIC ]; then
  CLOUD_TOPIC="/apollo/sensor/velodyne32_2/compensator/PointCloud2"
fi

IMAGE_TOPIC="/apollo/sensor/camera/traffic/image_short"

GNSS_LOC_FILE="gnss_loc.txt"
LIDAR_LOC_FILE="lidar_loc.txt"
FUSION_LOC_FILE="fusion_loc.txt"
ODOMETRY_LOC_FILE="odometry_loc.txt"

IN_FOLDER=$(realpath $1)
EXTRINSIC_FILE=$(realpath $2)
ZONE_ID=$3
MAP_FOLDER=$(realpath $4)
CAM_EXTRINSICS=$(realpath $5)
CAM_INTRINSICS=$(realpath $6)
RANGE=$7

function data_exporter() {
  local BAG_FILE=$1
  local OUT_FOLDER=$2
  $APOLLO_BIN_PREFIX/modules/localization/msf/local_tool/data_extraction/monitor_data_exporter \
    --bag_file $BAG_FILE \
    --out_folder $OUT_FOLDER \
    --cloud_topic $CLOUD_TOPIC \
    --gnss_loc_topic $GNSS_LOC_TOPIC \
    --lidar_loc_topic $LIDAR_LOC_TOPIC \
    --fusion_loc_topic $FUSION_LOC_TOPIC \
    --odometry_loc_topic $ODOMETRY_LOC_TOPIC
}

function poses_interpolation() {
  local INPUT_POSES_PATH=$1
  local REF_TIMESTAMPS_PATH=$2
  local EXTRINSIC_PATH=$3
  local GNSS_BEST_POSE_PATH=$4
  local GNSS_STATUS_PATH=$5
  local OUTPUT_POSES_PATH=$6
  local CHASSIS_PATH=$7
  $APOLLO_BIN_PREFIX/modules/localization/msf/local_tool/map_creation/poses_interpolator \
    --input_poses_path $INPUT_POSES_PATH \
    --ref_timestamps_path $REF_TIMESTAMPS_PATH \
    --extrinsic_path $EXTRINSIC_PATH \
    --gnss_best_pose_path $GNSS_BEST_POSE_PATH \
    --gnss_status_path $GNSS_STATUS_PATH \
    --output_poses_path $OUTPUT_POSES_PATH \
    --chassis_path $CHASSIS_PATH \
    --sim_control_conf_file /apollo/modules/dreamview/conf/moonx.pb.txt \
    --vehicle_config_path /apollo/modules/common/data/mkz_config.pb.txt
}

function pose_refinement() {
  local PCD_FOLDER=$1
  local SRC_POSE_FILE=$2
  local DEST_POSE_FILE=$3
  $APOLLO_BIN_PREFIX/modules/localization/msf/local_tool/pose_refinement/pose_refinement \
    --pcd_folder $PCD_FOLDER --src_pose_file $SRC_POSE_FILE --dest_pose_file $DEST_POSE_FILE
}

function create_pointcloud_map() {
  $APOLLO_BIN_PREFIX/modules/localization/msf/local_tool/map_creation/pointcloud_map_creator \
    --plane_inliner_only=false \
    --pcd_folder $1 \
    --pose_file $2 \
    --map_folder $MAP_FOLDER/pointcloud_map \
    --zone_id $ZONE_ID \
    --coordinate_type UTM \
    --map_resolution_type single \
    --map_resolution 0.0625 \
    --refine_overlap=true \
    --calib_file $EXTRINSIC_FILE \
    --apply_to_map=$3
}

function image_exportot() {
  $APOLLO_BIN_PREFIX/modules/map/util/colored_pointcloud/bag2image \
    --bag_file $1 \
    --output_folder $2 \
    --topic_name $IMAGE_TOPIC
}

function color_pointcloud() {
  $APOLLO_BIN_PREFIX/modules/map/util/colored_pointcloud/colored_pointcloud \
    --pcd_poses_path $1 \
    --img_folder $2 \
    --lidar_poses_at_img_time $3 \
    --lidar_cam_intrinsics $CAM_INTRINSICS \
    --lidar_cam_extrinsics $CAM_EXTRINSICS \
    --output_pcd_folder $4 \
    --range $RANGE
}

function create_color_map() {
  $APOLLO_BIN_PREFIX/modules/localization/msf/local_tool/color_map/create_color_map_from_pcd \
    --pcd_folder $1 \
    --xml_file $2 \
    --pcd_pose_file $3 \
    --zone_id $ZONE_ID \
    --output_folder $4
}

cd $IN_FOLDER
for item in $(ls -l *.bag | awk '{print $9}')
do
  DIR_NAME=`pwd`/$(echo $item | cut -d . -f 1)
  if [ ! -d ${DIR_NAME} ]; then
    mkdir -p $DIR_NAME
    mkdir -p $DIR_NAME/image
    echo "========= || Data Exporter || ========="
    image_exportot "${item}" "${DIR_NAME}/image/"
    data_exporter "${item}" "${DIR_NAME}"
    echo "||========= Pose Interpolator =========||"
    poses_interpolation \
        "${DIR_NAME}/pcd/${ODOMETRY_LOC_FILE}" \
        "${DIR_NAME}/pcd/pcd_timestamp.txt" "${EXTRINSIC_FILE}" \
        "${DIR_NAME}/pcd/gnss_best_pose.txt" \
        "${DIR_NAME}/pcd/gnss_status.txt" \
        "${DIR_NAME}/pcd/corrected_poses.txt" \
        "${DIR_NAME}/pcd/chassis.txt"
    poses_interpolation \
        "${DIR_NAME}/pcd/${ODOMETRY_LOC_FILE}" \
        "${DIR_NAME}/image/img_timestamps.txt" "${EXTRINSIC_FILE}" \
        "${DIR_NAME}/pcd/gnss_best_pose.txt" \
        "${DIR_NAME}/pcd/gnss_status.txt" \
        "${DIR_NAME}/image/corrected_poses.txt" \
        "${DIR_NAME}/pcd/chassis.txt"
  fi
done

BAG_ARR=($(ls -l *.bag | awk '{print $9}'))
for((i=1;i<=${#BAG_ARR[*]};i++))
do
  item=${BAG_ARR[$i - 1]}
  DIR_NAME=`pwd`/$(echo $item | cut -d . -f 1)
  echo $(date +"%T") > ${DIR_NAME}.time
#   echo "||========= Pose Refinement =========||"
#   if [ ! -f ${DIR_NAME}/pcd/corrected_poses_refined.txt ]; then
#     pose_refinement \
#       "${DIR_NAME}/pcd" \
#       "${DIR_NAME}/pcd/corrected_poses.txt" \
#       "${DIR_NAME}/pcd/corrected_poses_refined.txt"
#   fi
  # echo $(date +"%T") >> ${DIR_NAME}.time
#做一个简单地图
#   echo "||========= Create Lossless Map =========||"
#   if [ $i = ${#BAG_ARR[*]} ]; then
#     apply_to_map=true
#   fi
#   create_pointcloud_map "${DIR_NAME}/pcd" "${DIR_NAME}/pcd/corrected_poses.txt" $apply_to_map
#   echo $(date +"%T") >> ${DIR_NAME}.time
#   echo "||========= Create Color Pointcloud =========||"
#   mkdir -p $DIR_NAME/colored_pcd
#   color_pointcloud "${DIR_NAME}/pcd/corrected_poses.txt" \
#                    "${DIR_NAME}/image/" \
#                    "${DIR_NAME}/image/corrected_poses.txt" \
#                    "${DIR_NAME}/colored_pcd/"
#   echo "||========= Create Color Map =========||"
#   create_color_map "${DIR_NAME}/colored_pcd/" \
#                    "$MAP_FOLDER/pointcloud_map/config.xml" \
#                    "${DIR_NAME}/pcd/corrected_poses.txt" \
#                    "$MAP_FOLDER/pointcloud_map/"
#   echo $(date +"%T") >> ${DIR_NAME}.time
done

echo "Done."