#!/bin/bash
dataset_directory="$base_directory/dataset/$dataset_name"
download_filetype="zip"

dataset_base_url="http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset"
sequences=(
  machine_hall,MH_01_easy
  machine_hall,MH_02_easy
  machine_hall,MH_03_medium
  machine_hall,MH_04_difficult
  machine_hall,MH_05_difficult
  vicon_room1,V1_01_easy
  vicon_room1,V1_02_medium
  vicon_room1,V1_03_difficult
  vicon_room2,V2_01_easy
  vicon_room2,V2_02_medium
  vicon_room2,V2_03_difficult
)

function sequence_url() {
  local sequence=$(sequence_name $1)
  local baseurl=${dataset_base_url}/$(sequence_type $1)
  echo "${baseurl}/$sequence/$sequence.$download_filetype"
}

function sequence_unzip() {
  local filename=$1
  local directory=$2
  unzip -d $directory $filename || exit 1
}

function sequence_compile() {
  local element=$1
  local basepath=$(sequence_basepath $element)

  python $base_directory/tools/compile_dataset.py \
    -o $basepath.bag -d $basepath/mav0 || exit 1
}
