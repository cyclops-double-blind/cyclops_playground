#!/bin/bash
dataset_directory="$base_directory/dataset/$dataset_name"
download_filetype="tar"

dataset_base_url="https://cdn2.vision.in.tum.de/tumvi/exported/euroc/512_16"
sequences=(
  corridor1,CR_01
  corridor2,CR_02
  corridor3,CR_03
  corridor4,CR_04
  corridor5,CR_05
)

function sequence_url() {
  local sequence=$(sequence_type $1)
  echo "${dataset_base_url}/dataset-$(sequence_type $1)_512_16.tar"
}

function sequence_unzip() {
  local filename=$1
  local directory=$2
  mkdir -p $directory
  tar xvf $filename -C $directory || exit 1
}

function sequence_compile() {
  local element=$1
  local basepath=$(sequence_basepath $element)
  local subdirectory="dataset-$(sequence_type $element)_512_16"

  python $base_directory/tools/compile_dataset.py \
    -g mocap0 -o $basepath.bag -d $basepath/$subdirectory/mav0 || exit 1
}
