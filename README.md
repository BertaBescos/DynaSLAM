# DynaSLAM

[[Project]](https://bertabescos.github.io/DynaSLAM/)   [[Paper]](https://arxiv.org/pdf/1806.05620.pdf)

DynaSLAM is a visual SLAM system that is robust in dynamic scenarios for monocular, stereo and RGB-D configurations. Having a static map of the scene allows inpainting the frame background that has been occluded by such dynamic objects.

<img src="imgs/teaser.png" width="900px"/>

DynaSLAM: Tracking, Mapping and Inpainting in Dynamic Scenes   
[Berta Bescos Torcal](http://bertabescos.github.io), [José M. Fácil](http://webdiis.unizar.es/~jmfacil/), [Javier Civera](http://webdiis.unizar.es/~jcivera/) and [José Neira](http://webdiis.unizar.es/~jneira/)   
RA-L and IROS, 2018

We provide examples to run the SLAM system in the [TUM dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) as RGB-D or monocular, and in the [KITTI dataset](http://www.cvlibs.net/datasets/kitti/eval_odometry.php) as stereo or monocular.

## Setup
- Install ORB-SLAM2 dependencies (https://github.com/raulmur/ORB_SLAM2).
-- ff
- Install this Mask R-CNN implementation (https://github.com/matterport/Mask_RCNN).
- 

### ORB-SLAM2 Prerequisites
- C++11 or C++0x Compiler
- Pangolin
- OpenCV
- Eigen3
- DBoW2 and g2o (Included in Thirdparty folder)
For more information, go to https://github.com/raulmur/ORB_SLAM2.


# 1. License

If you use DynaSLAM in an academic work, please cite:

    @article{bescos2018dynaslam,
      title={{DynaSLAM}: Tracking, Mapping and Inpainting in Dynamic Environments},
      author={Bescos, Berta, F\'acil, JM., Civera, Javier and Neira, Jos\'e},
      journal={IEEE RA-L},
      year={2018}
     }


# 3. Building DynSLAM library and examples

Clone the repository:
```
git clone https://github.com/BertaBescos/DynSLAM.git DynSLAM
```

We provide a script `build.sh` to build the *Thirdparty* libraries and *ORB-SLAM2*. Please make sure you have installed all required dependencies (see section 2). Execute:
```
cd DynSLAM
chmod +x build.sh
./build.sh
```

This will create **libORB_SLAM2.so**  at *lib* folder and the executables **mono_tum**, **mono_kitti**, **rgbd_tum**, **stereo_kitti**, **mono_euroc** and **stereo_euroc** in *Examples* folder.

# 4. Monocular Examples

## TUM Dataset

1. Download a sequence from http://vision.in.tum.de/data/datasets/rgbd-dataset/download and uncompress it.

2. Execute the following command. Change `TUMX.yaml` to TUM1.yaml,TUM2.yaml or TUM3.yaml for freiburg1, freiburg2 and freiburg3 sequences respectively. Change `PATH_TO_SEQUENCE_FOLDER`to the uncompressed sequence folder.
```
./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt Examples/Monocular/TUMX.yaml PATH_TO_SEQUENCE_FOLDER
```

## KITTI Dataset  

1. Download the dataset (grayscale images) from http://www.cvlibs.net/datasets/kitti/eval_odometry.php 

2. Execute the following command. Change `KITTIX.yaml`by KITTI00-02.yaml, KITTI03.yaml or KITTI04-12.yaml for sequence 0 to 2, 3, and 4 to 12 respectively. Change `PATH_TO_DATASET_FOLDER` to the uncompressed dataset folder. Change `SEQUENCE_NUMBER` to 00, 01, 02,.., 11. 
```
./Examples/Monocular/mono_kitti Vocabulary/ORBvoc.txt Examples/Monocular/KITTIX.yaml PATH_TO_DATASET_FOLDER/dataset/sequences/SEQUENCE_NUMBER
```

# 5. Stereo Examples

## KITTI Dataset

1. Download the dataset (grayscale images) from http://www.cvlibs.net/datasets/kitti/eval_odometry.php 

2. Execute the following command. Change `KITTIX.yaml`to KITTI00-02.yaml, KITTI03.yaml or KITTI04-12.yaml for sequence 0 to 2, 3, and 4 to 12 respectively. Change `PATH_TO_DATASET_FOLDER` to the uncompressed dataset folder. Change `SEQUENCE_NUMBER` to 00, 01, 02,.., 11. 
```
./Examples/Stereo/stereo_kitti Vocabulary/ORBvoc.txt Examples/Stereo/KITTIX.yaml PATH_TO_DATASET_FOLDER/dataset/sequences/SEQUENCE_NUMBER
```

# 6. RGB-D Example

## TUM Dataset

1. Download a sequence from http://vision.in.tum.de/data/datasets/rgbd-dataset/download and uncompress it.

2. Associate RGB images and depth images using the python script [associate.py](http://vision.in.tum.de/data/datasets/rgbd-dataset/tools). We already provide associations for some of the sequences in *Examples/RGB-D/associations/*. You can generate your own associations file executing:

  ```
  python associate.py PATH_TO_SEQUENCE/rgb.txt PATH_TO_SEQUENCE/depth.txt > associations.txt
  ```

3. Execute the following command. Change `TUMX.yaml` to TUM1.yaml,TUM2.yaml or TUM3.yaml for freiburg1, freiburg2 and freiburg3 sequences respectively. Change `PATH_TO_SEQUENCE_FOLDER`to the uncompressed sequence folder. Change `ASSOCIATIONS_FILE` to the path to the corresponding associations file.

  ```
  ./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUMX.yaml PATH_TO_SEQUENCE_FOLDER ASSOCIATIONS_FILE
  ```

# DynSLAM
