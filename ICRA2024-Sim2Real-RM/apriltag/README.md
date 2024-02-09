AprilTag 3
==========

修改了 `tag25h9`的头文件与实现, 

修改了opencv_demo的family, 

创建了tag_example文件夹存放生成的tag,

 TODO: 精简apriltag项目为只有tag25h9族


Table of Contents
=================
- [Papers](#papers)
- [Install](#install)
- [Usage](#usage)
  - [Choosing a Tag Family](#choosing-a-tag-family)
  - [Getting Started with the Detector](#getting-started-with-the-detector)
    - [Python](#python)
    - [C](#c)
    - [Matlab](#matlab)
    - [Julia](#julia)

Papers
======
AprilTag is the subject of the following papers.

[AprilTag: A robust and flexible visual fiducial system](https://april.eecs.umich.edu/papers/details.php?name=olson2011tags)

[AprilTag 2: Efficient and robust fiducial detection](https://april.eecs.umich.edu/papers/details.php?name=wang2016iros)

[Flexible Layouts for Fiducial Tags](https://april.eecs.umich.edu/papers/details.php?name=krogius2019iros)

Install
=======

Officially only Linux operating systems are supported, although users have had success installing on Windows too.

The default installation will place headers in /usr/local/include and shared library in /usr/local/lib. It also installs a pkg-config script into /usr/local/lib/pkgconfig and will install a python wrapper if python3 is installed.

```
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target install
```
This will build shared (\*.so) libraries by default. If you need static (\*.a) libraries set `BUILD_SHARED_LIBS` to `OFF`:
```
cmake -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF
cmake --build build --target install
```

If you have Ninja (`sudo apt install ninja-build`) installed, you can use:
```
cmake -B build -GNinja -DCMAKE_BUILD_TYPE=Release
cmake --build build --target install
```
to generate and compile via the ninja build script. It will be much faster than with cmake's default Makefile generator.

You can omit `--target install` if you only want to use this locally without installing.


Usage
=====

## Choosing a Tag Family
For the vast majority of applications, the tagStandard41h12 family will be the correct choice. You can find the images for the tags in the [apriltag-imgs repo](https://github.com/AprilRobotics/apriltag-imgs). Scale up the images in your favorite editor and print them out.

Some heuristics for when to choose other tag families:
1. If you need more tags, use tagStandard52h13
2. If you need to maximize the use of space on a small circular object, use tagCircle49h12 (or tagCircle21h7).
3. If you want to make a recursive tag use tagCustom48h12.
4. If you want compatibility with the ArUcO detector use tag36h11

If none of these fit your needs, generate your own custom tag family [here](https://github.com/AprilRobotics/apriltag-generation).

## Getting Started with the Detector
### Python

    import cv2
    import numpy as np
    from apriltag import apriltag
    
    imagepath = 'test.jpg'
    image = cv2.imread(imagepath, cv2.IMREAD_GRAYSCALE)
    detector = apriltag("tagStandard41h12")
    
    detections = detector.detect(image)

Alternately you can use the AprilTag python bindings created by [duckietown](https://github.com/duckietown/apriltags3-py).

### C

    image_u8_t* im = image_u8_create_from_pnm("test.png");
    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_family_t *tf = tagStandard41h12_create();
    apriltag_detector_add_family(td, tf);
    zarray_t *detections = apriltag_detector_detect(td, im);
    
    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
    
        // Do stuff with detections here.
    }
    // Cleanup.
    apriltag_detections_destroy(detections);
    tagStandard41h12_destroy(tf);
    apriltag_detector_destroy(td);

