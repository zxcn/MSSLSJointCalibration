# MSSLS Joint Calibration

## Description

This project provides Matlab code and verification data for paper entitled "**A joint calibration method for monocular speckle structured light system**".

## Usage

Run `run_all.m` to begin.

| Code                                    | Description                                                                          |
| --------------------------------------- | ------------------------------------------------------------------------------------ |
| `run_all.m`                             | A script for runing all codes.                                                       |
| `run_circlegrid_detect_and_dic_match.m` | Detect circle grid pattern and perform DIC matching.                                 |
| `run_joint_optimization.m`              | Joint optimization.                                                                  |
| `run_depth_and_pointcloud_calc.m`       | Correct images and calculate depth and point cloud.                                  |
| `RefineCircleDetection.m`               | Function for refining circle grid pattern detection using homologous transformation. |
| `DicMatch.m`                            | Function for DIC matching.                                                           |
| `Zncc.m`                                | Function for calculating zero-normalized cross-correlation.                          |
| `SFExpressions.m`                       | Function for calculating shape function.                                             |
| `SubCorr.m`                             | Function for calculating sub-pixel correlation.                                      |
| `ReprojectionError.m`                   | Function for calculating reprojection error.                                         |
| `CorrectReferenceImage.m`               | Function for correcting refecence speckle image.                                     |
| `CorrectSceneImage.m`                   | Function for correcting scene speckle image.                                         |

## Data specification

 `./images/Sim`  gives simulation data to verify the correctness of our method. The simulation data are 16-bit images.  The simulation parameters are as follows.  

| Parameter           | Value                                       |
| ------------------- | ------------------------------------------- |
| circle grid cols    | 16                                          |
| circle grid rows    | 12                                          |
| circle grid spacing | 60                                          |
| $x_c,y_c,z_c$       | 75, 1, 1                                    |
| $A,B,D$             | 0.017465704563530, -0.0349207694917477, 750 |
| $f_x,f_y$           | 500, 500                                    |
| $c_x,c_y$           | 599.5, 449.5                                |
| $k_1,k_2,k_3$       | 0, 0, 0                                     |
| $p_1,p_2$           | 0, 0                                        |

`./images/Real` gives real data to verify the feasibility and accuracy of our method. The spacing of the circle grid pattern in `./image/Real/AccuracyTest/` is 20mm.

## License

Note that use of this code falls under the GNU GENERAL PUBLIC LICENSE Version 3. Furthermore, note that in the case of using this code for works related to publications (scientific or otherwise) requires citing of the source paper (citation given below).

## Cite

TODO

## Contact

jzx345@163.com

## Acknowledgement

We thank Yong Su et al. and Devan Atkinson et al. for their open source work on digital image correlation.

1. Su Y, Zhang Q. Glare: A free and open-source software for generation and assessment of digital speckle pattern[J]. Optics and Lasers in Engineering, 2022, 148: 106766.

2. https://gitee.com/yongsu1989/glare

3. Atkinson D, Becker T. A 117 line 2D digital image correlation code written in MATLAB[J]. Remote Sensing, 2020, 12(18): 2906.

4. https://github.com/SUMatEng/ADIC2D